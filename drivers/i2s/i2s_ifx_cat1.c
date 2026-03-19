/*
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_cat1_i2s

#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <cy_i2s.h>

LOG_MODULE_REGISTER(i2s_ifx_cat1, CONFIG_I2S_LOG_LEVEL);

#define TX_QUEUE_SIZE  CONFIG_I2S_IFX_CAT1_TX_QUEUE_SIZE
#define RX_QUEUE_SIZE  CONFIG_I2S_IFX_CAT1_RX_QUEUE_SIZE

#define INTR_TX_ERRORS  (CY_I2S_INTR_TX_OVERFLOW | CY_I2S_INTR_TX_UNDERFLOW)
#define INTR_RX_ERRORS  (CY_I2S_INTR_RX_OVERFLOW | CY_I2S_INTR_RX_UNDERFLOW)
#define TX_MAX_BLOCK_SAMPLES  170U
#define RX_MAX_BLOCK_SAMPLES  255U

struct queue_item {
	void   *buffer;
	size_t  size;
};

struct i2s_stream {
	int32_t           state;
	struct i2s_config cfg;
	struct k_msgq     queue;
	void             *mem_block;
	size_t            mem_block_len;
	bool              xfer_pending;
	bool              last_block;
	bool              drain;
};

struct dma_channel {
	const struct device    *dev_dma;
	uint32_t                channel_num;
	struct dma_config       dma_cfg;
	struct dma_block_config blk_cfg;
};

struct ifx_i2s_data {
	struct i2s_stream   tx;
	struct i2s_stream   rx;
	struct dma_channel  dma_tx;
	struct dma_channel  dma_rx;
	cy_stc_i2s_config_t pdl_cfg;   /* accumulated HW config for both TX and RX */
	bool                tx_waiting_to_start;
	struct queue_item   tx_queue_buf[TX_QUEUE_SIZE];
	struct queue_item   rx_queue_buf[RX_QUEUE_SIZE];
};

struct ifx_i2s_config {
	I2S_Type                        *reg;
	const struct pinctrl_dev_config *pcfg;
	uint32_t                         clk_hz;
	void (*irq_config)(const struct device *dev);
};

static int  start_dma_tx_transfer(const struct device *dev);
static int  start_dma_rx_transfer(const struct device *dev);
static void i2s_tx_stream_disable(const struct device *dev, bool drop);
static void i2s_rx_stream_disable(const struct device *dev, bool drop);

static cy_en_i2s_len_t word_size_to_pdl_len(uint8_t ws)
{
	switch (ws) {
	case 8:  return CY_I2S_LEN8;
	case 16: return CY_I2S_LEN16;
	case 18: return CY_I2S_LEN18;
	case 20: return CY_I2S_LEN20;
	case 24: return CY_I2S_LEN24;
	default: return CY_I2S_LEN32;
	}
}

static uint32_t word_size_to_dma_bytes(uint8_t ws)
{
	if (ws <= 8U) {
		return 1U;
	} else if (ws <= 16U) {
		return 2U;
	}
	return 4U;
}

static int fmt_to_pdl_alignment(i2s_fmt_t fmt, cy_en_i2s_alignment_t *out)
{
	switch (fmt & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		*out = CY_I2S_I2S_MODE;
		return 0;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		*out = CY_I2S_LEFT_JUSTIFIED;
		return 0;
	default:
		return -EINVAL;
	}
}

static int compute_clk_div(uint32_t clk_hz, const struct i2s_config *cfg,
			    uint8_t *out)
{
	/* SCK = frame_clk_freq * channels * word_size */
//	uint32_t sck = cfg->frame_clk_freq * (uint32_t)cfg->channels * (uint32_t)cfg->word_size;
	uint32_t sck = cfg->frame_clk_freq * (uint32_t)cfg->channels * 32;
	uint32_t div;
	uint32_t clock;
	clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_hf5)), NULL, &clock);
	if (sck == 0U) {
		return -EINVAL;
	}
	div = clock / (sck * 8U);
	if (div < 1U || div > 64U) {
		LOG_ERR("Cannot achieve %uHz: CLK_HFx=%u div=%u (valid: 1..64)",
			cfg->frame_clk_freq, clock, div);
		return -EINVAL;
	}
	printf("sck = %d, clock = %d, div = %d\n",sck, clock, div);
	*out = (uint8_t)div;
	return 0;
}

static void queue_flush(struct i2s_stream *s)
{
	struct queue_item item;

	while (k_msgq_get(&s->queue, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(s->cfg.mem_slab, item.buffer);
	}
}

static int start_dma_tx_transfer(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream  *stream = &data->tx;
	struct dma_channel *dma = &data->dma_tx;
	struct queue_item item;
	int ret;

	ret = k_msgq_get(&stream->queue, &item, K_NO_WAIT);
	if (ret != 0) {
		/* Queue empty */
		if (stream->state == I2S_STATE_STOPPING) {
			stream->last_block = true;
			stream->drain = true;
		}
		/* Push dummy samples so TX FIFO stays alive until underflow ISR */
		for (int i = 0; i < 6; i++) {
			Cy_I2S_WriteTxData(cfg->reg, 0U);
		}
		return ret;
	}

	stream->mem_block     = item.buffer;
	stream->mem_block_len = item.size;

	dma->blk_cfg.source_address = (uint32_t)item.buffer;
	/* block_size is element count, not bytes (IFX DW DMA convention) */
	dma->blk_cfg.block_size = (uint32_t)item.size / dma->dma_cfg.source_data_size;

	ret = dma_config(dma->dev_dma, dma->channel_num, &dma->dma_cfg);
	if (ret < 0) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		return ret;
	}

	ret = dma_start(dma->dev_dma, dma->channel_num);
	if (ret < 0) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		return ret;
	}

	return ret;
}

static int start_dma_rx_transfer(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream  *stream = &data->rx;
	struct dma_channel *dma = &data->dma_rx;
	uint32_t mask;
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("RX: no free slab block");
		i2s_rx_stream_disable(dev, false);
		stream->state = I2S_STATE_ERROR;
		return ret;
	}
	stream->mem_block_len = stream->cfg.block_size;

	dma->blk_cfg.dest_address = (uint32_t)stream->mem_block;
	dma->blk_cfg.block_size   = (uint32_t)stream->mem_block_len /
				     dma->dma_cfg.source_data_size;

	ret = dma_config(dma->dev_dma, dma->channel_num, &dma->dma_cfg);
	if (ret < 0) {
		goto fail;
	}

	ret = dma_start(dma->dev_dma, dma->channel_num);
	if (ret < 0) {
		goto fail;
	}

	/* Disable RX_TRIGGER while DMA is draining; callback re-enables it */
	mask = Cy_I2S_GetInterruptMask(cfg->reg);
	Cy_I2S_SetInterruptMask(cfg->reg, mask & ~CY_I2S_INTR_RX_TRIGGER);
	return 0;

fail:
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;
	return ret;
}


static int i2s_tx_stream_start(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	int ret;

	data->tx_waiting_to_start = true;

	ret = start_dma_tx_transfer(dev);
	if (ret != 0) {
		LOG_ERR("TX start_dma failed: %d", ret);
		data->tx_waiting_to_start = false;
	}
	return ret;
}

static int i2s_rx_stream_start(const struct device *dev)
{
	const struct ifx_i2s_config *cfg = dev->config;
	uint32_t mask;

	mask = Cy_I2S_GetInterruptMask(cfg->reg);
	Cy_I2S_SetInterruptMask(cfg->reg,
				mask | CY_I2S_INTR_RX_TRIGGER | INTR_RX_ERRORS);
#if 0
	 Cy_SysInt_EnableSystemInt(55);
#endif
	Cy_I2S_EnableRx(cfg->reg);
	return 0;
}

static void i2s_tx_stream_disable(const struct device *dev, bool drop)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *stream = &data->tx;
	uint32_t mask;

	mask = Cy_I2S_GetInterruptMask(cfg->reg);
	Cy_I2S_SetInterruptMask(cfg->reg, mask & ~CY_I2S_INTR_TX_TRIGGER);

	Cy_I2S_DisableTx(cfg->reg);
#if 0
	 Cy_SysInt_DisableSystemInt(55);
#endif
	dma_stop(data->dma_tx.dev_dma, data->dma_tx.channel_num);

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	if (drop) {
		queue_flush(stream);
	}
}

static void i2s_rx_stream_disable(const struct device *dev, bool drop)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *stream = &data->rx;
	uint32_t mask;

	mask = Cy_I2S_GetInterruptMask(cfg->reg);
	Cy_I2S_SetInterruptMask(cfg->reg, mask & ~CY_I2S_INTR_RX_TRIGGER);

	Cy_I2S_DisableRx(cfg->reg);
#if 0
	 Cy_SysInt_DisableSystemInt(55);
#endif
	dma_stop(data->dma_rx.dev_dma, data->dma_rx.channel_num);

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	if (drop) {
		queue_flush(stream);
		Cy_I2S_ClearRxFifo(cfg->reg);
	}
}

#if 1
static void dma_tx_callback(const struct device *dma_dev, void *arg,
                             uint32_t channel, int status)
{
    const struct device *dev = arg;
    struct ifx_i2s_data *data = dev->data;
    const struct ifx_i2s_config *cfg = dev->config;
    struct i2s_stream *stream = &data->tx;
    uint32_t mask;

    ARG_UNUSED(dma_dev);
    ARG_UNUSED(channel);

    if (status < 0) {
        LOG_ERR("TX DMA error %d", status);
        if (stream->mem_block != NULL) {
            k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
            stream->mem_block = NULL;
        }
        stream->state = I2S_STATE_ERROR;
        return;
    }

    k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
    stream->mem_block = NULL;

    /* ALWAYS clear and re-enable TX_TRIGGER (matches TDM driver) */
    Cy_I2S_ClearInterrupt(cfg->reg, CY_I2S_INTR_TX_TRIGGER);
    mask = Cy_I2S_GetInterruptMask(cfg->reg);
    Cy_I2S_SetInterruptMask(cfg->reg, mask | CY_I2S_INTR_TX_TRIGGER);

    if (stream->xfer_pending) {
        stream->xfer_pending = false;
        (void)start_dma_tx_transfer(dev);
    }

    if (data->tx_waiting_to_start) {
        data->tx_waiting_to_start = false;
        Cy_I2S_ClearInterrupt(cfg->reg,
                    CY_I2S_INTR_TX_TRIGGER | INTR_TX_ERRORS);
        mask = Cy_I2S_GetInterruptMask(cfg->reg);
        Cy_I2S_SetInterruptMask(cfg->reg,
                    mask | CY_I2S_INTR_TX_TRIGGER | INTR_TX_ERRORS);
        Cy_I2S_EnableTx(cfg->reg);
    }
}
#endif

static void dma_rx_callback(const struct device *dma_dev, void *arg,
			     uint32_t channel, int status)
{
	const struct device *dev = arg;
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream *stream = &data->rx;
	struct queue_item item;
	uint32_t mask;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		LOG_ERR("RX DMA error %d", status);
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		stream->state = I2S_STATE_ERROR;
		return;
	}

	item.buffer     = stream->mem_block;
	item.size       = stream->mem_block_len;
	stream->mem_block = NULL;

	if (k_msgq_put(&stream->queue, &item, K_NO_WAIT) != 0) {
		LOG_ERR("RX queue full, dropping block");
		k_mem_slab_free(stream->cfg.mem_slab, item.buffer);
		stream->state = I2S_STATE_ERROR;
		return;
	}

	if (stream->last_block) {
		i2s_rx_stream_disable(dev, false);
		stream->state = I2S_STATE_READY;
		return;
	}

	if (stream->xfer_pending) {
		stream->xfer_pending = false;
		(void)start_dma_rx_transfer(dev);
		return;
	}

	mask = Cy_I2S_GetInterruptMask(cfg->reg);
	Cy_I2S_SetInterruptMask(cfg->reg, mask | CY_I2S_INTR_RX_TRIGGER);
}

static void tx_fifo_trigger_handler(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream *stream = &data->tx;
	uint32_t mask;

	switch (stream->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		mask = Cy_I2S_GetInterruptMask(cfg->reg);
		Cy_I2S_SetInterruptMask(cfg->reg, mask & ~CY_I2S_INTR_TX_TRIGGER);

		if (stream->mem_block == NULL) {
			if (stream->last_block) {
				for (int i = 0; i < 4; i++) {
					Cy_I2S_WriteTxData(cfg->reg, 0U);
				}
				stream->drain = true;
			} else {
				(void)start_dma_tx_transfer(dev);
			}
		} else {
			stream->xfer_pending = true;
		}
		break;

	case I2S_STATE_ERROR:
		i2s_tx_stream_disable(dev, false);
		break;

	default:
		LOG_ERR("TX trigger: unhandled state %d", stream->state);
		break;
	}
}

static void rx_fifo_trigger_handler(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *stream = &data->rx;

	switch (stream->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		if (stream->mem_block == NULL) {
			(void)start_dma_rx_transfer(dev);
		} else {
			stream->xfer_pending = true;
		}
		break;

	case I2S_STATE_ERROR:
		i2s_rx_stream_disable(dev, false);
		break;

	default:
		LOG_ERR("RX trigger: unhandled state %d", stream->state);
		break;
	}
}


static void i2s_isr(const struct device *dev)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	uint32_t intr;
	intr = Cy_I2S_GetInterruptStatusMasked(cfg->reg);

	if (intr & CY_I2S_INTR_TX_OVERFLOW) {
		LOG_ERR("TX overflow");
		data->tx.state = I2S_STATE_ERROR;
	}

	if (intr & CY_I2S_INTR_TX_UNDERFLOW) {
		i2s_tx_stream_disable(dev, false);
		if (data->tx.last_block && data->tx.drain) {
			data->tx.state = I2S_STATE_READY;
		} else {
			data->tx.state = I2S_STATE_ERROR;
		}
	}

	if (intr & CY_I2S_INTR_TX_TRIGGER) {
		tx_fifo_trigger_handler(dev);
	}

	if (intr & CY_I2S_INTR_RX_OVERFLOW) {
		LOG_ERR("RX overflow");
		data->rx.state = I2S_STATE_ERROR;
	}

	if (intr & CY_I2S_INTR_RX_UNDERFLOW) {
		LOG_ERR("RX underflow");
		data->rx.state = I2S_STATE_ERROR;
	}

	if (intr & CY_I2S_INTR_RX_TRIGGER) {
		if (data->rx.state == I2S_STATE_STOPPING) {
			Cy_I2S_DisableRx(cfg->reg);
		}
		rx_fifo_trigger_handler(dev);
	}

	Cy_I2S_ClearInterrupt(cfg->reg, intr);
}


static int ifx_i2s_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	bool is_tx = (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH);
	bool is_rx = (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH);
	cy_en_i2s_alignment_t alignment;
	cy_en_i2s_len_t word_len;
	uint32_t dma_data_bytes;
	uint32_t block_samples;
	bool master;
	uint8_t pdl_clk_div;
	int ret;

	if (is_tx && data->tx.state != I2S_STATE_NOT_READY &&
	    data->tx.state != I2S_STATE_READY) {
		LOG_ERR("TX: cannot configure in state %d", data->tx.state);
		return -EINVAL;
	}
	if (is_rx && data->rx.state != I2S_STATE_NOT_READY &&
	    data->rx.state != I2S_STATE_READY) {
		LOG_ERR("RX: cannot configure in state %d", data->rx.state);
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		if (is_tx) {
			data->pdl_cfg.txEnabled = false;
			data->tx.state = I2S_STATE_NOT_READY;
		}
		if (is_rx) {
			data->pdl_cfg.rxEnabled = false;
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return 0;
	}

	ret = fmt_to_pdl_alignment(i2s_cfg->format, &alignment);
	if (ret) {
		LOG_ERR("Unsupported format 0x%x", i2s_cfg->format);
		return ret;
	}

	if (i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) {
		return -ENOTSUP;
	}

	if (I2S_OPT_BIT_CLK_GATED & i2s_cfg->options) {
		LOG_ERR("Gated bit clock not supported");
		return -ENOTSUP;
	}

	if (I2S_OPT_PINGPONG & i2s_cfg->options) {
		LOG_ERR("Ping-pong not supported");
		return -ENOTSUP;
	}

	if (!!(i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) !=
	    !!(i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
		LOG_ERR("Bit and frame clock must be same master/slave");
		return -EINVAL;
	}

	if (i2s_cfg->channels != 2U) {
		LOG_ERR("Only stereo (2 channels) supported");
		return -EINVAL;
	}

	master         = !(i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE);
	word_len       = word_size_to_pdl_len(i2s_cfg->word_size);
	dma_data_bytes = word_size_to_dma_bytes(i2s_cfg->word_size);
	block_samples  = (uint32_t)i2s_cfg->block_size / dma_data_bytes;

	if (is_tx && (block_samples > TX_MAX_BLOCK_SAMPLES)) {
		LOG_ERR("TX block too large: %u samples (max %u)",
			block_samples, TX_MAX_BLOCK_SAMPLES);
		return -EINVAL;
	}
	if (is_rx && block_samples > RX_MAX_BLOCK_SAMPLES) {
		LOG_ERR("RX block too large: %u samples (max %u)",
			block_samples, RX_MAX_BLOCK_SAMPLES);
		return -EINVAL;
	}

	if (master) {
		ret = compute_clk_div(cfg->clk_hz, i2s_cfg, &pdl_clk_div);
		if (ret) {
			return ret;
		}
	} else {
		pdl_clk_div = data->pdl_cfg.clkDiv ? data->pdl_cfg.clkDiv : 2U;
	}

	data->pdl_cfg.clkDiv = pdl_clk_div;
	data->pdl_cfg.extClk = false;
	data->pdl_cfg.mclkEn = true;
	data->pdl_cfg.mclkDiv = CY_I2S_MCLK_DIV_8;

	if (is_tx) {
		data->pdl_cfg.txEnabled          = true;
		data->pdl_cfg.txDmaTrigger       = false; /* ISR-driven, no HW DMA trigger */
		data->pdl_cfg.txMasterMode       = master;
		data->pdl_cfg.txAlignment        = alignment;
		data->pdl_cfg.txWsPulseWidth     = CY_I2S_WS_ONE_CHANNEL_LENGTH;
		data->pdl_cfg.txWatchdogEnable   = false;
		data->pdl_cfg.txWatchdogValue    = 0U;
		data->pdl_cfg.txSdoLatchingTime  = false;
		data->pdl_cfg.txSckoInversion    = !!(i2s_cfg->format & I2S_FMT_BIT_CLK_INV);
		data->pdl_cfg.txSckiInversion    = !!(i2s_cfg->format & I2S_FMT_BIT_CLK_INV);
		data->pdl_cfg.txChannels         = 2U;
		data->pdl_cfg.txChannelLength    = CY_I2S_LEN32;
		//data->pdl_cfg.txChannelLength    = word_len;
		data->pdl_cfg.txWordLength       = word_len;
		data->pdl_cfg.txOverheadValue    = CY_I2S_OVHDATA_ZERO;
		data->pdl_cfg.txFifoTriggerLevel = (uint8_t)(block_samples / 2U);

		data->dma_tx.dma_cfg.source_data_size = dma_data_bytes;
		data->dma_tx.dma_cfg.dest_data_size   = dma_data_bytes;

		/* Flush stale TX queue */
		queue_flush(&data->tx);
		memcpy(&data->tx.cfg, i2s_cfg, sizeof(struct i2s_config));
	}

	if (is_rx) {
		data->pdl_cfg.rxEnabled          = true;
		data->pdl_cfg.rxDmaTrigger       = false;
		data->pdl_cfg.rxMasterMode       = master;
		data->pdl_cfg.rxAlignment        = alignment;
		data->pdl_cfg.rxWsPulseWidth     = CY_I2S_WS_ONE_CHANNEL_LENGTH;
		data->pdl_cfg.rxWatchdogEnable   = false;
		data->pdl_cfg.rxWatchdogValue    = 0U;
		data->pdl_cfg.rxSdiLatchingTime  = false;
		data->pdl_cfg.rxSckoInversion    = !!(i2s_cfg->format & I2S_FMT_BIT_CLK_INV);
		data->pdl_cfg.rxSckiInversion    = !!(i2s_cfg->format & I2S_FMT_BIT_CLK_INV);
		data->pdl_cfg.rxChannels         = 2U;
		data->pdl_cfg.rxChannelLength    = CY_I2S_LEN32;
		//data->pdl_cfg.rxChannelLength    = word_len;
		data->pdl_cfg.rxWordLength       = word_len;
		data->pdl_cfg.rxSignExtension    = false;
		data->pdl_cfg.rxFifoTriggerLevel = (uint8_t)(block_samples - 1U);

		data->dma_rx.dma_cfg.source_data_size = dma_data_bytes;
		data->dma_rx.dma_cfg.dest_data_size   = dma_data_bytes;

		queue_flush(&data->rx);
		memcpy(&data->rx.cfg, i2s_cfg, sizeof(struct i2s_config));
	}

	if (CY_I2S_SUCCESS != Cy_I2S_Init(cfg->reg, &data->pdl_cfg)) {
		LOG_ERR("Cy_I2S_Init failed");
		return -EIO;
	}
	Cy_I2S_SetInterruptMask(cfg->reg, INTR_TX_ERRORS | INTR_RX_ERRORS);

	if (is_tx) {
		data->tx.state = I2S_STATE_READY;
	}
	if (is_rx) {
		data->rx.state = I2S_STATE_READY;
	}

	return 0;
}

static const struct i2s_config *ifx_i2s_config_get(const struct device *dev,
						    enum i2s_dir dir)
{
	struct ifx_i2s_data *data = dev->data;

	if (dir == I2S_DIR_TX) {
		return (data->tx.state != I2S_STATE_NOT_READY) ? &data->tx.cfg : NULL;
	} else if (dir == I2S_DIR_RX) {
		return (data->rx.state != I2S_STATE_NOT_READY) ? &data->rx.cfg : NULL;
	}
	return NULL;
}

static int ifx_i2s_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *stream = &data->rx;
	struct queue_item item;
	int ret;

	if (stream->state == I2S_STATE_NOT_READY) {
		return -EIO;
	}

	ret = k_msgq_get(&stream->queue, &item, SYS_TIMEOUT_MS(stream->cfg.timeout));
	if (ret != 0) {
		return (stream->state == I2S_STATE_ERROR) ? -EIO : ret;
	}

	*mem_block = item.buffer;
	*size = item.size;
	return 0;
}

static int ifx_i2s_write(const struct device *dev, void *mem_block, size_t size)
{
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *stream = &data->tx;
	int ret;
	struct queue_item item = {
		.buffer = mem_block,
		.size   = size,
	};

	if (stream->state != I2S_STATE_RUNNING && stream->state != I2S_STATE_READY) {
		return -EIO;
	}

	ret = k_msgq_put(&stream->queue, &item, SYS_TIMEOUT_MS(stream->cfg.timeout));
	if (ret) {
		LOG_ERR("k_msgq_put failed %d", ret);
	}
	return ret;
}

static int ifx_i2s_trigger(const struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
	struct ifx_i2s_data *data = dev->data;
	bool do_tx = (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH);
	bool do_rx = (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH);
	struct i2s_stream *tx = &data->tx;
	struct i2s_stream *rx = &data->rx;
	unsigned int key;
	int ret = 0;

	key = irq_lock();

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (do_tx) {
			if (tx->state != I2S_STATE_READY) {
				ret = -EIO;
				break;
			}
			tx->xfer_pending = false;
			tx->last_block   = false;
			tx->drain        = false;
			ret = i2s_tx_stream_start(dev);
			if (ret < 0) {
				break;
			}
			tx->state = I2S_STATE_RUNNING;
		}
		if (do_rx) {
			if (rx->state != I2S_STATE_READY) {
				ret = -EIO;
				break;
			}
			rx->xfer_pending = false;
			rx->last_block   = false;
			rx->drain        = false;
			ret = i2s_rx_stream_start(dev);
			if (ret < 0) {
				break;
			}
			rx->state = I2S_STATE_RUNNING;
		}
		break;

	case I2S_TRIGGER_STOP:
		if (do_tx) {
			if (tx->state != I2S_STATE_RUNNING) {
				ret = -EIO;
				break;
			}
			tx->last_block = true;
			tx->drain      = true; /* ISR TX_UNDERFLOW checks both */
			tx->state      = I2S_STATE_STOPPING;
		}
		if (do_rx) {
			if (rx->state != I2S_STATE_RUNNING) {
				ret = -EIO;
				break;
			}
			rx->last_block = true;
			rx->state      = I2S_STATE_STOPPING;
		}
		break;

	case I2S_TRIGGER_DRAIN:
		if (do_tx) {
			if (tx->state != I2S_STATE_RUNNING) {
				ret = -EIO;
				break;
			}
			tx->drain = true;
			tx->state = I2S_STATE_STOPPING;
		}
		if (do_rx) {
			if (rx->state != I2S_STATE_RUNNING) {
				ret = -EIO;
				break;
			}
			rx->last_block = true;
			rx->state      = I2S_STATE_STOPPING;
		}
		break;

	case I2S_TRIGGER_DROP:
		if (do_tx) {
			if (tx->state == I2S_STATE_NOT_READY) {
				ret = -EIO;
				break;
			}
			i2s_tx_stream_disable(dev, true);
			tx->state = I2S_STATE_READY;
		}
		if (do_rx) {
			if (rx->state == I2S_STATE_NOT_READY) {
				ret = -EIO;
				break;
			}
			i2s_rx_stream_disable(dev, true);
			rx->state = I2S_STATE_READY;
		}
		break;

	case I2S_TRIGGER_PREPARE:
		if (do_tx) {
			if (tx->state != I2S_STATE_ERROR) {
				ret = -EIO;
				break;
			}
			i2s_tx_stream_disable(dev, true);
			tx->state = I2S_STATE_READY;
		}
		if (do_rx) {
			if (rx->state != I2S_STATE_ERROR) {
				ret = -EIO;
				break;
			}
			i2s_rx_stream_disable(dev, true);
			rx->state = I2S_STATE_READY;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	irq_unlock(key);
	return ret;
}

static int ifx_i2s_init(const struct device *dev)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(data->dma_tx.dev_dma) ||
	    !device_is_ready(data->dma_rx.dev_dma)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	k_msgq_init(&data->tx.queue, (char *)data->tx_queue_buf,
		    sizeof(struct queue_item), TX_QUEUE_SIZE);
	k_msgq_init(&data->rx.queue, (char *)data->rx_queue_buf,
		    sizeof(struct queue_item), RX_QUEUE_SIZE);

	data->dma_tx.blk_cfg.dest_address    = (uint32_t)(&cfg->reg->TX_FIFO_WR);
	data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_tx.dma_cfg.head_block       = &data->dma_tx.blk_cfg;
	data->dma_tx.dma_cfg.user_data        = (void *)dev;
	data->dma_tx.dma_cfg.dma_callback     = dma_tx_callback;

	data->dma_rx.blk_cfg.source_address  = (uint32_t)(&cfg->reg->RX_FIFO_RD);
	data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_rx.blk_cfg.dest_addr_adj   = DMA_ADDR_ADJ_INCREMENT;
	data->dma_rx.dma_cfg.head_block       = &data->dma_rx.blk_cfg;
	data->dma_rx.dma_cfg.user_data        = (void *)dev;
	data->dma_rx.dma_cfg.dma_callback     = dma_rx_callback;

	data->tx.state            = I2S_STATE_NOT_READY;
	data->rx.state            = I2S_STATE_NOT_READY;
	data->tx_waiting_to_start = false;

	cfg->irq_config(dev);
	Cy_I2S_SetInterruptMask(cfg->reg, 0);
	LOG_DBG("I2S %s initialized", dev->name);
	return 0;
}

static DEVICE_API(i2s, ifx_i2s_api) = {
	.configure  = ifx_i2s_configure,
	.config_get = ifx_i2s_config_get,
	.read       = ifx_i2s_read,
	.write      = ifx_i2s_write,
	.trigger    = ifx_i2s_trigger,
};

#define I2S_DMA_CHANNEL_INIT(idx, dir, ch_dir)                                \
	.dev_dma     = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, dir)),    \
	.channel_num = DT_INST_DMAS_CELL_BY_NAME(idx, dir, channel),          \
	.dma_cfg = {                                                          \
		.channel_direction   = ch_dir,                                \
		.source_burst_length = 0,                                     \
		.dest_burst_length   = 0,                                     \
		.block_count         = 1,                                     \
		.complete_callback_en = 1,                                    \
		.source_handshake    = 1,                                     \
	}

#define IFX_I2S_INIT(n)                                                       \
                                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                            \
                                                                              \
	static void ifx_i2s_irq_config_##n(const struct device *dev)          \
	{                                                                     \
		enable_sys_int(DT_INST_PROP_BY_IDX(n, system_interrupts, 0),  \
			    DT_INST_PROP_BY_IDX(n, system_interrupts, 1),     \
			    (void (*)(const void *)) i2s_isr,                 \
			    dev);                                             \
	}                                                                     \
                                                                              \
	static struct ifx_i2s_data i2s_data_##n = {                           \
		.dma_tx = { I2S_DMA_CHANNEL_INIT(n, tx, MEMORY_TO_PERIPHERAL) }, \
		.dma_rx = { I2S_DMA_CHANNEL_INIT(n, rx, PERIPHERAL_TO_MEMORY) }, \
		.tx_waiting_to_start = false,                                 \
	};                                                                    \
                                                                              \
	static const struct ifx_i2s_config i2s_config_##n = {                \
		.reg        = (I2S_Type *)DT_INST_REG_ADDR(n),               \
		.pcfg       = PINCTRL_DT_INST_DEV_CONFIG_GET(n),             \
		.irq_config = ifx_i2s_irq_config_##n,                        \
	};                                                                   \
                                                                             \
	DEVICE_DT_INST_DEFINE(n, &ifx_i2s_init, NULL,                        \
			      &i2s_data_##n, &i2s_config_##n,                \
			      POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,         \
			      &ifx_i2s_api);

DT_INST_FOREACH_STATUS_OKAY(IFX_I2S_INIT)
