/*
 * copyright (c) 2025 Linumiz GmbH
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT infineon_ifx_sar

#include <errno.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sar_infineon_cat1, CONFIG_ADC_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <cy_sar2.h>
#include <cy_sysclk.h>

#define MAX_CHANNELS 32
#define SAR_RESOLUTION 12

struct ifx_cat1_sar_config {
	PASS_SAR_Type *base;
	const struct pinctrl_dev_config *pinctrl;
	void (*irq_cfg_func)(const struct device *dev);
	uint32_t frequency;
	uint32_t clock_peri_group;
        uint32_t clock_id;
        uint8_t peri_div_type;
        uint8_t peri_div_type_inst;
};

struct ifx_cat1_sar_data {
	struct adc_context ctx;
	struct k_work work;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
//	cy_stc_sar2_channel_config_t cy_ch_cfg;
};

static int ifx_cat1_sar_channel_setup(const struct device *dev,
                                      const struct adc_channel_cfg *channel_cfg)
{
	const struct ifx_cat1_sar_config *config = dev->config;
	struct ifx_cat1_sar_data *data = dev->data;
	uint8_t channel_id = channel_cfg->channel_id;
	cy_stc_sar2_channel_config_t cy_ch_cfg;
	cy_en_sar2_status_t status;
	uint16_t sample_time = 0;

	if (channel_id >= MAX_CHANNELS) {
                return -EINVAL;
        }

	if (channel_cfg->gain != ADC_GAIN_1) {
                return -ENOTSUP;
        }

        if (channel_cfg->differential) {
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported reference source");
		return -ENOTSUP;
	}
#if 1
        cy_ch_cfg.channelHwEnable = true;
        cy_ch_cfg.triggerSelection = CY_SAR2_TRIGGER_OFF;
        cy_ch_cfg.channelPriority = 0U;
        cy_ch_cfg.preenptionType = CY_SAR2_PREEMPTION_FINISH_RESUME;
        cy_ch_cfg.isGroupEnd = true;
        cy_ch_cfg.doneLevel = CY_SAR2_DONE_LEVEL_LEVEL;
	/* channel_cfg->input_positive (pin number to given it as address) */
        cy_ch_cfg.pinAddress = channel_cfg->input_positive;

        cy_ch_cfg.portAddress = CY_SAR2_PORT_ADDRESS_SARMUX0;
        cy_ch_cfg.extMuxSelect = 0U;
        cy_ch_cfg.extMuxEnable = false;
        cy_ch_cfg.preconditionMode = CY_SAR2_PRECONDITION_MODE_OFF;
        cy_ch_cfg.overlapDiagMode = CY_SAR2_OVERLAP_DIAG_MODE_OFF;
	/* sample time needs to be recalculated */
  	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		switch (ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time)) {
		case ADC_ACQ_TIME_TICKS:
			sample_time = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);
			break;
		case ADC_ACQ_TIME_MICROSECONDS:
			sample_time = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time) * ((config->frequency) / 1000000);
			break;
		case ADC_ACQ_TIME_NANOSECONDS:
			/* FIXME formula has to be recalculated */
			sample_time = (ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time) + 49) % 50;
			break;
		default:
			LOG_ERR("Selected ADC acquisition time units is not valid");
			return -EINVAL;
		}
	}
	cy_ch_cfg.sampleTime = sample_time;
        cy_ch_cfg.calibrationValueSelect = CY_SAR2_CALIBRATION_VALUE_REGULAR;
        cy_ch_cfg.resultAlignment = CY_SAR2_RESULT_ALIGNMENT_RIGHT;

        cy_ch_cfg.signExtention = CY_SAR2_SIGN_EXTENTION_UNSIGNED;
        cy_ch_cfg.postProcessingMode = CY_SAR2_POST_PROCESSING_MODE_NONE;
        cy_ch_cfg.averageCount = 1U;
        cy_ch_cfg.rightShift = 0U;
        cy_ch_cfg.positiveReload = 0U;
        cy_ch_cfg.negativeReload = 0U;
        cy_ch_cfg.rangeDetectionMode = CY_SAR2_RANGE_DETECTION_MODE_BELOW_LO;
        cy_ch_cfg.rangeDetectionLoThreshold = 0U;
        cy_ch_cfg.rangeDetectionHiThreshold = 0xFFFU;
        cy_ch_cfg.interruptMask = 0U;
	
	status = Cy_SAR2_Channel_Init(config->base, channel_id, &cy_ch_cfg);
        if (status != CY_SAR2_SUCCESS) {
                return -EIO;
        }
#endif
#if 0
        data->cy_ch_cfg.channelHwEnable = true;
        data->cy_ch_cfg.triggerSelection = CY_SAR2_TRIGGER_OFF;
        data->cy_ch_cfg.channelPriority = 0U;
        data->cy_ch_cfg.preenptionType = CY_SAR2_PREEMPTION_FINISH_RESUME;
        data->cy_ch_cfg.isGroupEnd = false;
        data->cy_ch_cfg.doneLevel = CY_SAR2_DONE_LEVEL_LEVEL;
	/* channel_cfg->input_positive (pin number to given it as address) */
        data->cy_ch_cfg.pinAddress = channel_cfg->input_positive;

        data->cy_ch_cfg.portAddress = CY_SAR2_PORT_ADDRESS_SARMUX0;
        data->cy_ch_cfg.extMuxSelect = 0U;
        data->cy_ch_cfg.extMuxEnable = false;
        data->cy_ch_cfg.preconditionMode = CY_SAR2_PRECONDITION_MODE_OFF;
        data->cy_ch_cfg.overlapDiagMode = CY_SAR2_OVERLAP_DIAG_MODE_OFF;
	/* sample time needs to be recalculated */
  	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		switch (ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time)) {
		case ADC_ACQ_TIME_TICKS:
			sample_time = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);
			break;
		case ADC_ACQ_TIME_MICROSECONDS:
			sample_time = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time) * ((config->frequency) / 1000000);
			break;
		case ADC_ACQ_TIME_NANOSECONDS:
			/* FIXME formula has to be recalculated */
			sample_time = (ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time) + 49) % 50;
			break;
		default:
			LOG_ERR("Selected ADC acquisition time units is not valid");
			return -EINVAL;
		}
	}
	data->cy_ch_cfg.sampleTime = sample_time;
        data->cy_ch_cfg.calibrationValueSelect = CY_SAR2_CALIBRATION_VALUE_REGULAR;
        data->cy_ch_cfg.resultAlignment = CY_SAR2_RESULT_ALIGNMENT_RIGHT;

        data->cy_ch_cfg.signExtention = CY_SAR2_SIGN_EXTENTION_UNSIGNED;
        data->cy_ch_cfg.postProcessingMode = CY_SAR2_POST_PROCESSING_MODE_NONE;
        data->cy_ch_cfg.averageCount = 1U;
        data->cy_ch_cfg.rightShift = 0U;
        data->cy_ch_cfg.positiveReload = 0U;
        data->cy_ch_cfg.negativeReload = 0U;
        data->cy_ch_cfg.rangeDetectionMode = CY_SAR2_RANGE_DETECTION_MODE_BELOW_LO;
        data->cy_ch_cfg.rangeDetectionLoThreshold = 0U;
        data->cy_ch_cfg.rangeDetectionHiThreshold = 0xFFFU;
        data->cy_ch_cfg.interruptMask = 0U;
	
	status = Cy_SAR2_Channel_Init(config->base, channel_id, &data->cy_ch_cfg);
        if (status != CY_SAR2_SUCCESS) {
                return -EIO;
        }
	/* REFERENCE NEEDS TO SELECTED TODO */
//	 Cy_SAR2_SetReferenceBufferMode(PASS0_EPASS_MMIO, CY_SAR2_REF_BUF_MODE_ON); 
	LOG_INF("channel setup done");
#endif
        return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat)
{
	struct ifx_cat1_sar_data *data = CONTAINER_OF(ctx, struct ifx_cat1_sar_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	} else {
		data->buffer++;
	}
}

static void ifx_cat1_sar_isr(const struct device *dev)
{
	const struct ifx_cat1_sar_config *config = dev->config;
        struct ifx_cat1_sar_data *data = dev->data;
	LOG_INF("ISR OCCURED");
	uint32_t channels = data->ctx.sequence.channels;
	uint32_t last_channel = find_msb_set(channels)-1;
	uint32_t ch = 0;
	LOG_INF("CHANNEL IDX = %d",last_channel);
	uint32_t status = Cy_SAR2_Channel_GetInterruptStatus(config->base, last_channel);
	Cy_SAR2_Channel_ClearInterrupt(config->base, last_channel, CY_SAR2_INT_GRP_DONE);
	
	if (status & CY_SAR2_INT_GRP_DONE) {
		while((channels > 1) && (status & CY_SAR2_INT_GRP_DONE))
		{
			ch = find_lsb_set(channels)-1;
			*data->buffer++ = Cy_SAR2_Channel_GetResult(config->base, ch, NULL);
			channels &= ~BIT(ch);
		}
		*data->buffer = Cy_SAR2_Channel_GetResult(config->base, ch, NULL);
	}

	adc_context_on_sampling_done(&data->ctx, dev);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ifx_cat1_sar_data *data = CONTAINER_OF(ctx, struct ifx_cat1_sar_data, ctx);
	const struct device *dev = data->dev;
	const struct ifx_cat1_sar_config *config = dev->config;
	uint8_t last_ch = find_msb_set(ctx->sequence.channels)-1;
	uint8_t first_ch = find_lsb_set(ctx->sequence.channels)-1;
	uint32_t mask;
	data->repeat_buffer = data->buffer;
#if 0
        data->cy_ch_cfg.isGroupEnd = true;
	Cy_SAR2_Channel_Init(config->base, last_ch, &data->cy_ch_cfg);
#endif
	Cy_SAR2_Channel_ClearInterrupt(config->base, last_ch, CY_SAR2_INT_GRP_DONE);
	Cy_SAR2_Channel_SetInterruptMask(config->base, last_ch, CY_SAR2_INT_GRP_DONE);
	Cy_SAR2_Channel_SoftwareTrigger(config->base, first_ch);
	mask = Cy_SAR2_GetPendingStatus(config->base);
	LOG_INF("TOTAL CHANNEL = %d",ctx->sequence.channels);
	LOG_INF("FIRST CHANNEL = %d",first_ch);
	LOG_INF("LAST CHANNEL = %d",last_ch);
	LOG_INF("PENDING STATUS = %d",mask);
}

static int ifx_cat1_sar_read_internal(const struct device *dev,
				      const struct adc_sequence *sequence)
{
	struct ifx_cat1_sar_data *data = dev->data;
	size_t exp_size;
	uint8_t count = __builtin_popcount(sequence->channels);

	/* Validate resolution */
        if (sequence->resolution != SAR_RESOLUTION) {
                LOG_ERR("Unsupported resolution %u (only %u-bit supported)",
                        sequence->resolution, SAR_RESOLUTION);
                return -EINVAL;
        }

	/* Oversampling not supported in this driver */
        if (sequence->oversampling != 0) {
                LOG_ERR("Oversampling not supported");
                return -ENOTSUP;
        }
	
	if (count == 0) {
                LOG_ERR("No channels selected");
                return -EINVAL;
        }

	exp_size = count * sizeof(uint16_t);
	if (sequence->options) {
                exp_size *= (1 + sequence->options->extra_samplings);
        }

	if (sequence->buffer_size < exp_size) {
                LOG_ERR("Buffer too small: need %zu bytes, have %zu",
                        exp_size, sequence->buffer_size);
                return -ENOMEM;
        }

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);
	LOG_INF("READ STARTED AND WAIT FOR THE RESULT");
	return adc_context_wait_for_completion(&data->ctx);
}

static int ifx_cat1_sar_read(const struct device *dev,
                             const struct adc_sequence *sequence)
{
	struct ifx_cat1_sar_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);
	LOG_INF("READING WAIT");
	ret = ifx_cat1_sar_read_internal(dev, sequence);
	adc_context_release(&data->ctx, ret);
	LOG_INF("READING DONE");

	return ret;
}

#ifdef CONFIG_ADC_ASYNC
static int ifx_cat1_sar_read_async(const struct device *dev,
                                   const struct adc_sequence *sequence,
                                   struct k_poll_signal *async)
{
	struct ifx_cat1_sar_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, true, async);
	ret = ifx_cat1_sar_read_internal(dev, sequence);
	adc_context_release(&data->ctx, ret);
	
	return ret;
}
#endif

static int ifx_cat1_sar_hw_init(const struct device *dev)
{
	const struct ifx_cat1_sar_config *config = dev->config;
        cy_stc_sar2_config_t sar_cfg = {0};
        cy_en_sar2_status_t status;
	sar_cfg.preconditionTime = 2U;
        sar_cfg.powerupTime = 10U;
        sar_cfg.enableIdlePowerDown = true;
        sar_cfg.msbStretchMode = CY_SAR2_MSB_STRETCH_MODE_2CYCLE;
        sar_cfg.enableHalfLsbConv = true;
        sar_cfg.sarMuxEnable = true;
        sar_cfg.adcEnable = true;
        sar_cfg.sarIpEnable = true;
	status = Cy_SAR2_Init(config->base, &sar_cfg);
        if (status != CY_SAR2_SUCCESS) {
                return -EIO;
        }
	return 0;
}

static int ifx_clock_config(const struct device *dev, uint32_t target_freq)
{
    const struct ifx_cat1_sar_config *config = dev->config;
    uint32_t divider;
    uint32_t hf_clock_frequency;
    uint32_t actual_sar_clock;
    /* Validate frequency */
    
    if (target_freq == 0 ||
        target_freq < 2000000 ||
        target_freq > 26500000) {
        LOG_ERR("Invalid SAR frequency: %u", target_freq);
        return -EINVAL;
    }

    clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_hf2)),
                                 NULL, &hf_clock_frequency);
    
    LOG_DBG("HF clock frequency: %u Hz", hf_clock_frequency);

    divider = (hf_clock_frequency + (target_freq / 2)) / target_freq;

    if (divider < 2) {
        divider = 2;
    }

    actual_sar_clock = hf_clock_frequency / divider;

    LOG_INF("SAR Clock Config: target=%u, divider=%u, actual=%u",
            target_freq, divider, actual_sar_clock);

    Cy_SysClk_PeriPclkDisableDivider(config->clock_peri_group,
                                     config->peri_div_type,
                                     config->peri_div_type_inst);

    Cy_SysClk_PeriPclkSetDivider(config->clock_peri_group,
                                 config->peri_div_type,
                                 config->peri_div_type_inst,
                                 divider);
    Cy_SysClk_PeriPclkEnableDivider(config->clock_peri_group,
                                    config->peri_div_type,
                                    config->peri_div_type_inst);
    Cy_SysClk_PeriPclkAssignDivider(config->clock_id,
                                   config->peri_div_type,
                                   config->peri_div_type_inst);
    LOG_INF("CLOCK CONFIGURED SUCCESS");
    return 0;
}


static int ifx_cat1_sar_init(const struct device *dev) {
	
	const struct ifx_cat1_sar_config *config = dev->config;
	struct ifx_cat1_sar_data *data = dev->data;
	int ret = 0;

	LOG_INF("Initilizing infineon CAT1 SAR2 ADC");
#if 0
	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Pinctrl setup failed: %d", ret);
		return ret;
	}
#endif
	data->dev = dev;

	ret = ifx_clock_config(dev, config->frequency);
	if (ret < 0) {
		return ret;
	}

	config->irq_cfg_func(dev);
	ret = ifx_cat1_sar_hw_init(dev);
	if(ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(10));

	LOG_INF("ADC Intilized successfully");	
	adc_context_unlock_unconditionally(&data->ctx);
	return ret;
}

static DEVICE_API(adc, ifx_cat1_driver_api) = {
	.channel_setup = ifx_cat1_sar_channel_setup,
	.read	       = ifx_cat1_sar_read,
	.ref_internal  = 2300 /* TODO */
#ifdef CONFIG_ADC_SYNC
	.read_async    = ifx_cat1_sar_read_async,
#endif
};

#define IRQ_CONFIGURE(n, inst)                                                                     \
	enable_sys_int(DT_INST_PROP_BY_IDX(inst, system_interrupts, 0 + (n*2)),                    \
		       DT_INST_PROP_BY_IDX(inst, system_interrupts, 1 + (n*2)),                    \
	               (void (*)(const void *))(void *)ifx_cat1_sar_isr,                           \
	               DEVICE_DT_INST_GET(inst));                                                  \

#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_CONFIGURE, (), inst)

#define SAR_CAT1_INIT_FUNC(n)                                                                   \
        static void ifx_cat1_sar_irq_config_func_##n(const struct device *dev)                  \
 	{                                                                                       \
	       CONFIGURE_ALL_IRQS(n , MAX_CHANNELS);						\
	}
	
#define IFX_SAR_ADC_INIT(n)								       \
	PINCTRL_DT_INST_DEFINE(n);                                                             \
	SAR_CAT1_INIT_FUNC(n); 								       \
	static struct ifx_cat1_sar_data ifx_cat1_sar_data_##n = {                              \
		ADC_CONTEXT_INIT_TIMER(ifx_cat1_sar_data_##n, ctx),                            \
		ADC_CONTEXT_INIT_LOCK(ifx_cat1_sar_data_##n, ctx),                             \
		ADC_CONTEXT_INIT_SYNC(ifx_cat1_sar_data_##n, ctx),                             \
	};										       \
	static const struct ifx_cat1_sar_config ifx_cat1_sar_cfg_##n = {                       \
                .pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                  \
                .base = (PASS_SAR_Type *)DT_INST_REG_ADDR(n),                                  \
                .irq_cfg_func = ifx_cat1_sar_irq_config_func_##n,                              \
		.clock_peri_group = DT_INST_PROP(n, ifx_peri_group),                           \
		.frequency = DT_INST_PROP_OR(n, clock_frequency, 26500000),		       \
                .clock_id = DT_INST_PROP(n, ifx_peri_clk),                                     \
                .peri_div_type = DT_INST_PROP(n, ifx_peri_div),                                \
                .peri_div_type_inst = DT_INST_PROP(n, ifx_peri_div_inst),                      \
        };                                                                                     \
	DEVICE_DT_INST_DEFINE(n, &ifx_cat1_sar_init, NULL, &ifx_cat1_sar_data_##n,             \
			      &ifx_cat1_sar_cfg_##n, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,    \
			      &ifx_cat1_driver_api);                                           \

DT_INST_FOREACH_STATUS_OKAY(IFX_SAR_ADC_INIT)
