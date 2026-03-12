/*Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 *
 * TLV320AIC26 Audio Codec Driver
 * Control: SPI | Audio data: I2S
 */

#define DT_DRV_COMPAT ti_tlv320aic26

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "tlv320aic26.h"

LOG_MODULE_REGISTER(tlv320aic26, CONFIG_AUDIO_LOG_LEVEL);

/* ============================================================ */

struct tlv320aic26_config {
	struct spi_dt_spec spi;
	uint32_t mclk_freq;
};

struct tlv320aic26_data {
	bool     configured;
	uint32_t sample_rate;
	uint8_t  word_size;
};

/* ============================================================
 * SPI register access
 * ============================================================ */

static int aic26_write(const struct device *dev, uint8_t page,
		       uint8_t addr, uint16_t val)
{
	const struct tlv320aic26_config *cfg = dev->config;
	uint16_t cmd = AIC26_BUILD_CMD(AIC26_CMD_WRITE, page, addr);
	uint8_t buf[4];

	buf[0] = (cmd >> 8) & 0xFF;
	buf[1] = cmd & 0xFF;
	buf[2] = (val >> 8) & 0xFF;
	buf[3] = val & 0xFF;

	const struct spi_buf tx = { .buf = buf, .len = 4 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	int ret = spi_write_dt(&cfg->spi, &tx_set);

	if (ret < 0) {
		LOG_ERR("SPI write fail page%u reg0x%02x: %d", page, addr, ret);
	}
	return ret;
}

static int aic26_read(const struct device *dev, uint8_t page,
		      uint8_t addr, uint16_t *val)
{
	const struct tlv320aic26_config *cfg = dev->config;
	uint16_t cmd = AIC26_BUILD_CMD(AIC26_CMD_READ, page, addr);
	uint8_t tx_buf[4] = {0};
	uint8_t rx_buf[4] = {0};

	tx_buf[0] = (cmd >> 8) & 0xFF;
	tx_buf[1] = cmd & 0xFF;

	const struct spi_buf tx = { .buf = tx_buf, .len = 4 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf rx = { .buf = rx_buf, .len = 4 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);

	if (ret < 0) {
		LOG_ERR("SPI read fail page%u reg0x%02x: %d", page, addr, ret);
		return ret;
	}

	*val = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
	return 0;
}

static int aic26_update(const struct device *dev, uint8_t page,
			uint8_t addr, uint16_t mask, uint16_t val)
{
	uint16_t reg;
	int ret = aic26_read(dev, page, addr, &reg);

	if (ret < 0) {
		return ret;
	}
	reg = (reg & ~mask) | (val & mask);
	return aic26_write(dev, page, addr, reg);
}

/* ============================================================
 * Software reset
 * ============================================================ */

static int aic26_sw_reset(const struct device *dev)
{
	int ret = aic26_write(dev, AIC26_PAGE1, AIC26_P1_RESET, AIC26_RESET_VALUE);

	if (ret == 0) {
		k_msleep(10);
	}
	return ret;
}

/* ============================================================
 * PLL calculation
 * ============================================================ */

struct pll_cfg {
	bool     pll_en;
	uint8_t  p;
	uint8_t  j;
	uint16_t d;
	uint8_t  q;
	uint8_t  fs_div;
};

static int aic26_calc_pll(uint32_t mclk, uint32_t fs, struct pll_cfg *out)
{
	static const uint32_t fsref_list[] = { 48000, 44100 };
	static const uint8_t  div_x2[]    = { 2, 3, 4, 6, 8, 10, 11, 12 };

	memset(out, 0, sizeof(*out));

	for (int fi = 0; fi < ARRAY_SIZE(fsref_list); fi++) {
		uint32_t fsref = fsref_list[fi];

		for (int di = 0; di < ARRAY_SIZE(div_x2); di++) {
			if ((fsref * 2) / div_x2[di] != fs) {
				continue;
			}

			out->fs_div = di;

			/* Try without PLL: Fsref = MCLK / (128 * Q) */
			for (uint8_t q = 2; q <= 17; q++) {
				if (mclk / (128U * q) == fsref) {
					out->pll_en = false;
					out->q = q;
					return 0;
				}
			}

			/* Try with PLL: Fsref = MCLK*K / (2048*P) */
			for (uint8_t p = 1; p <= 8; p++) {
				uint64_t k10k = ((uint64_t)fsref * 2048 * p *
						 10000ULL) / mclk;
				uint32_t j = (uint32_t)(k10k / 10000);
				uint32_t d = (uint32_t)(k10k % 10000);
				uint32_t mclk_p = mclk / p;

				if (d != 0) {
					if (j < 4 || j > 11) continue;
					if (mclk_p < 10000000 ||
					    mclk_p > 20000000) continue;
				} else {
					if (j < 4 || j > 55) continue;
					if (mclk_p < 2000000 ||
					    mclk_p > 20000000) continue;
				}

				uint64_t fvco = (uint64_t)mclk * k10k / (p * 10000ULL);
				if (fvco < 80000000 || fvco > 110000000) continue;

				out->pll_en = true;
				out->p = p;
				out->j = (uint8_t)j;
				out->d = (uint16_t)d;
				return 0;
			}
		}
	}

	LOG_ERR("No valid PLL config for MCLK=%u Fs=%u", mclk, fs);
	return -EINVAL;
}

static int aic26_set_pll(const struct device *dev, const struct pll_cfg *pll)
{
	uint16_t r1b, r1c;
	int ret;

	if (pll->pll_en) {
		uint8_t p_enc = (pll->p == 8) ? 0 : pll->p;

		r1b = AIC26_PLLSEL_BIT |
		      ((p_enc << AIC26_PVAL_SHIFT) & AIC26_PVAL_MASK) |
		      ((pll->j << AIC26_JVAL_SHIFT) & AIC26_JVAL_MASK);
		r1c = ((uint16_t)pll->d << AIC26_DVAL_SHIFT) & AIC26_DVAL_MASK;
	} else {
		uint8_t q_enc;

		if (pll->q == 16) {
			q_enc = 0;
		} else if (pll->q == 17) {
			q_enc = 1;
		} else {
			q_enc = pll->q;
		}

		r1b = (q_enc << AIC26_QVAL_SHIFT) & AIC26_QVAL_MASK;
		r1c = 0;
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_PLL1, r1b);
	if (ret < 0) return ret;

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_PLL2, r1c);
	if (ret < 0) return ret;

	if (pll->pll_en) {
		k_msleep(10);
	}

	return 0;
}

/* ============================================================
 * Codec API
 * ============================================================ */

static int aic26_configure(const struct device *dev,
			   struct audio_codec_cfg *cfg)
{
	const struct tlv320aic26_config *dev_cfg = dev->config;
	struct tlv320aic26_data *data = dev->data;
	struct pll_cfg pll;
	uint16_t reg;
	int ret;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("Only I2S DAI supported");
		return -EINVAL;
	}

	/* Word length */
	uint8_t wlen;

	switch (cfg->dai_cfg.i2s.word_size) {
	case 16: wlen = AIC26_WLEN_16; break;
	case 20: wlen = AIC26_WLEN_20; break;
	case 24: wlen = AIC26_WLEN_24; break;
	case 32: wlen = AIC26_WLEN_32; break;
	default:
		LOG_ERR("Bad word size %u", cfg->dai_cfg.i2s.word_size);
		return -EINVAL;
	}

	/* Data format */
	uint8_t datfm;

	switch (cfg->dai_cfg.i2s.format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:              datfm = AIC26_DATFM_I2S; break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:   datfm = AIC26_DATFM_LJ;  break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:  datfm = AIC26_DATFM_RJ;  break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
	case I2S_FMT_DATA_FORMAT_PCM_LONG:         datfm = AIC26_DATFM_DSP; break;
	default:
		LOG_ERR("Bad data format");
		return -EINVAL;
	}

	data->sample_rate = cfg->dai_cfg.i2s.frame_clk_freq;
	data->word_size   = cfg->dai_cfg.i2s.word_size;

	/* PLL */
	ret = aic26_calc_pll(dev_cfg->mclk_freq, data->sample_rate, &pll);
	if (ret < 0) return ret;

	/* Software reset */
	ret = aic26_sw_reset(dev);
	if (ret < 0) return ret;

	/* PLL registers */
	ret = aic26_set_pll(dev, &pll);
	if (ret < 0) return ret;

	/* Audio Control 1 */
	reg = (wlen << AIC26_WLEN_SHIFT) |
	      (datfm << AIC26_DATFM_SHIFT) |
	      (pll.fs_div << AIC26_DACFS_SHIFT) |
	      (pll.fs_div << AIC26_ADCFS_SHIFT);

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_AUDIO_CTL1, reg);
	if (ret < 0) return ret;

	/* Audio Control 3: Fsref indicator + master/slave */
	reg = 0;

	if (data->sample_rate == 44100 || data->sample_rate == 22050 ||
	    data->sample_rate == 11025) {
		reg |= AIC26_REFFS_BIT;
	}

	if (cfg->dai_cfg.i2s.format & I2S_FMT_CLK_FORMAT_MASK) {
		reg |= AIC26_SLVMS_BIT;
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_AUDIO_CTL3, reg);
	if (ret < 0) return ret;

	/* DAV pin as data-available */
	ret = aic26_update(dev, AIC26_PAGE1, AIC26_P1_STATUS,
			   AIC26_DAV_MASK, (0x01 << AIC26_DAV_SHIFT));
	if (ret < 0) return ret;

	data->configured = true;
	LOG_INF("Configured: Fs=%u ws=%u fmt=%u", data->sample_rate,
		data->word_size, datfm);

	return 0;
}

static void aic26_start_output(const struct device *dev)
{
	uint16_t reg;
	int ret;

	ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
	if (ret < 0) return;

	/* Power up codec + DAC, high-power output drivers */
	reg &= ~(AIC26_PWDNC_BIT | AIC26_DAPWDN_BIT);
	reg |= AIC26_DAODRC_BIT;

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, reg);
	if (ret < 0) return;

	/* Wait for DAC power-up complete */
	for (int i = 0; i < 100; i++) {
		k_msleep(10);
		ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
		if (ret < 0) return;
		if (!(reg & AIC26_DAPWDN_BIT) && !(reg & AIC26_DAPWDF_BIT)) {
			break;
		}
	}

	/* Unmute both channels at 0 dB */
	aic26_write(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN, 0x0000);

	LOG_INF("DAC started");
}

static void aic26_stop_output(const struct device *dev)
{
	uint16_t reg;
	int ret;

	/* Mute both channels */
	aic26_write(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN,
		    AIC26_DALMU_BIT | (0x7F << AIC26_DALVL_SHIFT) |
		    AIC26_DARMU_BIT | 0x7F);

	/* Wait for soft-stepping */
	for (int i = 0; i < 200; i++) {
		k_msleep(5);
		ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_AUDIO_CTL2, &reg);
		if (ret < 0) return;
		if ((reg & BIT(3)) && (reg & BIT(2))) break;
	}

	/* Power down DAC */
	ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
	if (ret < 0) return;

	reg |= AIC26_DAPWDN_BIT;
	aic26_write(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, reg);

	LOG_INF("DAC stopped");
}

static void aic26_start_input(const struct device *dev)
{
	uint16_t reg;
	int ret;

	ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
	if (ret < 0) return;

	reg &= ~(AIC26_PWDNC_BIT | AIC26_ADPWDN_BIT);

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, reg);
	if (ret < 0) return;

	/* ADC input = single-ended MIC */
	aic26_update(dev, AIC26_PAGE2, AIC26_P2_AUDIO_CTL1,
		     AIC26_ADCIN_MASK, 0x00);

	/* Unmute ADC at 0 dB */
	aic26_write(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN, 0x0000);

	/* Wait for ADC power-up */
	for (int i = 0; i < 100; i++) {
		k_msleep(10);
		ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
		if (ret < 0) return;
		if (!(reg & AIC26_ADPWDN_BIT) && !(reg & AIC26_ADPWDF_BIT)) {
			break;
		}
	}

	LOG_INF("ADC started");
}

static void aic26_stop_input(const struct device *dev)
{
	uint16_t reg;
	int ret;

	/* Mute ADC */
	aic26_write(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN, AIC26_ADMUT_BIT);
	k_msleep(50);

	/* Power down ADC */
	ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
	if (ret < 0) return;

	reg |= AIC26_ADPWDN_BIT;
	aic26_write(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, reg);

	LOG_INF("ADC stopped");
}

static int aic26_set_property(const struct device *dev,
			      audio_property_t property,
			      audio_channel_t channel,
			      audio_property_value_t val)
{
	uint16_t reg;
	int ret;

	switch (property) {

	case AUDIO_PROPERTY_OUTPUT_VOLUME: {
		/* val.vol: dB, 0 to -63. AIC26: 0x00=0dB, 0x7F=-63.5dB (0.5dB steps) */
		int db = val.vol;

		if (db > 0) db = 0;
		if (db < -63) db = -63;

		uint8_t rv = (uint8_t)(-db * 2);

		if (rv > 0x7F) rv = 0x7F;

		ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN, &reg);
		if (ret < 0) return ret;

		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
			reg = (reg & ~(AIC26_DALMU_BIT | AIC26_DALVL_MASK)) |
			      ((uint16_t)rv << AIC26_DALVL_SHIFT);
			break;
		case AUDIO_CHANNEL_FRONT_RIGHT:
			reg = (reg & ~(AIC26_DARMU_BIT | AIC26_DARVL_MASK)) | rv;
			break;
		case AUDIO_CHANNEL_ALL:
			reg = ((uint16_t)rv << AIC26_DALVL_SHIFT) | rv;
			break;
		default:
			return -EINVAL;
		}
		return aic26_write(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN, reg);
	}

	case AUDIO_PROPERTY_OUTPUT_MUTE: {
		ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN, &reg);
		if (ret < 0) return ret;

		uint16_t lm = AIC26_DALMU_BIT;
		uint16_t rm = AIC26_DARMU_BIT;

		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
			reg = val.mute ? (reg | lm) : (reg & ~lm);
			break;
		case AUDIO_CHANNEL_FRONT_RIGHT:
			reg = val.mute ? (reg | rm) : (reg & ~rm);
			break;
		case AUDIO_CHANNEL_ALL:
			reg = val.mute ? (reg | lm | rm) : (reg & ~(lm | rm));
			break;
		default:
			return -EINVAL;
		}
		return aic26_write(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN, reg);
	}

	case AUDIO_PROPERTY_INPUT_VOLUME: {
		/* ADC PGA: 0-59.5 dB in 0.5 dB steps */
		int g = val.vol * 2;

		if (g < 0) g = 0;
		if (g > 119) g = 119;

		ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN, &reg);
		if (ret < 0) return ret;

		reg = (reg & ~(AIC26_ADMUT_BIT | AIC26_ADPGA_MASK)) |
		      ((uint16_t)g << AIC26_ADPGA_SHIFT);

		return aic26_write(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN, reg);
	}

	case AUDIO_PROPERTY_INPUT_MUTE:
		return aic26_update(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN,
				   AIC26_ADMUT_BIT,
				   val.mute ? AIC26_ADMUT_BIT : 0);

	default:
		return -ENOTSUP;
	}
}

static int aic26_apply_properties(const struct device *dev)
{
	return 0;
}

/* ============================================================
 * Init
 * ============================================================ */

static int aic26_init(const struct device *dev)
{
	const struct tlv320aic26_config *cfg = dev->config;
	struct tlv320aic26_data *data = dev->data;
	uint16_t reg;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Software reset */
	ret = aic26_sw_reset(dev);
	if (ret < 0) {
		LOG_ERR("Software reset failed: %d", ret);
		return ret;
	}

	/* Verify device responds */
	ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
	if (ret < 0) {
		LOG_ERR("Cannot communicate with codec: %d", ret);
		return ret;
	}
	LOG_DBG("Power control after reset: 0x%04X", reg);

	/* Internal reference 2.5V for auxiliary ADC */
	ret = aic26_write(dev, AIC26_PAGE1, AIC26_P1_REFERENCE,
			  AIC26_VREFM_BIT | AIC26_IREFV_BIT);
	if (ret < 0) return ret;

	data->configured = false;

	LOG_INF("TLV320AIC26 initialized (MCLK=%u Hz)", cfg->mclk_freq);
	return 0;
}

/* ============================================================
 * API + instantiation
 * ============================================================ */

static const struct audio_codec_api aic26_api = {
	.configure        = aic26_configure,
	.start_output     = aic26_start_output,
	.stop_output      = aic26_stop_output,
	.start_input      = aic26_start_input,
	.stop_input       = aic26_stop_input,
	.set_property     = aic26_set_property,
	.apply_properties = aic26_apply_properties,
};

#define TLV320AIC26_INIT(n)                                             \
	static struct tlv320aic26_data aic26_data_##n;                  \
	                                                                \
	static const struct tlv320aic26_config aic26_cfg_##n = {        \
		.spi = SPI_DT_SPEC_INST_GET(n,                          \
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |         \
			SPI_WORD_SET(8) | SPI_MODE_CPHA, 0),            \
		.mclk_freq = DT_INST_PROP(n, mclk_frequency),          \
	};                                                              \
	                                                                \
	DEVICE_DT_INST_DEFINE(n,                                        \
			      aic26_init, NULL,                          \
			      &aic26_data_##n, &aic26_cfg_##n,          \
			      POST_KERNEL,                              \
			      CONFIG_TLV320AIC26_INIT_PRIORITY,         \
			      &aic26_api);

DT_INST_FOREACH_STATUS_OKAY(TLV320AIC26_INIT)
