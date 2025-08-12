/*
 * Copyright (c) 2024 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_adc12

#include <errno.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_mspm0);

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/regulator.h>
#include <soc.h>

/* Driverlib includes */
#include <ti/driverlib/dl_adc12.h>
#include <ti/driverlib/dl_gpio.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADC_MSPM0_MEMRES_MAX      (12)
#define ADC_MSPM0_CHANNEL_MAX     (ADC_SYS_NUM_ANALOG_CHAN)
#define ADC_MSPM0_CHANNEL_NO_INIT (0xFFFFFFFF)
#define POWER_STARTUP_DELAY	  16

/** Internal sample time unit conversion entry. */

struct adc_mspm0_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;

	int32_t sampleTime0;
	int32_t sampleTime1;
	uint32_t channelMemCtl[ADC_MSPM0_CHANNEL_MAX];

	uint32_t channels;
	uint8_t resolution;
	uint16_t oversampling;
	uint32_t channel_eoc;
};

struct adc_mspm0_cfg {
	uint32_t base;
	DL_ADC12_ClockConfig ADCClockConfig;
	const struct pinctrl_dev_config *pinctrl;
	void (*irq_cfg_func)(void);
//      const struct device * vref_config;
};


static void adc_mspm0_isr(const struct device *dev);

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_mspm0_data *data = CONTAINER_OF(ctx, struct adc_mspm0_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_mspm0_cfg *config = dev->config;

	data->repeat_buffer = data->buffer;

	DL_ADC12_clearInterruptStatus(
		(ADC12_Regs *)config->base,
		((1 << data->channel_eoc) << ADC12_CPU_INT_IMASK_MEMRESIFG0_OFS));
	DL_ADC12_enableInterrupt((ADC12_Regs *)config->base,
				 ((1 << data->channel_eoc) << ADC12_CPU_INT_IMASK_MEMRESIFG0_OFS));
	DL_ADC12_enableConversions((ADC12_Regs *)config->base);

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_get(dev);
#endif
	DL_ADC12_startConversion((ADC12_Regs *)config->base);
#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_put(dev);
#endif
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat)
{
	struct adc_mspm0_data *data = CONTAINER_OF(ctx, struct adc_mspm0_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	} else {
		data->buffer++;
	}
}

static int adc_mspm0_init(const struct device *dev)
{
	struct adc_mspm0_data *data = dev->data;
	const struct adc_mspm0_cfg *config = dev->config;
	int ret;

	LOG_DBG("Initializing %s", dev->name);

	data->dev = dev;

	/* Init GPIO */
	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("MSPM0 ADC pinctrl error (%d)", ret);
		return ret;
	}

	/* Init power */
	DL_ADC12_reset((ADC12_Regs *)config->base);
	DL_ADC12_enablePower((ADC12_Regs *)config->base);
	delay_cycles(POWER_STARTUP_DELAY);

	/* Configure clock */
	DL_ADC12_setClockConfig((ADC12_Regs *)config->base,
				(DL_ADC12_ClockConfig *)&config->ADCClockConfig);

	DL_ADC12_setPowerDownMode((ADC12_Regs *)config->base, DL_ADC12_POWER_DOWN_MODE_AUTO);

	/* Reset the sample time configuration */
	data->sampleTime0 = -1;
	data->sampleTime1 = -1;

	/* Initialize ADC channel configuration */
	for (int i = 0; i < ADC_MSPM0_CHANNEL_MAX; i++) {
		data->channelMemCtl[i] = ADC_MSPM0_CHANNEL_NO_INIT;
	}
	config->irq_cfg_func();

	adc_context_unlock_unconditionally(&data->ctx);

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_enable(dev);
#endif

	return 0;
}

static int adc_mspm0_validate_sampling_time(const struct device *dev, uint16_t acq_time)
{
	if (acq_time == ADC_ACQ_TIME_DEFAULT) {
		return 0;
	}

	/* Current implementation only supports configuration in ADC ticks.
	 *	The maximum value is limited by the hardware
	 */
	if ((ADC_ACQ_TIME_UNIT(acq_time) == ADC_ACQ_TIME_TICKS) &&
	    (ADC_ACQ_TIME_VALUE(acq_time) <= ADC12_SCOMP0_VAL_MASK)) {
		return ADC_ACQ_TIME_VALUE(acq_time);
	}

	LOG_ERR("Sampling time not supported.");
	return -EINVAL;
}

static int adc_mspm0_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	struct adc_mspm0_data *data = dev->data;
	const struct adc_mspm0_cfg *config = dev->config;
	const uint8_t ch = channel_cfg->channel_id;
	const struct adc_driver_api *api = (struct adc_driver_api *)dev->api;
	int samplingTime;
#if 0
	static uint32_t prev_vol = 0;
	uint32_t vrefInit = ((uint32_t)api->internal) * 1000;
#endif

	if (ch > ADC_MSPM0_CHANNEL_MAX) {
		LOG_ERR("Channel 0x%X is not supported, max 0x%X", ch, ADC_MSPM0_CHANNEL_MAX);
		return -EINVAL;
	}

	/* Initialize MEMCTL configuration to default */
	data->channelMemCtl[ch] = 0;

	samplingTime = adc_mspm0_validate_sampling_time(dev, channel_cfg->acquisition_time);
	if (samplingTime < 0) {
		return samplingTime;
	}

	/* Select one of the sampling timer registers */
	LOG_DBG("Setup %d sampling time %d", ch, samplingTime);
	if (data->sampleTime0 == -1) {
		DL_ADC12_setSampleTime0((ADC12_Regs *)config->base, samplingTime);
		data->sampleTime0 = samplingTime;
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0;
	} else if (data->sampleTime0 == samplingTime) {
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0;
	} else if (data->sampleTime1 == -1) {
		DL_ADC12_setSampleTime1((ADC12_Regs *)config->base, samplingTime);
		data->sampleTime1 = samplingTime;
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1;
	} else if (data->sampleTime0 == samplingTime) {
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1;
	} else {
		LOG_ERR("Two sampling times are supported by this ADC");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -EINVAL;
	}
#if 0
	if (config->vref_config && !prev_volts) {
		
		if (regulator_enable(config->vref_config) < 0)
			return -ENODEV;
	}
#endif	
	switch(channel_cfg->reference) {
	case ADC_REF_VDD_1:
		data->channelMemCtl[ch] |= DL_ADC12_REFERENCE_VOLTAGE_VDDA;
		break;
	
	case ADC_REF_INTERNAL:
		data->channelMemCtl[ch] |= DL_ADC12_REFERENCE_VOLTAGE_INTREF;
#if 0
		if(prev_volt != vref_init) {
		
		regulator_set_voltage(config->vref_config, vref_init , vref_init);
		prev_volt = vref_init;

		}
#endif
		break;
	
	case ADC_REF_EXTERNAL0:
		data->channelMemCtl[ch] |= DL_ADC12_REFERENCE_VOLTAGE_EXTREF;
		break;
	default:
		LOG_ERR("ADC voltage reference is not supported");
		return -EINVAL;
	}
	
	LOG_DBG("ADC Channel setup successful!");
	return 0;
}

static int adc_mspm0g3xx_configSequence(const struct device *dev)
{
	struct adc_mspm0_data *data = dev->data;
	const struct adc_mspm0_cfg *config = dev->config;
	uint32_t resolution_regVal;
	uint32_t avg_enabled_regVal, avg_acc_regVal, avg_div_regVal;
	uint32_t channels;
	uint32_t memCtl_count;
	uint8_t ch;
	int error = 0;

	resolution_regVal = 0;
	switch (data->resolution) {
	case 14:
		resolution_regVal |= DL_ADC12_SAMP_CONV_RES_12_BIT;
		break;
	case 12:
		resolution_regVal |= DL_ADC12_SAMP_CONV_RES_12_BIT;
		break;
	case 10:
		resolution_regVal |= DL_ADC12_SAMP_CONV_RES_10_BIT;
		break;
	case 8:
		resolution_regVal |= DL_ADC12_SAMP_CONV_RES_8_BIT;
		break;
	default:
		return -EINVAL;
	}

	avg_enabled_regVal = 0;
	switch (data->oversampling) {
	case 0:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_DISABLED;
		break;
	case 2:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_ENABLED;
		avg_acc_regVal = DL_ADC12_HW_AVG_NUM_ACC_2;
		avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_2;
		break;
	case 4:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_ENABLED;
		avg_acc_regVal = DL_ADC12_HW_AVG_NUM_ACC_4;
		avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_4;
		break;
	case 8:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_ENABLED;
		avg_acc_regVal = DL_ADC12_HW_AVG_NUM_ACC_8;
		avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_8;
		break;
	case 16:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_ENABLED;
		avg_acc_regVal = DL_ADC12_HW_AVG_NUM_ACC_16;
		avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_16;
		break;
	case 32:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_ENABLED;
		avg_acc_regVal = DL_ADC12_HW_AVG_NUM_ACC_32;
		avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_32;
		break;
	case 64:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_ENABLED;
		avg_acc_regVal = DL_ADC12_HW_AVG_NUM_ACC_64;
		avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_64;
		break;
	case 128:
		avg_enabled_regVal = DL_ADC12_AVERAGING_MODE_ENABLED;
		avg_acc_regVal = DL_ADC12_HW_AVG_NUM_ACC_128;
		if(data->resolution == 14){
			avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_32;
		}
		else{
			avg_div_regVal = DL_ADC12_HW_AVG_DEN_DIV_BY_128;
		}
		break;
	default:
		return -EINVAL;
	}

	channels = data->channels;
	memCtl_count = 0;
	while (channels) {
		ch = find_lsb_set(channels) - 1;
		if (ch >= ADC_MSPM0_CHANNEL_MAX) {
			LOG_ERR("ADC channel not available: %d", ch);
			return -EINVAL;
		}

		if (data->channelMemCtl[ch] == ADC_MSPM0_CHANNEL_NO_INIT) {
			LOG_ERR("ADC channel not initialized");
			return -EINVAL;
		}
		if (memCtl_count < ADC_MSPM0_MEMRES_MAX) {
			DL_ADC12_configConversionMem(
				(ADC12_Regs *)config->base, memCtl_count,
				(ch << ADC12_MEMCTL_CHANSEL_OFS),
				(data->channelMemCtl[ch] & ADC12_MEMCTL_VRSEL_MASK),
				(data->channelMemCtl[ch] & ADC12_MEMCTL_STIME_MASK),
				avg_enabled_regVal, DL_ADC12_BURN_OUT_SOURCE_DISABLED,
				DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
				DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
		} else {
			LOG_ERR("Number of conversions exceed ADC MEM registers");
			return -EINVAL;
		}
		memCtl_count++;
		channels &= ~BIT(ch);
	}

	if (avg_enabled_regVal == DL_ADC12_AVERAGING_MODE_ENABLED) {
		DL_ADC12_configHwAverage((ADC12_Regs *)config->base, avg_acc_regVal,
					 avg_div_regVal);
	} else {
		DL_ADC12_configHwAverage((ADC12_Regs *)config->base,
					 DL_ADC12_HW_AVG_NUM_ACC_DISABLED,
					 DL_ADC12_HW_AVG_DEN_DIV_BY_1);
	}

	if (memCtl_count - 1 != data->channel_eoc) {
		LOG_ERR("Configured ADC channels %d doesn't match requested %d", memCtl_count - 1,
			data->channel_eoc);
		return -EINVAL;
	}
	DL_ADC12_initSeqSample((ADC12_Regs *)config->base, DL_ADC12_REPEAT_MODE_ENABLED,
			       DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
			       DL_ADC12_SEQ_START_ADDR_00,
			       (data->channel_eoc) << ADC12_CTL2_ENDADD_OFS, resolution_regVal,
			       DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

	return error;
}

static int adc_mspm0_read_internal(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_mspm0_data *data = dev->data;
	size_t exp_size;
	int sequence_ret;
	int ch_count;

	if ((sequence->resolution != 14) && (sequence->resolution != 12) && (sequence->resolution != 10) &&
	    (sequence->resolution != 8)) {
		LOG_ERR("ADC resolution %d not supported. Only 8/10/12/14 bits.",
			sequence->resolution);
		return -EINVAL;
	}

	data->resolution = sequence->resolution;

	data->channels = sequence->channels;
	ch_count = POPCOUNT(data->channels);
	if (ch_count == 0) {
		LOG_ERR("No ADC channels selected");
		return -EINVAL;
	} else if (ch_count > ADC_MSPM0_MEMRES_MAX) {
		LOG_ERR("ADC implementation supports up to %d channels per sequence",
			ADC_MSPM0_MEMRES_MAX);
		return -EINVAL;
	}

	data->channel_eoc = ch_count - 1;

	exp_size = ch_count * sizeof(uint16_t);
	if (sequence->options) {
		exp_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < exp_size) {
		LOG_ERR("Required buffer size is %u, but %u got", exp_size, sequence->buffer_size);
		return -ENOMEM;
	}

	data->buffer = sequence->buffer;

	if(data->resolution == 14 && sequence->oversampling != 7){
		return -EINVAL;
	}

	if (sequence->oversampling > 7) {
		LOG_ERR("Oversampling out of range");
		return -EINVAL;
	}

	data->oversampling = sequence->oversampling ? BIT(sequence->oversampling) : 0;
	if (sequence->calibrate) {
		LOG_ERR("Calibration not supported");
		return -ENOTSUP;
	}

	sequence_ret = adc_mspm0g3xx_configSequence(dev);
	if (sequence_ret < 0) {
		LOG_ERR("Error in ADC sequence configuration");
		return sequence_ret;
	}

	adc_context_start_read(&data->ctx, sequence);
	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_mspm0_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_mspm0_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_mspm0_read_internal(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_mspm0_read_async(const struct device *dev, const struct adc_sequence *sequence,
				     struct k_poll_signal *async)
{
	struct adc_mspm0_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = adc_mspm0_read_internal(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif

static void adc_mspm0_isr(const struct device *dev)
{
	struct adc_mspm0_data *data = dev->data;
	const struct adc_mspm0_cfg *config = dev->config;
	int mem_ix;

	switch (DL_ADC12_getPendingInterrupt((ADC12_Regs *)config->base)) {
	case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM2_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM4_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM5_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM6_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM7_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM8_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM9_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM10_RESULT_LOADED:
	case DL_ADC12_IIDX_MEM11_RESULT_LOADED:
		for (mem_ix = 0; mem_ix <= data->channel_eoc; mem_ix++) {
			*data->buffer++ = DL_ADC12_getMemResult((ADC12_Regs *)config->base, mem_ix);
		}
		DL_ADC12_disableInterrupt(
			(ADC12_Regs *)config->base,
			((1 << (data->channel_eoc)) << ADC12_CPU_INT_IMASK_MEMRESIFG0_OFS));
		break;
	default:
		break;
	}
	adc_context_on_sampling_done(&data->ctx, dev);
}
#if 0
#ifdef CONFIG_PM_DEVICE
static int adc_mspm0_pm_action(const struct device *dev, enum pm_device_action action)
{
	if (initVREF != true)
		return 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		gVREFConfig.vrefEnable = DL_VREF_ENABLE_ENABLE;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		gVREFConfig.vrefEnable = DL_VREF_ENABLE_DISABLE;
		break;
	default:
		return -ENOTSUP;
	}

	DL_VREF_configReference(VREF, (DL_VREF_Config *)&gVREFConfig);

	return 0;
}
#endif
#endif

#define ADC_DT_CLOCK_SOURCE(x) DT_INST_PROP(x, ti_clk_source)

#define ADC_CLOCK_DIV(x)    DT_INST_PROP(x, ti_clk_divider)
#define ADC_DT_CLOCK_DIV(x) _CONCAT(DL_ADC12_CLOCK_DIVIDE_, ADC_CLOCK_DIV(x))

#define ADC_DT_CLOCK_RANGE(x) DT_INST_PROP(x, ti_clk_range)

#define MSPM0_ADC_INIT(index)                                                                 \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	PM_DEVICE_DT_INST_DEFINE(index, adc_mspm0_pm_action);			\
                                                                                                   \
	static void adc_mspm0_cfg_func_##index(void);                                         \
                                                                                                   \
	static const struct adc_mspm0_cfg adc_mspm0_cfg_##index = {                      \
		.base = DT_INST_REG_ADDR(index),                                                   \
		.irq_cfg_func = adc_mspm0_cfg_func_##index,                                   \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                  \
		.ADCClockConfig = {.clockSel = ADC_DT_CLOCK_SOURCE(index),                         \
				   .freqRange = ADC_DT_CLOCK_RANGE(index),                         \
				   .divideRatio = ADC_DT_CLOCK_DIV(index)},\
		.vref_config = DEVICE_DT_GET_OR(DT_PHANDLE(DT_DRV_INST(index), vref), NULL),		\
	};                                                                                          \
	static const struct adc_driver_api mspm0_driver_api##index = {                        \
		.channel_setup = adc_mspm0_channel_setup,                                     \
		.read = adc_mspm0_read,                                                       \
		.ref_internal = DT_INST_PROP(index, vref_mv),                                      \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_mspm0_read_async,))};        \
	static struct adc_mspm0_data adc_mspm0_data_##index = {                          \
		ADC_CONTEXT_INIT_TIMER(adc_mspm0_data_##index, ctx),                          \
		ADC_CONTEXT_INIT_LOCK(adc_mspm0_data_##index, ctx),                           \
		ADC_CONTEXT_INIT_SYNC(adc_mspm0_data_##index, ctx),\
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(index, &adc_mspm0_init, PM_DEVICE_DT_INST_GET(index), &adc_mspm0_data_##index,     \
			      &adc_mspm0_cfg_##index, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,  \
			      &mspm0_driver_api##index);                                      \
                                                                                                   \
	static void adc_mspm0_cfg_func_##index(void)                                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), adc_mspm0_isr, \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

DT_INST_FOREACH_STATUS_OKAY(MSPM0_ADC_INIT)
