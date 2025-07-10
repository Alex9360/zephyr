#define DT_DRV_COMPAT mps_mpq7932
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include "pmbus.h"
#include "mpq7932.h"

#define MPQ7932_OP_ENABLE           0x80
#define MPQ7932_OP_DISABLE          0x00

#define MPQ7932_UV_MIN              0.20625f  
#define MPQ7932_UV_STEP             0.00625f 

#define MPQ7932_VOUT_MAX            0x24
#define MPQ7932_VOUT_MIN            0x2B

struct mpq7932_config {
	const struct i2c_dt_spec i2c;
};

static int mpq7932_init(const struct device *dev)
{
        const struct mpq7932_config *cfg = dev->config;
        if (!device_is_ready(cfg->i2c.bus)) {
                return -ENODEV;
        }
        return 0;
}

static int mpq7932_clear_faults(const struct device *dev)
{
        const struct mpq7932_config *cfg = dev->config;
        return pmbus_clear_faults(cfg->i2c.bus, cfg->i2c.addr);
}

static int mpq7932_read_vout(const struct device *dev, uint8_t page, float *vout)
{
        const struct mpq7932_config *cfg = dev->config;
        uint8_t code;
        int ret = pmbus_read_byte_data(cfg->i2c.bus, cfg->i2c.addr, page, PMBUS_VOUT_COMMAND, &code);
        if (ret) return ret;
        *vout = MPQ7932_UV_MIN + MPQ7932_UV_STEP * code;
        return 0;
}

static int mpq7932_set_vout(const struct device *dev, uint8_t page, float volts)
{
        const struct mpq7932_config *cfg = dev->config;
        if (volts < MPQ7932_UV_MIN) 
		return -EINVAL;
        uint8_t code = (volts - MPQ7932_UV_MIN) / MPQ7932_UV_STEP;
        return pmbus_write_byte_data(cfg->i2c.bus, cfg->i2c.addr, page, PMBUS_VOUT_COMMAND, code);
}

static int mpq7932_read_status_word(const struct device *dev, uint8_t page, uint16_t *status)
{
        const struct mpq7932_config *cfg = dev->config;
        return pmbus_read_word_data(cfg->i2c.bus, cfg->i2c.addr, page, PMBUS_STATUS_WORD, status);
}

static int mpq7932_set_vout_min(const struct device *dev, uint8_t page, float volts)
{
	const struct mpq7932_config *cfg = dev->config;
	if (volts < MPQ7932_UV_MIN) return -EINVAL;
	uint8_t code = (volts - MPQ7932_UV_MIN) / MPQ7932_UV_STEP;
	return pmbus_write_byte_data(cfg->i2c.bus, cfg->i2c.addr, page, PMBUS_VOUT_MIN, code);
}

static int mpq7932_set_vout_max(const struct device *dev, uint8_t page, float volts)
{
	const struct mpq7932_config *cfg = dev->config;
	if (volts < MPQ7932_UV_MIN) return -EINVAL;
	uint8_t code = (volts - MPQ7932_UV_MIN) / MPQ7932_UV_STEP;
	return pmbus_write_byte_data(cfg->i2c.bus, cfg->i2c.addr, page, PMBUS_VOUT_MAX, code);
}

static int mpq7932_output_enable(const struct device *dev, uint8_t page, bool state)
{
	const struct mpq7932_config *cfg = dev->config;
	const struct mask = 0x80;
        uint8_t mode = state ? MPQ7932_OP_ENABLE : MPQ7932_OP_DISABLE;
        return pmbus_update_byte_data(cfg->i2c.bus, cfg->i2c.addr, page,
                                         MPQ7932_CMD_OPERATION, mask, mode);
}

static const struct mpq7932_driver_api mpq7932_driver_func = {
	.read_vout = mpq7932_read_vout,
	.set_vout = mpq7932_set_vout,
	.read_status_word = mpq7932_read_status_word,
	.set_vout_min = mpq7932_set_vout_min,
	.set_vout_max = mpq7932_set_vout_max,
	.enable = mpq7932_output_enable,
};

#define MPQ7932_DEFINE(inst)\
        static const struct mpq7932_config mpq7932_cfg_##inst = {\
                .i2c = I2C_DT_SPEC_INST_GET(inst),\
        };\
        DEVICE_DT_INST_DEFINE(inst,\
                              mpq7932_init,\
                              NULL,\
                              NULL, \
                              &mpq7932_cfg_##inst,\
                              POST_KERNEL,\
                              CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
                              &mpq7932_driver_func);

DT_INST_FOREACH_STATUS_OKAY(MPQ7932_DEFINE)


