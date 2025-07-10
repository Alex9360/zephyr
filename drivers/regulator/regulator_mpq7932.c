/*#define DT_DRV_COMPAT mps_mpq7932
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/pmbus.h>

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
/*
static int mpq7932_enable(const struct device *dev)
{
	const struct mpq7932_config *cfg = dev->cfg;
	uint8_t mask = 0x80;
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x00, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_ENABLE))
        	printk("buck0 is not enable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x01, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_ENABLE))
        	printk("buck1 is not enable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x02, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_ENABLE))
        	printk("buck2 is not enable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x03, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_ENABLE))
        	printk("buck3 is not enable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x04, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_ENABLE))
        	printk("buck4 is not enable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x05, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_ENABLE))
        	printk("buck5 is not enable\n");	
}
static int mpq7932_diable(const struct device *dev)
{
	const struct mpq7932_config *cfg = dev->cfg;
	uint8_t mask = 0x80;
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x00, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_DISABLE))
        	printk("buck0 is not disable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x01, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_DISABLE))
        	printk("buck1 is not disble\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x02, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_DISABLE))
        	printk("buck2 is not disable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x03, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_DISABLE))
        	printk("buck3 is not disable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x04, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_DISABLE))
        	printk("buck4 is not disable\n");	
	if(pmbus_write_byte(cfg->i2c.bus, cfg->i2c.addr, 0x05, MPQ7932_CMD_OPERATION, mask, MPQ7932_OP_DISABLE))
        	printk("buck5 is not disable\n");	
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
        if (ret) 
		return ret;
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

static DEVICE_API(regulator, mpq7932_api) = {
.enable = mpq7932_enable
.disable = mpq7932_disable
.set_volatge = mpq7932_set_vout
.get_voltage = mpq7932_read_vout
}

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
*/

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/pmbus.h>
#define DT_DRV_COMPAT mps_mpq7932


#define PMBUS_OPERATION         0x01
#define PMBUS_VOUT_COMMAND      0x21

enum mpq7932_reg_op {
    MPQ7932_OP_ENABLE,
    MPQ7932_OP_DISABLE,
    MPQ7932_OP_SET_VOLTAGE,
    MPQ7932_OP_GET_VOLTAGE,
};

struct mpq7932_config {
    struct i2c_dt_spec i2c;
};

struct mpq7932_data {
    uint8_t page;
    uint32_t min_uv;
    uint32_t max_uv;
};

static int mpq7932_regulator_ctrl(const struct device *dev,
                                   enum mpq7932_reg_op op,
                                   int32_t *uvolt)
{
    const struct mpq7932_config *cfg = dev->config;
    struct mpq7932_data *data = dev->data;
    int ret = 0;

    switch (op) {
    case MPQ7932_OP_ENABLE:
        ret = pmbus_update_byte_data(cfg->i2c.bus, cfg->i2c.addr, data->page,
                                     PMBUS_OPERATION, 0x80, 0x80);
        break;
    case MPQ7932_OP_DISABLE:
        ret = pmbus_update_byte_data(cfg->i2c.bus, cfg->i2c.addr, data->page,
                                     PMBUS_OPERATION, 0x80, 0x00);
        break;
    case MPQ7932_OP_SET_VOLTAGE:
        if (!uvolt) return -EINVAL;
        if (*uvolt < 600000) return -EINVAL;
        {
            uint8_t code = (*uvolt - 206250) / 6250;
            ret = pmbus_write_byte_data(cfg->i2c.bus, cfg->i2c.addr, data->page,
                                        PMBUS_VOUT_COMMAND, code);
        }
        break;
    case MPQ7932_OP_GET_VOLTAGE:
        if (!uvolt) return -EINVAL;
        {
            uint8_t code;
            ret = pmbus_read_byte_data(cfg->i2c.bus, cfg->i2c.addr, data->page,
                                       PMBUS_VOUT_COMMAND, &code);
            if (ret == 0) {
                *uvolt = 206250 + code * 6250;
            }
        }
        break;
    default:
        return -ENOTSUP;
    }

    return ret;
}

static int mpq7932_reg_enable(const struct device *dev)
{
    return mpq7932_regulator_ctrl(dev, MPQ7932_OP_ENABLE, NULL);
}

static int mpq7932_reg_disable(const struct device *dev)
{
    return mpq7932_regulator_ctrl(dev, MPQ7932_OP_DISABLE, NULL);
}

static int mpq7932_reg_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
    int32_t target = CLAMP(min_uv, 600000, max_uv);
    return mpq7932_regulator_ctrl(dev, MPQ7932_OP_SET_VOLTAGE, &target);
}

static int mpq7932_reg_get_voltage(const struct device *dev, int32_t *uvolt)
{
    return mpq7932_regulator_ctrl(dev, MPQ7932_OP_GET_VOLTAGE, uvolt);
}

static DEVICE_API(regulator, mpq7932_api) = {
    .enable = mpq7932_reg_enable,
    .disable = mpq7932_reg_disable,
    .set_voltage = mpq7932_reg_set_voltage,
    .get_voltage = mpq7932_reg_get_voltage,
};

static int mpq7932_reg_init(const struct device *dev)
{
    const struct mpq7932_config *cfg = dev->config;
    return device_is_ready(cfg->i2c.bus) ? 0 : -ENODEV;
}
/*
#define MPQ7932_DEFINE(inst) \
    static const struct mpq7932_config mpq7932_cfg_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
    }; \
    static struct mpq7932_data mpq7932_data_##inst = { \
        .page = DT_INST_PROP(inst, regulator_page), \
        .min_uv = DT_INST_PROP(inst, regulator_min_microvolt), \
        .max_uv = DT_INST_PROP(inst, regulator_max_microvolt), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, \
        mpq7932_reg_init, \
        NULL, \
        &mpq7932_data_##inst, \
        &mpq7932_cfg_##inst, \
        POST_KERNEL, \
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
        &mpq7932_reg_api);

DT_INST_FOREACH_STATUS_OKAY(MPQ7932_DEFINE)
*/

#define MPQ7932_REGULATOR_DEFINE(inst) \
    static const struct mpq7932_config mpq7932_cfg_##inst = { \
        .i2c = I2C_DT_SPEC_GET(inst), \
    }; \
    static struct mpq7932_data mpq7932_data_##inst = { \
        .page = DT_INST_PROP(inst, regulator_page), \
        .min_uv = DT_INST_PROP(inst, regulator_min_microvolt), \
        .max_uv = DT_INST_PROP(inst, regulator_max_microvolt), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, \
        mpq7932_reg_init, \
        NULL, \
        &mpq7932_data_##inst, \
        &mpq7932_cfg_##inst, \
        POST_KERNEL, \
        CONFIG_REGULATOR_INIT_PRIORITY, \
        &mpq7932_api);
DT_INST_FOREACH_STATUS_OKAY(MPQ7932_REGULATOR_DEFINE)






















