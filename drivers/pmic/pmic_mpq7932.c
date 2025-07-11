#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/pmic/mpq7932.h>
#include <zephyr/pmic/pmbus.h>

#define DT_DRV_COMPAT mps_mpq7932

#define PMBUS_OPERATION         0x01
#define PMBUS_VOUT_COMMAND      0x21
#define PMBUS_STATUS_WORD       0x79
#define PMBUS_CLEAR_FAULTS      0x03

#define MPQ7932_PAGE            0  
struct mpq7932_config {
    struct i2c_dt_spec i2c;
};
struct mpq7932_data{};

static int mpq7932_init(const struct device *dev)
{
    const struct mpq7932_config *cfg = dev->config;
   if(!(device_is_ready(cfg->i2c.bus)))
	{
		printk("driver is loded but notready");
		return -EINVAL;
	}
   printk("device loaded succesfully=%d\n",cfg->i2c.addr);
   return 0;
}

static int mpq7932_control(const struct device *dev, enum mpq7932_cmd cmd, uint8_t page, void *data)
{
    const struct mpq7932_config *cfg = dev->config;
//    struct mpq7932_data *drv = dev->data;
    int ret = 0;
    printk("device address is =%d\n",cfg->i2c.addr);
    switch (cmd) {
    case MPQ7932_ENABLE:
        ret = pmbus_update_byte_data(cfg->i2c.bus, cfg->i2c.addr, page,
                                     PMBUS_OPERATION, 0x80, 0x80);
        break;
    case MPQ7932_DISABLE:
        ret = pmbus_update_byte_data(cfg->i2c.bus, cfg->i2c.addr, page,
                                     PMBUS_OPERATION, 0x80, 0x00);
        break;
    case MPQ7932_SET_VOLTAGE: 
        if (!data) {
		return -EINVAL;
	}
	printk("data is given successfully\n");
        float volts = *(float *)data;
        if (volts < 0.20625f) {
		printk("value is les\n");
		return -EINVAL;}
	printk("value is great\n");
        uint8_t code = (volts - 0.20625f) / 0.00625f;
	printk("ready to enter the write mode\n");
        ret = pmbus_write_byte_data(cfg->i2c.bus, cfg->i2c.addr, page,
                                    PMBUS_VOUT_COMMAND, code);
        break;
    case MPQ7932_GET_VOLTAGE: {
        if (!data) return -EINVAL;
        uint8_t code;
        ret = pmbus_read_byte_data(cfg->i2c.bus, cfg->i2c.addr, page,
                                   PMBUS_VOUT_COMMAND, &code);
        if (ret == 0){
            *(float *)data = 0.20625f + 0.00625f * code;
	    printk("vol get success\n");
	}
        printk("got no success\n");
        break;
    }
    case MPQ7932_CLEAR_FAULTS:
        ret = pmbus_clear_faults(cfg->i2c.bus, cfg->i2c.addr);
        break;
    case MPQ7932_READ_STATUS: {
        if (!data) return -EINVAL;
        ret = pmbus_read_word_data(cfg->i2c.bus, cfg->i2c.addr, page,
                                   PMBUS_STATUS_WORD, (uint16_t *)data);
        break;
    }
    default:
        return -ENOTSUP;
    }

    return ret;
}
static const struct mpq7932_func mpq7932_api ={
       .pmic_control = mpq7932_control,
};

#define MPQ7932_DEFINE(inst) \
    static const struct mpq7932_config mpq7932_cfg_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
    }; \
    static struct mpq7932_data mpq7932_data_##inst = { \
    }; \
    DEVICE_DT_INST_DEFINE(inst, \
        mpq7932_init, \
        NULL, \
        &mpq7932_data_##inst, \
        &mpq7932_cfg_##inst, \
        POST_KERNEL, \
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
        &mpq7932_api);

DT_INST_FOREACH_STATUS_OKAY(MPQ7932_DEFINE)
