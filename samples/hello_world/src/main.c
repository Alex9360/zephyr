#if 0
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pmic/mpq7932.h>
#define NODE DT_NODELABEL(pmic)
//#define NODE DEVICE_DT_GET(DT_NODELABEL(i2c1));
int main(void)
{
//	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
       float vol=10;
      const struct device *dev1 = DEVICE_DT_GET(NODE);
       if(!(device_is_ready(dev1))) {
		  printk("device is not ready");
		  return -1;
	 }
       const struct mpq7932_func *api = (const struct mpq7932_func *) dev1->api;
       if(!(api->pmic_control(dev1, MPQ7932_SET_VOLTAGE, 0x00, &vol)))
	      return -1; 
       printk("voltage is %d\n ",vol);
	return 0;
}
#endif
#if 0
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define PMIC_NODE DT_NODELABEL(pmic)

int main(void)
{
    printk("enters to the function\n");
    const struct device *i2c_dev = DEVICE_DT_GET(PMIC_NODE);

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return -1;
    }
   printk("device is ready\n");
    uint8_t val;
    int ret = i2c_reg_read_byte(i2c_dev, 0x4B, 0x11, &val);  // 0x4B = address, 0x11 = command/register

    if (ret == 0) {
        printk("Read value = 0x%02X\n", val);
    } else {
        printk("Failed to read from device (err %d)\n", ret);
    }

    return 0;
}
#endif

#if 0
#include <zephyr/drivers/i2c.h>

static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(DT_NODELABEL(pmic));

int main(void)
{
    if (!device_is_ready(i2c_dev.bus)) {
        printk("I2C bus not ready\n");
        return -1;
    }

    uint8_t val;
    int ret = i2c_reg_read_byte_dt(&i2c_dev, 0x11, &val);

    if (ret == 0) {
        printk("Register 0x11 = 0x%02X\n", val);
    } else {
        printk("Read failed: %d\n", ret);
    }

    return 0;
}
#endif

#if 0

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

static const struct smbus_dt_spec smbus_dev = SMBUS_DT_SPEC_GET(DT_NODELABEL(pmic));

int main(void)
{
    if (!device_is_ready(smbus_dev.bus)) {
        printk("SMBus device not ready\n");
        return -1;
    }

    uint8_t val;
    int ret = smbus_read_byte_data_dt(&smbus_dev, 0x11, &val);

    if (ret == 0) {
        printk("SMBus Register 0x11 = 0x%02X\n", val);
    } else {
        printk("SMBus read failed: %d\n", ret);
    }

    return 0;
}

#endif

#if 0
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>


static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(DT_NODELABEL(pmic));

int i2c_smbus_read_byte_data_dt(const struct i2c_dt_spec *spec, uint8_t command, uint8_t *val)
{
    return i2c_write_read_dt(spec, &command, 1, val, 1);
}
int main(void)
{
    if (!device_is_ready(i2c_dev.bus)) {
        printk("I2C bus not ready\n");
        return -1;
    }

    uint8_t val;
    int ret = i2c_smbus_read_byte_data_dt(&i2c_dev, 0x11, &val);

    if (ret == 0) {
        printk("SMBus (emulated) register 0x11 = 0x%02X\n", val);
    } else {
        printk("Read failed: %d\n", ret);
    }

    return 0;
}
#endif


#if 0
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

static const struct i2c_dt_spec pmbus_dev = I2C_DT_SPEC_GET(DT_NODELABEL(pmic));
int pmbus_read_byte_data_dt(const struct i2c_dt_spec *spec, uint8_t command, uint8_t *val);
int pmbus_write_byte_data_dt(const struct i2c_dt_spec *spec, uint8_t command, uint8_t val);

int pmbus_read_byte_data_dt(const struct i2c_dt_spec *spec, uint8_t command, uint8_t *val)
{
    return i2c_write_read_dt(spec, &command, 1, val, 1);
}

int pmbus_write_byte_data_dt(const struct i2c_dt_spec *spec, uint8_t command, uint8_t val)
{
    uint8_t buf[2] = {command, val};
    return i2c_write_dt(spec, buf, sizeof(buf));
}

int main(void)
{
    if (!device_is_ready(pmbus_dev.bus)) {
        printk("PMBus device not ready\n");
        return -1;
    }

    uint8_t val;
    int ret = pmbus_read_byte_data_dt(&pmbus_dev, 0x11, &val);

    if (ret == 0) {
        printk("PMBus Read: Reg 0x11 = 0x%02X\n", val);
    } else {
        printk("PMBus Read failed: %d\n", ret);
    }

    ret = pmbus_write_byte_data_dt(&pmbus_dev, 0x11, 0xA5);
    if (ret == 0) {
        printk("PMBus Write successful to Reg 0x11\n");
    } else {
        printk("PMBus Write failed: %d\n", ret);
    }

    return 0;
}
#endif
#if 0
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

static const struct i2c_dt_spec pmbus_dev = I2C_DT_SPEC_GET(DT_NODELABEL(pmic));

static int pmbus_read_byte_data(const struct i2c_dt_spec *spec, uint8_t command, uint8_t *val)
{
    return i2c_write_read_dt(spec, &command, 1, val, 1);
}

static int pmbus_write_byte_data(const struct i2c_dt_spec *spec, uint8_t command, uint8_t val)
{
	printf("call set \n");
    uint8_t buf[2] = { command, val };
    return i2c_write_dt(spec, buf, sizeof(buf));
}

static int pmbus_read_word_data(const struct i2c_dt_spec *spec, uint8_t command, uint16_t *val)
{
    uint8_t buf[2];
    int ret = i2c_write_read_dt(spec, &command, 1, buf, 2);
    if (ret == 0) {
        *val = ((uint16_t)buf[1] << 8) | buf[0];
    }
    return ret;
}

static int pmbus_write_word_data(const struct i2c_dt_spec *spec, uint8_t command, uint16_t val)
{
    uint8_t buf[3] = { command, val & 0xFF, (val >> 8) & 0xFF };
    return i2c_write_dt(spec, buf, sizeof(buf));
}

static int pmbus_set_page(const struct i2c_dt_spec *spec, uint8_t page)
{
    return pmbus_write_byte_data(spec, 0x00, page);
}

static int mpq7932_write_byte(const struct i2c_dt_spec *dev, uint8_t page, uint8_t reg, uint8_t val)
{
      int ret = pmbus_set_page(dev, page);
//       if (ret)
//	       printk("page return not succeed\n");
//	       return -1;
       return pmbus_write_byte_data(dev, reg, val);
}

int main(void)
{
    if (!device_is_ready(pmbus_dev.bus)) {
        printk("I2C bus not ready\n");
        return -1;
    }

    uint8_t val=10;
    int ret = mpq7932_write_byte(&pmbus_dev, 0x00, 0x00, val);

    if (ret == 0) {
        printk("SMBus (emulated) page 0x10 = 0x%02X\n", val);
    } else {
        printk("Read failed: %d\n", ret);
    }

    return 0;
}

#endif

/*
 * main.c - Test PMBus multi-page control for MPQ7932 in Zephyr
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "pmbus.h"

/* I2C Configuration */
#define I2C_DEV_NODE DT_NODELABEL(i2c1)
#define PMBUS_ADDR 0x50  /* Default MPQ7932 address (adjust as needed) */

/* Default voltages for each page (in mV) */
static const uint16_t default_voltages[MPQ7932_NUM_PAGES] = {
    1200,  /* Page 0 */
    1800,  /* Page 1 */
    3300,  /* Page 2 */
    1200,  /* Page 3 */
    2500,  /* Page 4 */
    3300   /* Page 5 */
};

void main(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
    
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }

    printk("PMBus Multi-Page Control Demo Started\n");

    /* 1. Initialize all pages with default voltages */
    for (uint8_t page = 0; page < MPQ7932_NUM_PAGES; page++) {
        /* Set voltage */
        int ret = pmbus_set_voltage(i2c_dev, PMBUS_ADDR, page, default_voltages[page]);
        if (ret < 0) {
            printk("Failed to set voltage on page %u (err %d)\n", page, ret);
            continue;
        }

        /* Enable output */
        ret = pmbus_enable_output(i2c_dev, PMBUS_ADDR, page);
        if (ret < 0) {
            printk("Failed to enable page %u (err %d)\n", page, ret);
        }
    }

    /* 2. Clear any existing faults */
    pmbus_clear_faults(i2c_dev, PMBUS_ADDR);
    printk("Cleared all PMBus faults\n");

    /* 3. Verify all voltages */
    while (1) {
        printk("\nCurrent Voltages:\n");
        
        for (uint8_t page = 0; page < MPQ7932_NUM_PAGES; page++) {
            int voltage = pmbus_read_voltage(i2c_dev, PMBUS_ADDR, page);
            if (voltage >= 0) {
                printk("Page %u: %d mV\n", page, voltage);
            } else {
                printk("Page %u read failed (err %d)\n", page, voltage);
            }
        }

        k_sleep(K_SECONDS(5));  /* Check every 5 seconds */
    }
}
