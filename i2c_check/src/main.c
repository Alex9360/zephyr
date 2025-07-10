#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/smbus.h>
#include "pmbus.h"

#define I2C_DEV     DT_LABEL(DT_NODELABEL(i2c1))  // or i2c0 based on board
#define PMIC_ADDR   0x60
#define TEST_PAGE   0

void main(void)
{
    const struct device *i2c_dev = device_get_binding(I2C_DEV);

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready!\n");
        return;
    }

    printk("Testing MPQ7932 via PMBus...\n");

    // Test: Set PAGE
    if (pmbus_set_page(i2c_dev, PMIC_ADDR, TEST_PAGE) == 0) {
        printk("PAGE set to %d\n", TEST_PAGE);
    } else {
        printk("Failed to set PAGE\n");
    }

    // Test: Read VOUT_MODE (0x20, byte register)
    uint8_t vout_mode;
    if (pmbus_read_byte_data(i2c_dev, PMIC_ADDR, TEST_PAGE, 0x20, &vout_mode) == 0) {
        printk("VOUT_MODE = 0x%02X\n", vout_mode);
    } else {
        printk("Failed to read VOUT_MODE\n");
    }

    // Test: Read STATUS_WORD (0x79, word register)
    uint16_t status;
    if (pmbus_read_word_data(i2c_dev, PMIC_ADDR, TEST_PAGE, 0x79, &status) == 0) {
        printk("STATUS_WORD = 0x%04X\n", status);
    } else {
        printk("Failed to read STATUS_WORD\n");
    }

    // Optional: Clear Faults
    if (pmbus_clear_faults(i2c_dev, PMIC_ADDR) == 0) {
        printk("Faults cleared.\n");
    } else {
        printk("Failed to clear faults\n");
    }
}

