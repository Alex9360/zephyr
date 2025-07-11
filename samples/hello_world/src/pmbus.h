/* pmbus_multipage.h - PMBus multi-page control for Zephyr using I2C/SMBus */

#ifndef PMBUS_MULTIPAGE_H
#define PMBUS_MULTIPAGE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* PMBus Standard Registers */
#define PMBUS_PAGE            0x00
#define PMBUS_OPERATION       0x01
#define PMBUS_CLEAR_FAULTS    0x03
#define PMBUS_VOUT_COMMAND    0x21
#define PMBUS_READ_VOUT       0x8B

/* MPQ7932-Specific Constants */
#define MPQ7932_NUM_PAGES     6
#define MPQ7932_VOUT_SCALE    160
#define MPQ7932_VOUT_OFFSET   (-33)

/**
 * @brief Selects a PMBus page
 * @param dev I2C device pointer
 * @param addr PMBus device address
 * @param page Page number (0 to 5 for MPQ7932)
 * @return 0 on success, negative errno on failure
 */
static inline int pmbus_select_page(const struct device *dev, uint8_t addr, uint8_t page) {
    uint8_t buf[2] = {PMBUS_PAGE, page};
    return i2c_write(dev, buf, sizeof(buf), addr);
}

/**
 * @brief Sets output voltage for a PMBus page
 * @param dev I2C device pointer
 * @param addr PMBus device address
 * @param page Page number (0-5)
 * @param millivolts Desired voltage (e.g., 1200 for 1.2V)
 * @return 0 on success, error code on failure
 */
static inline int pmbus_set_voltage(const struct device *dev, uint8_t addr, 
                                  uint8_t page, uint16_t millivolts) {
    int ret;
    uint8_t vout_cmd = (millivolts - MPQ7932_VOUT_OFFSET) / MPQ7932_VOUT_SCALE;
    uint8_t buf[2] = {PMBUS_VOUT_COMMAND, vout_cmd};

    if ((ret = pmbus_select_page(dev, addr, page)) < 0) {
        return ret;
    }
    return i2c_write(dev, buf, sizeof(buf), addr);
}

/**
 * @brief Reads output voltage from a PMBus page
 * @param dev I2C device pointer
 * @param addr PMBus device address
 * @param page Page number (0-5)
 * @return Voltage in mV (negative if error)
 */
static inline int pmbus_read_voltage(const struct device *dev, 
                                    uint8_t addr, uint8_t page) {
    int ret;
    uint8_t vout_raw;
    uint8_t reg = PMBUS_READ_VOUT;

    if ((ret = pmbus_select_page(dev, addr, page)) < 0) {
        return ret;
    }
    
    if ((ret = i2c_write_read(dev, addr, &reg, sizeof(reg), 
                             &vout_raw, sizeof(vout_raw))) < 0) {
        return ret;
    }
    
    return (MPQ7932_VOUT_SCALE * vout_raw) + MPQ7932_VOUT_OFFSET;
}

/**
 * @brief Clears faults on all PMBus pages
 * @param dev I2C device pointer
 * @param addr PMBus device address
 * @return 0 on success, error code on failure
 */
static inline int pmbus_clear_faults(const struct device *dev, uint8_t addr) {
    uint8_t cmd = PMBUS_CLEAR_FAULTS;
    return i2c_write(dev, &cmd, sizeof(cmd), addr);
}

/**
 * @brief Enables power output on a PMBus page
 * @param dev I2C device pointer
 * @param addr PMBus device address
 * @param page Page number (0-5)
 * @return 0 on success, error code on failure
 */
static inline int pmbus_enable_output(const struct device *dev, 
                                    uint8_t addr, uint8_t page) {
    uint8_t buf[2] = {PMBUS_OPERATION, 0x80}; /* Control ON */
    int ret;
    
    if ((ret = pmbus_select_page(dev, addr, page)) < 0) {
        return ret;
    }
    return i2c_write(dev, buf, sizeof(buf), addr);
}

#endif /* PMBUS_MULTIPAGE_H */
