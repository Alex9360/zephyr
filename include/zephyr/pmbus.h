
#ifndef ZEPHYR_INCLUDE_DRIVERS_PMBUS_H_
#define ZEPHYR_INCLUDE_DRIVERS_PMBUS_H_

#include <zephyr/device.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/kernel.h>

#define PMBUS_PAGE           0x00
#define PMBUS_OPERATION       0x01

#define PMBUS_CLEAR_FAULTS   0x03
#define PMBUS_VOUT_MODE      0x20
#define PMBUS_VOUT_COMMAND   0x21

#define PMBUS_STATUS_BYTE    0x78
#define PMBUS_STATUS_WORD    0x79
#define PMBUS_VOUT_MAX       0x24
#define PMBUS_VOUT_MIN       0x2B

#define PMBUS_NO_PAGE        0xFF
#define PMBUS_MAX_BLOCK_LEN  255

static inline bool pmbus_check_byte_register(uint8_t reg);
static inline bool pmbus_check_word_register(uint8_t reg);

static inline int pmbus_set_page(const struct device *dev, uint8_t addr, uint8_t page)
{
    if (!device_is_ready(dev)) {
	printf("device is not ready\n");
        return -ENODEV;
    }
	printf("device is ready\n");
    return smbus_byte_data_write(dev, addr, PMBUS_PAGE, page);
}

/*
static inline int pmbus_set_phase(const struct device *dev, uint8_t addr, uint8_t phase)
{
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    return smbus_byte_data_write(dev, addr, PMBUS_PHASE, phase);
}
*/

static inline int pmbus_read_byte_data(const struct device *dev, uint8_t addr, uint8_t page,
                                       uint8_t reg, uint8_t *value)
{
    int ret;
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    if (!pmbus_check_byte_register(reg)) {
        return -EINVAL;
    }
    if (page != PMBUS_NO_PAGE) {
        ret = pmbus_set_page(dev, addr, page);
        if (ret) {
            return ret;
        }
    }
    return smbus_byte_data_read(dev, addr, reg, value);
}
static inline int pmbus_write_byte_data(const struct device *dev, uint8_t addr, uint8_t page,
                                        uint8_t reg, uint8_t val)
{
    int ret;
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    if (!pmbus_check_byte_register(reg)) {
        return -EINVAL;
    }
    if (page != PMBUS_NO_PAGE) {
        ret = pmbus_set_page(dev, addr, page);
        if (ret) {
            return ret;
        }
    }
    return smbus_byte_data_write(dev, addr, reg, val);
}

static inline int pmbus_update_byte_data(const struct device *dev, uint8_t addr, uint8_t page,
                                         uint8_t reg, uint8_t mask, uint8_t val)
{
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    if (!pmbus_check_byte_register(reg)) {
        return -EINVAL;
    }
    uint8_t orig;
    int ret = pmbus_read_byte_data(dev, addr, page, reg, &orig);
    if (ret) {
        return ret;
    }
    uint8_t updated = (orig & ~mask) | (val & mask);
    if (updated != orig) {
        ret = pmbus_write_byte_data(dev, addr, page, reg, updated);
    }
    return ret;
}
static inline int pmbus_read_word_data(const struct device *dev, uint8_t addr, uint8_t page,
                                       uint8_t reg, uint16_t *val)
{
    int ret;
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    if (!pmbus_check_word_register(reg)) {
        return -EINVAL;
    }
    if (page != PMBUS_NO_PAGE) {
        ret = pmbus_set_page(dev, addr, page);
        if (ret) {
            return ret;
        }
    }
    return smbus_word_data_read(dev, addr, reg, val);
}
static inline int pmbus_write_word_data(const struct device *dev, uint8_t addr, uint8_t page,
                                        uint8_t reg, uint16_t word)
{
    int ret;
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    if (!pmbus_check_word_register(reg)) {
        return -EINVAL;
    }
    if (page != PMBUS_NO_PAGE) {
         ret = pmbus_set_page(dev, addr, page);
        if (ret) {
            return ret;
        }
    }
    return smbus_word_data_write(dev, addr, reg, word);
}
static inline int pmbus_read_block_data(const struct device *dev, uint8_t addr, uint8_t page,
                                        uint8_t reg, uint8_t *data, size_t *len)
{
    int ret;
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    if (!pmbus_check_byte_register(reg) || *len > PMBUS_MAX_BLOCK_LEN) {
        return -EINVAL;
    }
    if (page != PMBUS_NO_PAGE) {
        ret = pmbus_set_page(dev, addr, page);
        if (ret) {
            return ret;
        }
    }
    return smbus_block_read(dev, addr, reg, len, data);
}

static inline int pmbus_write_byte(const struct device *dev, uint8_t addr,
                                   uint8_t page, uint8_t value)
{
	int ret;
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    if (page != PMBUS_NO_PAGE) {
        ret = pmbus_set_page(dev, addr, page);
        if (ret) {
            return ret;
        }
    }
    return smbus_byte_write(dev, addr, value);
}

static inline int pmbus_clear_faults(const struct device *dev, uint8_t addr)
{
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    return smbus_byte_write(dev, addr, PMBUS_CLEAR_FAULTS);
}

static inline bool pmbus_check_byte_register(uint8_t reg)
{
    return reg <= 0xFF; 
}

static inline bool pmbus_check_word_register(uint8_t reg)
{
    return reg >= 0x20 && reg <= 0x8F;
}
/*
static inline float pmbus_linear11_to_float(uint16_t val)
{
	int16_t raw = (int16_t)val;
	int exponent = raw >> 11;
	int mantissa = raw & 0x7FF;
	if (mantissa & 0x400) mantissa |= 0xF800; // sign extend mantissa
	return mantissa * powf(2, exponent);
}

static inline uint16_t pmbus_float_to_linear11(float value)
{
	for (int exp = -15; exp < 16; ++exp) {
		int mant = value / powf(2, exp);
		if (mant >= -1024 && mant <= 1023) {
			return ((exp & 0x1F) << 11) | (mant & 0x7FF);
		}
	}
	return 0;
}
*/
static inline int pmbus_update_byte_data(const struct device *dev, uint8_t addr, uint8_t page,
                                         uint8_t reg, uint8_t mask, uint8_t value)
{
        uint8_t orig;
        int ret = pmbus_read_byte_data(dev, addr, page, reg, &orig);
        if (ret) return ret;

        uint8_t updated = (orig & ~mask) | (value & mask);
        return pmbus_write_byte_data(dev, addr, page, reg, updated);
}


#endif 
