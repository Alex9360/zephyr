#ifndef ZEPHYR_INCLUDE_DRIVERS_PMBUS_H_
#define ZEPHYR_INCLUDE_DRIVERS_PMBUS_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/smbus.h>
#include <stdbool.h>

#define PMBUS_CMD_PAGE           0x00
#define PMBUS_CMD_PHASE          0x94
#define PMBUS_CMD_CLEAR_FAULTS   0x03

static inline int pmbus_set_page(const struct device *dev,
	       	uint8_t addr, uint8_t page)
{
	return smbus_byte_data_write(dev, addr, PMBUS_CMD_PAGE, page);
}

static inline int pmbus_read_byte_data(const struct device *dev, 
		uint8_t addr, uint8_t page, uint8_t reg, uint8_t *value)
{
	int ret = pmbus_set_page(dev, addr, page);
	if (ret) return ret;
	return smbus_byte_data_read(dev, addr, reg, value);
}

static inline int pmbus_write_byte_data(const struct device *dev, uint8_t addr,
	       	uint8_t page, uint8_t reg, uint8_t value)
{
	int ret = pmbus_set_page(dev, addr, page);
	if (ret) return ret;
	return smbus_byte_data_write(dev, addr, reg, value);
}

static inline int pmbus_read_word_data(const struct device *dev, uint8_t addr, 
		uint8_t page, uint8_t reg, uint16_t *value)
{
	int ret = pmbus_set_page(dev, addr, page);
	if (ret) return ret;
	return smbus_word_data_read(dev, addr, reg, value);
}

static inline int pmbus_write_word_data(const struct device *dev, uint8_t addr, 
		uint8_t page, uint8_t reg, uint16_t word)
{
	int ret = pmbus_set_page(dev, addr, page);
	if (ret) return ret;
	return smbus_word_data_write(dev, addr, reg, word);
}

static inline int pmbus_update_byte_data(const struct device *dev, uint8_t addr, uint8_t page,
					 uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t orig;
	int ret = pmbus_read_byte_data(dev, addr, page, reg, &orig);
	if (ret) return ret;

	uint8_t updated = (orig & ~mask) | (value & mask);
	return pmbus_write_byte_data(dev, addr, page, reg, updated);
}

static inline void pmbus_clear_faults(const struct device *dev, uint8_t addr)
{
	smbus_send_byte(dev, addr, PMBUS_CMD_CLEAR_FAULTS);
}

static inline int pmbus_combined_write(const struct device *dev, uint8_t addr,
                                       uint8_t page, uint8_t reg, uint8_t val)
{
	uint8_t buf[4] = { PMBUS_CMD_PAGE, page, reg, val };
	return i2c_write(dev, buf, sizeof(buf), addr);
}
static inline int pmbus_combined_read_byte(const struct device *dev, uint8_t addr,
                                           uint8_t page, uint8_t reg, uint8_t *val)
{
	uint8_t wbuf[3] = { PMBUS_CMD_PAGE, page, reg };
	return i2c_write_read(dev, addr, wbuf, sizeof(wbuf), val, 1);
}

static inline bool pmbus_check_byte_register(uint8_t reg)
{
	return (reg <= 0x7F);
}

static inline bool pmbus_check_word_register(uint8_t reg)
{
	return (reg >= 0x20 && reg <= 0x8F);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_PMBUS_H_ */
