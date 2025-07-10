#ifndef ZEPHYR_DRIVERS_PMIC_MPQ7932_H_
#define ZEPHYR_DRIVERS_PMIC_MPQ7932_H_

#include <zephyr/device.h>

enum mpq7932_margin_mode {
    MPQ7932_MARGIN_HIGH = 0x08,
    MPQ7932_MARGIN_LOW  = 0x10,
};

int mpq7932_set_margin(const struct device *dev, uint8_t page, enum mpq7932_margin_mode mode);
int mpq7932_clear_faults(const struct device *dev);
int mpq7932_read_status(const struct device *dev, uint8_t page, uint16_t *status);

#endif
