#ifndef ZEPHYR_DRIVERS_PMIC_MPQ7932_H_
#define ZEPHYR_DRIVERS_PMIC_MPQ7932_H_

#include <zephyr/device.h>
#include <stdint.h>

enum mpq7932_cmd {
    MPQ7932_ENABLE,
    MPQ7932_DISABLE,
    MPQ7932_SET_VOLTAGE,      
    MPQ7932_GET_VOLTAGE,      
    MPQ7932_CLEAR_FAULTS,
    MPQ7932_READ_STATUS       
};

struct mpq7932_func {
int (*pmic_control)(const struct device *dev, enum mpq7932_cmd cmd, uint8_t page, void *data);
};

#endif 

