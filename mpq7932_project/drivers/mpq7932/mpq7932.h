#ifndef ZEPHYR_DRIVERS_MPQ7932_H_
#define ZEPHYR_DRIVERS_MPQ7932_H_

#define MPQ7932_OP_ENABLE           0x80
#define MPQ7932_OP_DISABLE          0x00

#define MPQ7932_UV_MIN              0.20625f  
#define MPQ7932_UV_STEP             0.00625f 

#define MPQ7932_VOUT_MAX            0x24
#define MPQ7932_VOUT_MIN            0x2B

struct mpq7932_config {
        const struct i2c_dt_spec i2c;
};

struct mpq7932_driver_api {
	 int (*read_vout)(const struct device *dev, uint8_t page, float *vout);
	 int (*set_vout)(const struct device *dev, uint8_t page, float volts);
	 int (*read_status_word)(const struct device *dev, uint8_t page, uint16_t *status);
	 int (*set_vout_min)(const struct device *dev, uint8_t page, float volts);
	 int (*set_vout_max)(const struct device *dev, uint8_t page, float volts);
	 int (*enable)(const struct device *dev, uint8_t page, bool state);
}
#endif
