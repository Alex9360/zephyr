#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pmic/mpq7932.h>
#define NODE DT_NODELABEL(pmic)
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
