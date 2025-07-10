#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include "mpq7932.h"

#define PMIC_NODE DT_NODELABEL(pmic0)
#define PMBUS_PAGE 0
#define VOUT_SET 1.2f

void main(void)
{
	printk("MPQ7932 PMBus Driver Test Start\n");

	const struct device *pmic = DEVICE_DT_GET(PMIC_NODE);

	if (!device_is_ready(pmic)) {
		printk("PMIC device not ready\n");
		return;
	}
	printk("PMIC device ready\n");

	if (mpq7932_output_enable(pmic, PMBUS_PAGE, true) == 0) {
		printk(" Output enabled\n");
	} else {
		printk("Failed to enable output\n");
	}
	if (mpq7932_set_vout(pmic, PMBUS_PAGE, VOUT_SET) == 0) {
		printk("VOUT set to %.3f V\n", VOUT_SET);
	} else {
		printk("Failed to set VOUT\n");
	}

	if (mpq7932_clear_faults(pmic) == 0) {
		printk(" Faults cleared\n");
	} else {
		printk("Failed to clear faults\n");
	}
}

