/*#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include "mpq7932.h"
//#include <zephyr/pmbus.h>

#define PMIC_NODE DT_NODELABEL(pmic0)
#define PMBUS_PAGE 0
#define VOUT_SET 1.2f

void main()
{
	printk("MPQ7932 PMBus Driver Test Start\n");

	const struct device *pmic = DEVICE_DT_GET(PMIC_NODE);

	if (!device_is_ready(pmic)) {
		printk("PMIC device not ready\n");
		return;
	}
	printk("PMIC device ready\n");
	const struct mpq7932_driver_api* vol = pmic->api;
	if(vol->set_vout(pmic, PMBUS_PAGE, true) == 0)
		printk("output is enabled");
}
	/*

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
}*/


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/printk.h>

#define BUCK_NODE DT_NODELABEL(pmic)

int main(void)
{
    const struct device *buck = DEVICE_DT_GET(BUCK_NODE);
    if (!device_is_ready(buck)) {
        printk("Buck regulator not ready\n");
        return -EINVAL;
    }

    printk("Enabling buck1...\n");
    if (regulator_enable(buck) == 0) {
        printk(" Buck1 enabled\n");
    }

    printk("setting buck1 to 1.2V\n");
    if (regulator_set_voltage(buck, 1200000, 1200000) == 0) {
        printk(" Voltage set\n");
    }

    int32_t voltage;
    if (regulator_get_voltage(buck, &voltage) == 0) {
        printk("Voltage read: %d uV\n", voltage);
    }
    return 0;
}

