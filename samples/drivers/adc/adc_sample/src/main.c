#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

#define STACKSIZE 1024
#define PRIORITY 7

// Get ADC specification from devicetree using overlay io-channels definition
static const struct adc_dt_spec adc_chan0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static void adc_sample_thread(void *arg1, void *arg2, void *arg3)
{
    int err;
    int16_t sample_buffer;

    struct adc_sequence sequence = {
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
    };

    printk("ADC sampling started\n");

    if (!adc_is_ready_dt(&adc_chan0)) {
        printk("ADC device not ready\n");
        return;
    }

    err = adc_channel_setup_dt(&adc_chan0);
    if (err < 0) {
        printk("Could not setup channel #0 (%d)\n", err);
        return;
    }

    while (1) {
        err = adc_sequence_init_dt(&adc_chan0, &sequence);
        if (err < 0) {
            printk("Sequence init failed (%d)\n", err);
            k_sleep(K_MSEC(1000));
            continue;
        }

        printk("Reading %s, channel %d: ", adc_chan0.dev->name, adc_chan0.channel_id);

        err = adc_read_dt(&adc_chan0, &sequence);
        if (err < 0) {
            printk("Read failed (%d)\n", err);
            k_sleep(K_MSEC(1000));
            continue;
        }

        int32_t val_mv = sample_buffer;
	 if (adc_chan0.channel_cfg.reference == ADC_REF_VDD_1 &&
            adc_chan0.vref_mv == 0) {
            err = adc_raw_to_millivolts(3300, adc_chan0.channel_cfg.gain,
                                        adc_chan0.resolution, &val_mv);
        } else {
            err = adc_raw_to_millivolts_dt(&adc_chan0, &val_mv);
        }
        if (err < 0) {
            printk("mV conversion failed (%d)\n", err);
        } else {
            printk("raw = %d, %"PRId32" mV\n", sample_buffer, val_mv);
        }

       k_sleep(K_MSEC(1000));
    }
}

K_THREAD_STACK_DEFINE(adc_thread_stack_area, STACKSIZE);
static struct k_thread adc_thread_data;

int main(void)
{
    printk("MSPM0 ADC Sample Application\n");

    if (!adc_is_ready_dt(&adc_chan0)) {
        printk("ADC controller device not ready\n");
        return 0;
    }

    k_thread_create(&adc_thread_data, adc_thread_stack_area,
                    K_THREAD_STACK_SIZEOF(adc_thread_stack_area),
                    adc_sample_thread, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&adc_thread_data, "adc_thread");
    return 0;
}
