#if 0
/*
 * Copyright 2024-2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/toolchain.h>
#include <string.h>

#ifndef CONFIG_USE_DMIC
#include "song.h"
#endif

#define I2S_CODEC_TX DT_ALIAS(i2s_codec_tx)

#define SAMPLE_FREQUENCY CONFIG_SAMPLE_FREQ
#define SAMPLE_BIT_WIDTH CONFIG_SAMPLE_WIDTH
#define BYTES_PER_SAMPLE CONFIG_BYTES_PER_SAMPLE
#if CONFIG_USE_DMIC
#define NUMBER_OF_CHANNELS CONFIG_DMIC_CHANNELS
#else
#define NUMBER_OF_CHANNELS (2U)
#endif
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK ((SAMPLE_FREQUENCY / 10) * NUMBER_OF_CHANNELS)
#define INITIAL_BLOCKS    CONFIG_I2S_INIT_BUFFERS
#define TIMEOUT           (2000U)

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT (INITIAL_BLOCKS + CONFIG_EXTRA_BLOCKS)

K_MEM_SLAB_DEFINE_IN_SECT_STATIC(mem_slab, __nocache, BLOCK_SIZE, BLOCK_COUNT, 4);

static bool configure_tx_streams(const struct device *i2s_dev, struct i2s_config *config)
{
	int ret;

	ret = i2s_configure(i2s_dev, I2S_DIR_TX, config);
	if (ret < 0) {
		printk("Failed to configure codec stream: %d\n", ret);
		return false;
	}

	return true;
}

static bool trigger_command(const struct device *i2s_dev_codec, enum i2s_trigger_cmd cmd)
{
	int ret;

	ret = i2s_trigger(i2s_dev_codec, I2S_DIR_TX, cmd);
	if (ret < 0) {
		printk("Failed to trigger command %d on TX: %d\n", cmd, ret);
		return false;
	}

	return true;
}

int main(void)
{
	const struct device *const i2s_dev_codec = DEVICE_DT_GET(I2S_CODEC_TX);
#if CONFIG_USE_DMIC
	const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
#endif
	const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
	struct i2s_config config;
	struct audio_codec_cfg audio_cfg;
	int ret = 0;

#if CONFIG_USE_DMIC
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};
#endif
	printk("codec sample\n");

#if CONFIG_USE_DMIC
	if (!device_is_ready(dmic_dev)) {
		printk("%s is not ready", dmic_dev->name);
		return 0;
	}
#endif

	if (!device_is_ready(i2s_dev_codec)) {
		printk("%s is not ready\n", i2s_dev_codec->name);
		return 0;
	}

	if (!device_is_ready(codec_dev)) {
		printk("%s is not ready", codec_dev->name);
		return 0;
	}
	audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK;
	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	audio_cfg.dai_cfg.i2s.channels = 2;
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
#ifdef CONFIG_USE_CODEC_CLOCK
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER;
#else
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_TARGET | I2S_OPT_BIT_CLK_TARGET;
#endif
	audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	audio_cfg.dai_cfg.i2s.mem_slab = &mem_slab;
	audio_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE;
	audio_codec_configure(codec_dev, &audio_cfg);
	k_msleep(1000);

#if CONFIG_USE_DMIC
	cfg.channel.req_num_chan = 2;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
				      dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
	cfg.streams[0].pcm_rate = SAMPLE_FREQUENCY;
	cfg.streams[0].block_size = BLOCK_SIZE;

	printk("PCM output rate: %u, channels: %u\n", cfg.streams[0].pcm_rate,
	       cfg.channel.req_num_chan);

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		printk("Failed to configure the driver: %d", ret);
		return ret;
	}
#endif

	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
#ifdef CONFIG_USE_CODEC_CLOCK
	config.options = I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET;
#else
	config.options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER;
#endif
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &mem_slab;
	config.block_size = BLOCK_SIZE;
	config.timeout = TIMEOUT;
	if (!configure_tx_streams(i2s_dev_codec, &config)) {
		printk("failure to config streams\n");
		return 0;
	}

	printk("start streams\n");
	for (;;) {
		bool started = false;
#if CONFIG_USE_DMIC
		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		if (ret < 0) {
			printk("START trigger failed: %d", ret);
			return ret;
		}
#endif
		while (1) {
			void *mem_block;
			uint32_t block_size = BLOCK_SIZE;
			int i;

			for (i = 0; i < CONFIG_I2S_INIT_BUFFERS; i++) {
#if CONFIG_USE_DMIC
				/* If using DMIC, use a buffer (memory slab) from dmic_read */
				ret = dmic_read(dmic_dev, 0, &mem_block, &block_size, TIMEOUT);
				if (ret < 0) {
					printk("read failed: %d", ret);
					break;
				}

				ret = i2s_write(i2s_dev_codec, mem_block, block_size);
#else
				/* If not using DMIC, play a sine wave 440Hz */

				BUILD_ASSERT(
					BLOCK_SIZE <= __16kHz16bit_stereo_sine_pcm_len,
					"BLOCK_SIZE is bigger than test sine wave buffer size."
				);
				mem_block = (void *)&__16kHz16bit_stereo_sine_pcm;

				ret = i2s_buf_write(i2s_dev_codec, mem_block, block_size);
#endif
				if (ret < 0) {
					printk("Failed to write data: %d\n", ret);
					break;
				}
			}
			if (ret < 0) {
				printk("error %d\n", ret);
				break;
			}
			if (!started) {
				i2s_trigger(i2s_dev_codec, I2S_DIR_TX, I2S_TRIGGER_START);
				started = true;
			}
		}
		if (!trigger_command(i2s_dev_codec, I2S_TRIGGER_DROP)) {
			printk("Send I2S trigger DRAIN failed: %d", ret);
			return 0;
		}
#if CONFIG_USE_DMIC
		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		if (ret < 0) {
			printk("STOP trigger failed: %d", ret);
			return 0;
		}
#endif
		printk("Streams stopped\n");
		return 0;
	}
}
#endif

#if 1
/*
 * Copyright (c) 2026 Linumiz
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/toolchain.h>

#include "song.h"   /* provides __16kHz_16bit_stereo_sine_pcm + _len */

#define I2S_CODEC_TX       DT_ALIAS(i2s_codec_tx)
#define SAMPLE_FREQUENCY   48000
#define SAMPLE_BIT_WIDTH   16U
#define NUMBER_OF_CHANNELS 2U

/* Keep this small (your i2s_infineon driver was failing with big blocks) */
#define BLOCK_SIZE         168U

#define TIMEOUT_MS         2000U

/* Provide enough slab blocks so the driver queue can stay fed */
#define BLOCK_COUNT        16

K_MEM_SLAB_DEFINE_IN_SECT_STATIC(mem_slab, __nocache, BLOCK_SIZE, BLOCK_COUNT, 4);

static int i2s_send_block_retry(const struct device *i2s_dev, void *mem_block, size_t block_size)
{
    int ret;

    do {
        ret = i2s_buf_write(i2s_dev, mem_block, block_size);
        if (ret == -EAGAIN) {
            /* TX queue full -> let ISR/DMA consume and retry */
            k_msleep(1);
        }
    } while (ret == -EAGAIN);

    return ret;
}

int main(void)
{
    const struct device *const i2s_dev  = DEVICE_DT_GET(I2S_CODEC_TX);

    struct i2s_config i2s_cfg = {0};
    int ret;
    const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
    struct audio_codec_cfg audio_cfg = {0};

    if (!device_is_ready(i2s_dev)) {
        printk("ERROR: %s not ready\n", i2s_dev->name);
        return -1;
    }
    if (!device_is_ready(codec_dev)) {
        printk("ERROR: codec not ready\n");
        return -1;
    }

    /* Codec setup (playback) */
    audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK;
    audio_cfg.dai_type  = AUDIO_DAI_TYPE_I2S;
    audio_cfg.dai_cfg.i2s.word_size      = SAMPLE_BIT_WIDTH;
    audio_cfg.dai_cfg.i2s.channels       = NUMBER_OF_CHANNELS;
    audio_cfg.dai_cfg.i2s.format         = I2S_FMT_DATA_FORMAT_I2S;

#ifdef CONFIG_USE_CODEC_CLOCK
    audio_cfg.dai_cfg.i2s.options        = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
#else
    audio_cfg.dai_cfg.i2s.options        = I2S_OPT_FRAME_CLK_SLAVE  | I2S_OPT_BIT_CLK_SLAVE;
#endif

    audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
    audio_cfg.dai_cfg.i2s.mem_slab       = &mem_slab;
    audio_cfg.dai_cfg.i2s.block_size     = BLOCK_SIZE;

   audio_codec_configure(codec_dev, &audio_cfg);
   audio_codec_start_output(codec_dev);
    /* I2S TX setup */
    i2s_cfg.word_size      = SAMPLE_BIT_WIDTH;
    i2s_cfg.channels       = NUMBER_OF_CHANNELS;
    i2s_cfg.format         = I2S_FMT_DATA_FORMAT_I2S;

#ifdef CONFIG_USE_CODEC_CLOCK
    /* Codec provides clocks, so MCU is slave */
    i2s_cfg.options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
#else
    /* MCU provides clocks */
    i2s_cfg.options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
#endif

    i2s_cfg.frame_clk_freq = SAMPLE_FREQUENCY;
    i2s_cfg.mem_slab       = &mem_slab;
    i2s_cfg.block_size     = BLOCK_SIZE;
    i2s_cfg.timeout        = TIMEOUT_MS;

    ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
    if (ret < 0) {
        printk("ERROR: i2s_configure(TX) failed: %d\n", ret);
        return ret;
    }

    /* Prime the TX queue with a few blocks before START */
    size_t off = 0;
    const uint8_t *buf = __16kHz16bit_stereo_sine_pcm;
    const size_t len   = __16kHz16bit_stereo_sine_pcm_len;
    printf("length = %d \n", len);

    BUILD_ASSERT(BLOCK_SIZE <= __16kHz16bit_stereo_sine_pcm_len,
                 "BLOCK_SIZE bigger than sine buffer."); /* [file:16] pattern */

    for (int i = 0; i < 4; i++) {
        if (off + BLOCK_SIZE > len) {
            off = 0;
        }

        ret = i2s_send_block_retry(i2s_dev, (void *)&buf[off], BLOCK_SIZE);
        if (ret < 0) {
            printf("ERROR: prime i2s_buf_write failed: %d\n", ret);
            return ret;
        }
        off += BLOCK_SIZE;
    }

    ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0) {
        printf("ERROR: I2S_TRIGGER_START failed: %d\n", ret);
        return ret;
    }

    printf("Sine playback started (%u Hz, %u-bit, %u ch)\n",
           SAMPLE_FREQUENCY, SAMPLE_BIT_WIDTH, NUMBER_OF_CHANNELS);
    /* Continuous streaming loop */
    while (1) {
        if (off + BLOCK_SIZE > len) {
            off = 0;
        }
        ret = i2s_send_block_retry(i2s_dev, (void *)&buf[off], BLOCK_SIZE);
        if (ret < 0) {
            printf("ERROR: i2s_buf_write failed: %d\n", ret);
            break;
        }
        off += BLOCK_SIZE;
    }
    /* If you ever exit, stop TX cleanly */
    (void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_STOP);
    (void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    return 0;
}
#endif
