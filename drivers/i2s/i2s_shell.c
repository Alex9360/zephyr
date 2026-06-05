/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2S shell: streams a generated test tone over any I2S controller. Transport
 * only - it has no codec dependency. To route the audio through a codec, use
 * the 'codec' shell (codec configure / codec start) alongside it.
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/linker/section_tags.h>

/* 16-bit stereo. Block size/count are Kconfig-tunable (FIFO controllers cap
 * a block at FIFO_DEPTH/2 samples).
 */
#define I2S_SH_BITS		16
#define I2S_SH_CHANNELS		2
#define I2S_SH_FRAMES		CONFIG_I2S_SHELL_BLOCK_FRAMES
#define I2S_SH_BLOCK_BYTES	(I2S_SH_FRAMES * I2S_SH_CHANNELS *	\
				 (I2S_SH_BITS / 8))
#define I2S_SH_BLOCK_COUNT	CONFIG_I2S_SHELL_BLOCK_COUNT

#define I2S_SH_DEFAULT_RATE	48000
#define I2S_SH_DEFAULT_TONE	440
#define I2S_SH_AMPLITUDE	16384				/* ~ -6 dBFS */

#define I2S_SH_TBL		1024
#define I2S_SH_TBL_MASK		(I2S_SH_TBL - 1)
#define I2S_SH_PHASE_FRAC	16

/* DMA reads this buffer directly, bypassing the CPU data cache. Keep it in
 * non-cached memory so cache-coherent SoCs do not transmit stale data (heard
 * as noise). Needs CONFIG_NOCACHE_MEMORY; a normal slab otherwise.
 */
K_MEM_SLAB_DEFINE_IN_SECT_STATIC(i2s_sh_slab, __nocache, I2S_SH_BLOCK_BYTES,
				 I2S_SH_BLOCK_COUNT, 32);

K_THREAD_STACK_DEFINE(i2s_sh_stack, CONFIG_I2S_SHELL_STACK_SIZE);

static struct {
	const struct device *i2s;
	const struct shell *sh;
	uint32_t rate;
	uint32_t tone;
	uint32_t phase;
	uint32_t phase_inc;
	struct k_thread thread;
	k_tid_t tid;
	volatile bool running;
} strm;

static int16_t i2s_sh_sine[I2S_SH_TBL];
static bool i2s_sh_sine_ready;

/* sin(x) for x in [0, pi/2], 5-term Taylor, < 1 LSB error, no libm. */
static float i2s_sh_sin_quarter(float x)
{
	float x2 = x * x;

	return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f +
		    x2 * (-1.0f / 5040.0f + x2 * (1.0f / 362880.0f)))));
}

static void i2s_sh_build_sine(void)
{
	const float half_pi = 1.57079632679f;

	if (i2s_sh_sine_ready) {
		return;
	}

	for (int i = 0; i < I2S_SH_TBL; i++) {
		float q = (float)(4 * i) / (float)I2S_SH_TBL;
		int quad = (int)q;
		float a = (q - (float)quad) * half_pi;
		float s;

		switch (quad) {
		case 0:
			s = i2s_sh_sin_quarter(a);
			break;
		case 1:
			s = i2s_sh_sin_quarter(half_pi - a);
			break;
		case 2:
			s = -i2s_sh_sin_quarter(a);
			break;
		default:
			s = -i2s_sh_sin_quarter(half_pi - a);
			break;
		}

		s *= (float)I2S_SH_AMPLITUDE;
		i2s_sh_sine[i] = (int16_t)(s + (s >= 0.0f ? 0.5f : -0.5f));
	}

	i2s_sh_sine_ready = true;
}

static void i2s_sh_set_tone(uint32_t tone, uint32_t rate)
{
	strm.tone = tone;
	strm.rate = rate;
	strm.phase_inc = (uint32_t)(((uint64_t)tone << I2S_SH_PHASE_FRAC) *
				    I2S_SH_TBL / rate);
}

static void i2s_sh_fill(int16_t *dst)
{
	for (int f = 0; f < I2S_SH_FRAMES; f++) {
		uint32_t idx = (strm.phase >> I2S_SH_PHASE_FRAC) & I2S_SH_TBL_MASK;
		uint32_t nxt = (idx + 1U) & I2S_SH_TBL_MASK;
		int32_t frac = strm.phase & ((1U << I2S_SH_PHASE_FRAC) - 1U);
		int32_t a = i2s_sh_sine[idx];
		int32_t b = i2s_sh_sine[nxt];
		int16_t s = (int16_t)(a + (((b - a) * frac) >> I2S_SH_PHASE_FRAC));

		strm.phase += strm.phase_inc;
		*dst++ = s;
		*dst++ = s;
	}
}

static void i2s_sh_feeder(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	unsigned int queued = 0;
	bool started = false;

	while (strm.running) {
		void *block;
		int ret;

		ret = k_mem_slab_alloc(&i2s_sh_slab, &block, K_MSEC(200));
		if (ret != 0) {
			continue;
		}

		i2s_sh_fill(block);

		ret = i2s_write(strm.i2s, block, I2S_SH_BLOCK_BYTES);
		if (ret < 0) {
			k_mem_slab_free(&i2s_sh_slab, block);
			shell_warn(strm.sh, "i2s_write failed (%d)", ret);
			break;
		}

		/* Prime a couple of blocks before starting the transfer. */
		if (!started && ++queued >= 2) {
			ret = i2s_trigger(strm.i2s, I2S_DIR_TX,
					  I2S_TRIGGER_START);
			if (ret < 0) {
				shell_warn(strm.sh, "i2s START failed (%d)",
					   ret);
				break;
			}
			started = true;
		}
	}

	i2s_trigger(strm.i2s, I2S_DIR_TX,
		    started ? I2S_TRIGGER_DRAIN : I2S_TRIGGER_DROP);
	strm.running = false;
}

static int cmd_tone(const struct shell *sh, size_t argc, char *argv[])
{
	const struct device *dev;
	struct i2s_config cfg;
	uint32_t tone = I2S_SH_DEFAULT_TONE;
	uint32_t rate = I2S_SH_DEFAULT_RATE;
	int ret;

	if (strm.running) {
		shell_error(sh, "Already streaming on %s; 'i2s stop' first",
			    strm.i2s->name);
		return -EBUSY;
	}

	dev = shell_device_get_binding(argv[1]);
	if (!dev) {
		shell_error(sh, "I2S device '%s' not found", argv[1]);
		return -ENODEV;
	}

	if (argc > 2) {
		tone = strtoul(argv[2], NULL, 0);
	}
	if (argc > 3) {
		rate = strtoul(argv[3], NULL, 0);
	}
	if (tone == 0 || rate == 0) {
		shell_error(sh, "tone and rate must be > 0");
		return -EINVAL;
	}

	cfg.word_size = I2S_SH_BITS;
	cfg.channels = I2S_SH_CHANNELS;
	cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	cfg.options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER;
	cfg.frame_clk_freq = rate;
	cfg.mem_slab = &i2s_sh_slab;
	cfg.block_size = I2S_SH_BLOCK_BYTES;
	cfg.timeout = 200;

	ret = i2s_configure(dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		shell_error(sh, "i2s_configure failed (%d)", ret);
		return ret;
	}

	i2s_sh_build_sine();
	strm.phase = 0;
	i2s_sh_set_tone(tone, rate);
	strm.i2s = dev;
	strm.sh = sh;
	strm.running = true;

	strm.tid = k_thread_create(&strm.thread, i2s_sh_stack,
				   K_THREAD_STACK_SIZEOF(i2s_sh_stack),
				   i2s_sh_feeder, NULL, NULL, NULL,
				   CONFIG_I2S_SHELL_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(strm.tid, "i2s_tone");

	shell_print(sh, "Streaming %u Hz tone on %s @ %u Hz, 16-bit stereo",
		    tone, dev->name, rate);
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!strm.running && strm.tid == NULL) {
		shell_warn(sh, "Not streaming");
		return 0;
	}

	strm.running = false;
	if (strm.tid != NULL) {
		k_thread_join(&strm.thread, K_FOREVER);
		strm.tid = NULL;
	}

	shell_print(sh, "Stopped");
	return 0;
}

static int cmd_info(const struct shell *sh, size_t argc, char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "state    : %s", strm.running ? "streaming" : "idle");
	if (strm.i2s) {
		shell_print(sh, "i2s dev  : %s", strm.i2s->name);
	}
	shell_print(sh, "tone     : %u Hz", strm.tone);
	shell_print(sh, "rate     : %u Hz", strm.rate);
	shell_print(sh, "format   : 16-bit stereo, %u-frame blocks x %u",
		    I2S_SH_FRAMES, I2S_SH_BLOCK_COUNT);
	return 0;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2s,
	SHELL_CMD_ARG(tone, &dsub_device_name,
		SHELL_HELP("Stream a generated test tone over I2S TX",
			   "<i2s-device> [tone_hz] [sample_rate]"),
		cmd_tone, 2, 2),
	SHELL_CMD_ARG(stop, NULL,
		SHELL_HELP("Stop streaming", ""),
		cmd_stop, 1, 0),
	SHELL_CMD_ARG(info, NULL,
		SHELL_HELP("Show stream status", ""),
		cmd_info, 1, 0),
	SHELL_SUBCMD_SET_END
);
/* clang-format on */

SHELL_CMD_REGISTER(i2s, &sub_i2s, "I2S commands", NULL);
