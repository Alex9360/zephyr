/*
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_
#define ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_

#include <zephyr/sys/util.h>

#define AIC26_CMD_READ              BIT(15)
#define AIC26_CMD_WRITE             0
#define AIC26_CMD_PAGE_SHIFT        11
#define AIC26_CMD_ADDR_SHIFT        5

#define AIC26_BUILD_CMD(rw, page, addr)                                 \
	((rw) | (((page) & 0x0F) << AIC26_CMD_PAGE_SHIFT) |            \
	 (((addr) & 0x3F) << AIC26_CMD_ADDR_SHIFT))

#define AIC26_PAGE1                 1
#define AIC26_PAGE2                 2

#define AIC26_P1_REFERENCE          0x03
#define AIC26_P1_RESET              0x04

/* Reference Control (Page 1, Reg 0x03) */
#define AIC26_VREFM_BIT             BIT(4)
#define AIC26_IREFV_BIT             BIT(0)

#define AIC26_RESET_VALUE           0xBB00

/* Page 2 registers */
#define AIC26_P2_AUDIO_CTL1         0x00
#define AIC26_P2_DAC_GAIN           0x02
#define AIC26_P2_POWER_CTL          0x05
#define AIC26_P2_AUDIO_CTL3         0x06
#define AIC26_P2_PLL1               0x1B
#define AIC26_P2_PLL2               0x1C

/* Audio Control 1 (Page 2, Reg 0x00) */
#define AIC26_WLEN_SHIFT            10
#define AIC26_WLEN_MASK             (0x03 << AIC26_WLEN_SHIFT)
#define AIC26_DATFM_SHIFT           8
#define AIC26_DATFM_MASK            (0x03 << AIC26_DATFM_SHIFT)
#define AIC26_DACFS_SHIFT           3
#define AIC26_DACFS_MASK            (0x07 << AIC26_DACFS_SHIFT)
#define AIC26_ADCFS_SHIFT           0
#define AIC26_ADCFS_MASK            (0x07 << AIC26_ADCFS_SHIFT)

#define AIC26_WLEN_16               0x00
#define AIC26_WLEN_20               0x01
#define AIC26_WLEN_24               0x02
#define AIC26_WLEN_32               0x03

#define AIC26_DATFM_I2S             0x00
#define AIC26_DATFM_DSP             0x01
#define AIC26_DATFM_RJ              0x02
#define AIC26_DATFM_LJ              0x03

/* DACFS / ADCFS divisor values */
#define AIC26_FS_DIV_1              0x00
#define AIC26_FS_DIV_1_5            0x01
#define AIC26_FS_DIV_2              0x02
#define AIC26_FS_DIV_3              0x03
#define AIC26_FS_DIV_4              0x04
#define AIC26_FS_DIV_5              0x05
#define AIC26_FS_DIV_5_5            0x06
#define AIC26_FS_DIV_6              0x07

/* DAC Gain (Page 2, Reg 0x02) */
#define AIC26_DALMU_BIT             BIT(15)
#define AIC26_DALVL_SHIFT           8
#define AIC26_DALVL_MASK            (0x7F << AIC26_DALVL_SHIFT)
#define AIC26_DARMU_BIT             BIT(7)
#define AIC26_DARVL_SHIFT           0
#define AIC26_DARVL_MASK            (0x7F << AIC26_DARVL_SHIFT)

#define AIC26_DAC_MUTE_ALL          (AIC26_DALMU_BIT |                  \
				     (0x7F << AIC26_DALVL_SHIFT) |      \
				     AIC26_DARMU_BIT | 0x7F)

/* Power Control (Page 2, Reg 0x05) */
#define AIC26_PWDNC_BIT             BIT(15)
#define AIC26_DAODRC_BIT            BIT(12)
#define AIC26_DAPWDN_BIT            BIT(10)
#define AIC26_ADPWDN_BIT            BIT(9)
#define AIC26_VGPWDN_BIT            BIT(8)
#define AIC26_DAPWDF_BIT            BIT(6)

/* Audio Control 3 (Page 2, Reg 0x06) */
#define AIC26_REFFS_BIT             BIT(13)
#define AIC26_SLVMS_BIT             BIT(11)

/* PLL1 (Page 2, Reg 0x1B) */
#define AIC26_PLLSEL_BIT            BIT(15)
#define AIC26_QVAL_SHIFT            11
#define AIC26_QVAL_MASK             (0x0F << AIC26_QVAL_SHIFT)
#define AIC26_PVAL_SHIFT            8
#define AIC26_PVAL_MASK             (0x07 << AIC26_PVAL_SHIFT)
#define AIC26_JVAL_SHIFT            2
#define AIC26_JVAL_MASK             (0x3F << AIC26_JVAL_SHIFT)

/* PLL2 (Page 2, Reg 0x1C) */
#define AIC26_DVAL_SHIFT            2
#define AIC26_DVAL_MASK             (0x3FFF << AIC26_DVAL_SHIFT)

/* Timing */
#define AIC26_RESET_WAIT_MS         10
#define AIC26_PLL_LOCK_MS           10
#define AIC26_DAC_SETTLE_MS         50
#define AIC26_DAC_POLL_MS           10
#define AIC26_DAC_TIMEOUT_MS        500

#endif
