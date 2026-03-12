opyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * TLV320AIC26 Audio Codec Register Definitions
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_
#define ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_

#include <zephyr/types.h>

/*
 * SPI Command Word:
 * Bit 15:    R/W (1=read, 0=write)
 * Bit 14-11: Page (PG3-PG0)
 * Bit 10-5:  Address (ADDR5-ADDR0)
 * Bit 4-0:   Reserved (must be 0)
 */
#define AIC26_CMD_READ          BIT(15)
#define AIC26_CMD_WRITE         0
#define AIC26_CMD_PAGE_SHIFT    11
#define AIC26_CMD_ADDR_SHIFT    5

#define AIC26_BUILD_CMD(rw, page, addr) \
	((rw) | (((page) & 0x0F) << AIC26_CMD_PAGE_SHIFT) | \
	 (((addr) & 0x3F) << AIC26_CMD_ADDR_SHIFT))

/* Pages */
#define AIC26_PAGE0             0
#define AIC26_PAGE1             1
#define AIC26_PAGE2             2

/* ---- PAGE 0: Data Registers (read only) ---- */
#define AIC26_P0_BAT1           0x05
#define AIC26_P0_BAT2           0x06
#define AIC26_P0_AUX            0x07
#define AIC26_P0_TEMP1          0x09
#define AIC26_P0_TEMP2          0x0A

/* ---- PAGE 1: Auxiliary Control ---- */
#define AIC26_P1_ADC_CTL        0x00
#define AIC26_P1_STATUS         0x01
#define AIC26_P1_REFERENCE      0x03
#define AIC26_P1_RESET          0x04

/* Page1 Reg00: ADC Control */
#define AIC26_ADST_BIT          BIT(14)
#define AIC26_ADSCM_SHIFT       10
#define AIC26_ADSCM_MASK        (0x0F << AIC26_ADSCM_SHIFT)
#define AIC26_RESOL_SHIFT       8
#define AIC26_RESOL_MASK        (0x03 << AIC26_RESOL_SHIFT)
#define AIC26_ADAVG_SHIFT       6
#define AIC26_ADAVG_MASK        (0x03 << AIC26_ADAVG_SHIFT)
#define AIC26_ADCR_SHIFT        4
#define AIC26_ADCR_MASK         (0x03 << AIC26_ADCR_SHIFT)

/* Page1 Reg01: Status */
#define AIC26_DAV_SHIFT         14
#define AIC26_DAV_MASK          (0x03 << AIC26_DAV_SHIFT)

/* Page1 Reg03: Reference */
#define AIC26_VREFM_BIT         BIT(4)
#define AIC26_RPWDN_BIT         BIT(1)
#define AIC26_IREFV_BIT         BIT(0)

/* Software reset magic value */
#define AIC26_RESET_VALUE       0xBB00

/* ---- PAGE 2: Audio Control ---- */
#define AIC26_P2_AUDIO_CTL1     0x00
#define AIC26_P2_ADC_GAIN       0x01
#define AIC26_P2_DAC_GAIN       0x02
#define AIC26_P2_SIDETONE       0x03
#define AIC26_P2_AUDIO_CTL2     0x04
#define AIC26_P2_POWER_CTL      0x05
#define AIC26_P2_AUDIO_CTL3     0x06
#define AIC26_P2_FILTER_BASE    0x07
#define AIC26_P2_PLL1           0x1B
#define AIC26_P2_PLL2           0x1C
#define AIC26_P2_AUDIO_CTL4     0x1D
#define AIC26_P2_AUDIO_CTL5     0x1E

/* Page2 Reg00: Audio Control 1 */
#define AIC26_ADCHPF_SHIFT      14
#define AIC26_ADCHPF_MASK       (0x03 << AIC26_ADCHPF_SHIFT)
#define AIC26_ADCIN_SHIFT       12
#define AIC26_ADCIN_MASK        (0x03 << AIC26_ADCIN_SHIFT)
#define AIC26_WLEN_SHIFT        10
#define AIC26_WLEN_MASK         (0x03 << AIC26_WLEN_SHIFT)
#define AIC26_DATFM_SHIFT       8
#define AIC26_DATFM_MASK        (0x03 << AIC26_DATFM_SHIFT)
#define AIC26_DACFS_SHIFT       3
#define AIC26_DACFS_MASK        (0x07 << AIC26_DACFS_SHIFT)
#define AIC26_ADCFS_SHIFT       0
#define AIC26_ADCFS_MASK        (0x07 << AIC26_ADCFS_SHIFT)

#define AIC26_WLEN_16           0x00
#define AIC26_WLEN_20           0x01
#define AIC26_WLEN_24           0x02
#define AIC26_WLEN_32           0x03

#define AIC26_DATFM_I2S         0x00
#define AIC26_DATFM_DSP         0x01
#define AIC26_DATFM_RJ          0x02
#define AIC26_DATFM_LJ          0x03

#define AIC26_FS_DIV_1          0x00
#define AIC26_FS_DIV_1_5        0x01
#define AIC26_FS_DIV_2          0x02
#define AIC26_FS_DIV_3          0x03
#define AIC26_FS_DIV_4          0x04
#define AIC26_FS_DIV_5          0x05
#define AIC26_FS_DIV_5_5        0x06
#define AIC26_FS_DIV_6          0x07

/* Page2 Reg01: ADC Gain */
#define AIC26_ADMUT_BIT         BIT(15)
#define AIC26_ADPGA_SHIFT       8
#define AIC26_ADPGA_MASK        (0x7F << AIC26_ADPGA_SHIFT)
#define AIC26_AGCTG_SHIFT       5
#define AIC26_AGCTG_MASK        (0x07 << AIC26_AGCTG_SHIFT)
#define AIC26_AGCTC_SHIFT       1
#define AIC26_AGCTC_MASK        (0x0F << AIC26_AGCTC_SHIFT)
#define AIC26_AGCEN_BIT         BIT(0)

/* Page2 Reg02: DAC Gain */
#define AIC26_DALMU_BIT         BIT(15)
#define AIC26_DALVL_SHIFT       8
#define AIC26_DALVL_MASK        (0x7F << AIC26_DALVL_SHIFT)
#define AIC26_DARMU_BIT         BIT(7)
#define AIC26_DARVL_SHIFT       0
#define AIC26_DARVL_MASK        (0x7F << AIC26_DARVL_SHIFT)

/* Page2 Reg05: Power Control */
#define AIC26_PWDNC_BIT         BIT(15)
#define AIC26_ASTPWD_BIT        BIT(13)
#define AIC26_DAODRC_BIT        BIT(12)
#define AIC26_ASTPWF_BIT        BIT(11)
#define AIC26_DAPWDN_BIT        BIT(10)
#define AIC26_ADPWDN_BIT        BIT(9)
#define AIC26_VGPWDN_BIT        BIT(8)
#define AIC26_ADPWDF_BIT        BIT(7)
#define AIC26_DAPWDF_BIT        BIT(6)
#define AIC26_ADWSF_BIT         BIT(5)
#define AIC26_VBIAS_BIT         BIT(4)
#define AIC26_EFFCTL_BIT        BIT(1)
#define AIC26_DEEMPF_BIT        BIT(0)

/* Page2 Reg06: Audio Control 3 */
#define AIC26_DMSVOL_SHIFT      14
#define AIC26_DMSVOL_MASK       (0x03 << AIC26_DMSVOL_SHIFT)
#define AIC26_REFFS_BIT         BIT(13)
#define AIC26_DAXFM_BIT         BIT(12)
#define AIC26_SLVMS_BIT         BIT(11)
#define AIC26_DAPK2PK_SHIFT     9
#define AIC26_DAPK2PK_MASK      (0x03 << AIC26_DAPK2PK_SHIFT)
#define AIC26_ADCOVF_BIT        BIT(8)
#define AIC26_DALOVF_BIT        BIT(7)
#define AIC26_DAROVF_BIT        BIT(6)
#define AIC26_AGCNL_SHIFT       4
#define AIC26_AGCNL_MASK        (0x03 << AIC26_AGCNL_SHIFT)

/* Page2 Reg1B: PLL1 */
#define AIC26_PLLSEL_BIT        BIT(15)
#define AIC26_QVAL_SHIFT        11
#define AIC26_QVAL_MASK         (0x0F << AIC26_QVAL_SHIFT)
#define AIC26_PVAL_SHIFT        8
#define AIC26_PVAL_MASK         (0x07 << AIC26_PVAL_SHIFT)
#define AIC26_JVAL_SHIFT        2
#define AIC26_JVAL_MASK         (0x3F << AIC26_JVAL_SHIFT)

/* Page2 Reg1C: PLL2 */
#define AIC26_DVAL_SHIFT        2
#define AIC26_DVAL_MASK         (0x3FFF << AIC26_DVAL_SHIFT)

/* Page2 Reg1D: Audio Control 4 */
#define AIC26_ASTPD_BIT         BIT(15)
#define AIC26_DASTPD_BIT        BIT(14)
#define AIC26_SHCKT_DIS_BIT     BIT(8)
#define AIC26_SHCKT_PD_BIT      BIT(7)
#define AIC26_SHCKT_FLAG_BIT    BIT(6)
#define AIC26_DAC_POP_RED_BIT   BIT(5)

/* Page2 Reg1E: Audio Control 5 */
#define AIC26_DRV_POP_DIS_BIT   BIT(2)
#define AIC26_DRV_POP_LEN_BIT   BIT(1)

#endif /* ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_ */
