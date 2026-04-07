/*
 * copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>

#include <cy_syslib.h>
#include <cy_flash_srom.h>

#define CY_SROM_RESP_STATUS_MASK	0xFF000000U
#define CY_SROM_UNIQUE_ID_BYTES		11U

static uint32_t reset_cause(uint32_t cause_mask)
{
	uint32_t reset = 0U;

	if (cause_mask & CY_SYSLIB_RESET_HWWDT) {
		reset |= RESET_WATCHDOG;
	}

	if (cause_mask & (CY_SYSLIB_RESET_ACT_FAULT | CY_SYSLIB_RESET_DPSLP_FAULT)) {
		reset |= RESET_HARDWARE;
	}
#ifdef CY_SYSLIB_RESET_TC_DBGRESET
	if (cause_mask & CY_SYSLIB_RESET_TC_DBGRESET) {
		reset |= RESET_DEBUG;
	}
#endif
	if (cause_mask & CY_SYSLIB_RESET_SOFT) {
		reset |= RESET_SOFTWARE;
	}

	if (cause_mask & (CY_SYSLIB_RESET_SWWDT0 |
			  CY_SYSLIB_RESET_SWWDT1 |
			  CY_SYSLIB_RESET_SWWDT2 |
			  CY_SYSLIB_RESET_SWWDT3)) {
		reset |= RESET_WATCHDOG;
	}

	if (cause_mask & CY_SYSLIB_RESET_HIB_WAKEUP) {
		reset |= RESET_LOW_POWER_WAKE;
	}

	if (cause_mask & CY_SYSLIB_RESET_XRES) {
		reset |= RESET_PIN;
	}

	if (cause_mask & (CY_SYSLIB_RESET_BODVDDD |
			  CY_SYSLIB_RESET_BODVDDA |
			  CY_SYSLIB_RESET_BODVCCD)) {
		reset |= RESET_BROWNOUT;
	}

	if (cause_mask & (CY_SYSLIB_RESET_OVDVDDD |
			  CY_SYSLIB_RESET_OVDVDDA |
			  CY_SYSLIB_RESET_OVDVCCD)) {
		reset |= RESET_HARDWARE;
	}

	if (cause_mask & (CY_SYSLIB_RESET_OCD_ACT_LINREG |
			  CY_SYSLIB_RESET_OCD_DPSLP_LINREG |
			  CY_SYSLIB_RESET_OCD_REGHC |
			  CY_SYSLIB_RESET_PMIC)) {
		reset |= RESET_HARDWARE;
	}

	if (cause_mask & CY_SYSLIB_RESET_PXRES) {
		reset |= RESET_PIN | RESET_HARDWARE;
	}

	if (cause_mask & CY_SYSLIB_RESET_STRUCT_XRES) {
		reset |= RESET_HARDWARE;
	}

	if (cause_mask & CY_SYSLIB_RESET_PORVDDD) {
		reset |= RESET_POR;
	}

	return reset;
}

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	un_srom_api_args_t  args;
	un_srom_api_resps_t resps;
	cy_en_srom_driver_status_t st;
	uint32_t uid[3];
	size_t out_len;

	if ((buffer == NULL) || (length == 0U)) {
		return -EINVAL;
	}

	memset(&args, 0, sizeof(args));
	memset(&resps, 0, sizeof(resps));

	args.RdUnId.arg0.Opcode = CY_SROM_OP_READ_UNIQUE_ID;
	st = Cy_Srom_CallApi(&args, &resps);
	if (st != CY_SROM_DR_SUCCEEDED) {
		return -EIO;
	}

	uid[0] = resps.resp[0] & ~CY_SROM_RESP_STATUS_MASK;
	uid[1] = resps.resp[1];
	uid[2] = resps.resp[2];

	out_len = MIN(length, CY_SROM_UNIQUE_ID_BYTES);

	memcpy(buffer, uid, out_len);

	if (out_len < 0) {
		return -ENOSYS;
	}

	return out_len;
}

int z_impl_hwinfo_get_device_eui64(uint8_t *buffer)
{
	return -ENOSYS;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t reason;

	if (cause == NULL) {
		return -EINVAL;
	}

	reason = Cy_SysLib_GetResetReason();
	*cause = reset_cause(reason);

	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	Cy_SysLib_ClearResetReason();
	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	if (supported == NULL) {
		return -EINVAL;
	}

	*supported = RESET_PIN |
		     RESET_SOFTWARE |
		     RESET_BROWNOUT |
		     RESET_POR |
		     RESET_WATCHDOG |
#ifdef CY_SYSLIB_RESET_TC_DBGRESET
		     RESET_DEBUG |
#endif
		     RESET_LOW_POWER_WAKE |
		     RESET_HARDWARE;

	return 0;
}
