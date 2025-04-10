/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bm2cm_msg.h"
#include "telemetry.h"

static bool bmfw_ping_valid;
static uint16_t fan_rpm;
static uint16_t board_pwr_limit;
static int32_t current;

void SetBmfwPingValid(bool ping_valid)
{
	bmfw_ping_valid = ping_valid;
}

bool GetBmfwPingValid(void)
{
	return bmfw_ping_valid;
}

int32_t GetInputCurrent(void)
{
	return current;
}

uint16_t GetFanRPM(void)
{
	return fan_rpm;
}

uint16_t GetBoardPowerLimit(void)
{
	return board_pwr_limit;
}

int32_t Bm2CmPingHandler(const uint8_t *data, uint8_t size)
{
	if (size != 2) {
		return -1;
	}

	uint16_t response = *(uint16_t *)data;

	if (response != 0xA5A5) {
		bmfw_ping_valid = false;
		return -1;
	}
	bmfw_ping_valid = true;
	return 0;
}

int32_t Bm2CmSendDataHandler(const uint8_t *data, uint8_t size)
{
#ifndef CONFIG_TT_SMC_RECOVERY
	if (size != sizeof(bmStaticInfo)) {
		return -1;
	}

	bmStaticInfo *info = (bmStaticInfo *)data;

	if (info->version != 0) {
		UpdateBmFwVersion(info->bl_version, info->app_version);
		return 0;
	}
#endif

	return -1;
}

int32_t Bm2CmSendCurrentHandler(const uint8_t *data, uint8_t size)
{
#ifndef CONFIG_TT_SMC_RECOVERY
	if (size != 4) {
		return -1;
	}

	current = *(int32_t *)data;

	return 0;
#endif

	return -1;
}

int32_t Bm2CmSendBoardPwrLimHandler(const uint8_t *data, uint8_t size)
{
#ifndef CONFIG_TT_SMC_RECOVERY
	if (size != 2) {
		return -1;
	}

	board_pwr_limit = *(uint16_t *)data;
	/* SetThrottlerLimit(kThrottlerBoardPwr, board_pwr_limit); */
	/* Is there a chance that this doesn't get received before throttler init? */

	return 0;
#endif

	return -1;
}

int32_t Bm2CmSendFanRPMHandler(const uint8_t *data, uint8_t size)
{
#ifndef CONFIG_TT_SMC_RECOVERY
	if (size != 2) {
		return -1;
	}

	fan_rpm = *(uint16_t *)data;

	return 0;
#endif

	return -1;
}
