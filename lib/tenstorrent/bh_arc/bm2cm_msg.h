/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BM2CM_MSG_H
#define BM2CM_MSG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct bmStaticInfo {
	/* Non-zero for valid data */
	/* Allows for breaking changes */
	uint32_t version;
	uint32_t bl_version;
	uint32_t app_version;
} __packed bmStaticInfo;

void SetBmfwPingValid(bool ping_valid);
bool GetBmfwPingValid(void);
int32_t GetInputCurrent(void);
uint16_t GetFanRPM(void);
uint16_t GetBoardPowerLimit(void);

int32_t Bm2CmPingHandler(const uint8_t *data, uint8_t size);
int32_t Bm2CmSendDataHandler(const uint8_t *data, uint8_t size);
int32_t Bm2CmSendCurrentHandler(const uint8_t *data, uint8_t size);
int32_t Bm2CmSendBoardPwrLimHandler(const uint8_t *data, uint8_t size);
int32_t Bm2CmSendFanRPMHandler(const uint8_t *data, uint8_t size);

#endif
