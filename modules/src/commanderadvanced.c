/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include "FreeRTOS.h"
#include "task.h"

#include "commanderadvanced.h"
#include "crtp.h"
#include "configblock.h"
#include "param.h"

#define MIN_THRUST  1000
#define MAX_THRUST  60000

struct CommanderAdvancedCrtpValues {
	uint8_t id;
	uint16_t pid;
	uint8_t rssi;
	uint16_t x;
	uint16_t y;
	uint16_t z;
	float roll;
	float pitch;
	float yaw;
	uint16_t thrust;
}__attribute__((packed));

static struct CommanderAdvancedCrtpValues targetVal[2];
static bool isInit;
static int side = 0;
static uint32_t lastUpdate;
static bool isInactive;
static bool thrustLocked;
static bool altHoldMode = false;
static bool altHoldModeOld = false;

static RPYType stabilizationModeRoll = ANGLE; // Current stabilization type of roll (rate or angle)
static RPYType stabilizationModePitch = ANGLE; // Current stabilization type of pitch (rate or angle)
static RPYType stabilizationModeYaw = RATE; // Current stabilization type of yaw (rate or angle)

static YawModeType yawMode = DEFUALT_YAW_MODE; // Yaw mode configuration
static bool carefreeResetFront;          // Reset what is front in carefree mode

static void commanderAdvancedCrtpCB(CRTPPacket* pk);
static void commanderAdvancedWatchdogReset(void);

void commanderAdvancedInit(void) {
	if (isInit)
		return;

	crtpInit();
	crtpRegisterPortCB(CRTP_PORT_COMMANDER_ADVANCED, commanderAdvancedCrtpCB);

	lastUpdate = xTaskGetTickCount();
	isInactive = true;
	thrustLocked = true;
	isInit = true;
}

bool commanderAdvancedTest(void) {
	crtpTest();
	return isInit;
}

static void commanderAdvancedCrtpCB(CRTPPacket* pk) {
	targetVal[!side] = *((struct CommanderAdvancedCrtpValues*) pk->data);
	side = !side;

	if (targetVal[side].thrust == 0) {
		thrustLocked = false;
	}

	commanderAdvancedWatchdogReset();
}

void commanderAdvancedWatchdog(void) {
	int usedSide = side;
	uint32_t ticktimeSinceUpdate;

	ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;

	if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_STABALIZE) {
		targetVal[usedSide].roll = 0;
		targetVal[usedSide].pitch = 0;
		targetVal[usedSide].yaw = 0;
	}
	if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
		targetVal[usedSide].thrust = 0;
		altHoldMode = false; // do we need this? It would reset the target altitude upon reconnect if still hovering
		isInactive = true;
		thrustLocked = true;
	} else {
		isInactive = false;
	}
}

static void commanderAdvancedWatchdogReset(void) {
	lastUpdate = xTaskGetTickCount();
}

uint32_t commanderAdvancedGetInactivityTime(void) {
	return xTaskGetTickCount() - lastUpdate;
}

void commanderAdvancedGetRPY(float* eulerRollDesired, float* eulerPitchDesired,
		float* eulerYawDesired) {
	int usedSide = side;

	*eulerRollDesired = targetVal[usedSide].roll;
	*eulerPitchDesired = targetVal[usedSide].pitch;
	*eulerYawDesired = targetVal[usedSide].yaw;
}

void commanderAdvancedGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange) {
	*altHold = altHoldMode; // Still in altitude hold mode
	*setAltHold = !altHoldModeOld && altHoldMode; // Hover just activated
	*altHoldChange =
			altHoldMode ?
					((float) targetVal[side].thrust - 32767.f) / 32767.f : 0.0; // Amount to change altitude hold target
	altHoldModeOld = altHoldMode;
}

bool commanderAdvancedGetAltHoldMode(void) {
	return (altHoldMode);
}

void commanderAdvancedSetAltHoldMode(bool altHoldModeNew) {
	altHoldMode = altHoldModeNew;

	/**
	 * Dirty trick to ensure the altHoldChange variable remains zero after next call to commanderAdvancedGetAltHold().
	 *
	 * This is needed since the commanderAdvancedGetAltHold calculates the altHoldChange to -1 if altHoldMode is enabled
	 * with a simultaneous thrust command of 0.
	 *
	 * When altHoldChange is calculated to -1 when enabling altHoldMode, the altTarget will steadily decrease
	 * until thrust is commanded to correct the altitude, which is what we want to avoid.
	 */
	if (altHoldModeNew) {
		targetVal[side].thrust = 32767;
	}
}

void commanderAdvancedGetRPYType(RPYType* rollType, RPYType* pitchType,
		RPYType* yawType) {
	*rollType = stabilizationModeRoll;
	*pitchType = stabilizationModePitch;
	*yawType = stabilizationModeYaw;
}

void commanderAdvancedGetThrust(uint16_t* thrust) {
	int usedSide = side;
	uint16_t rawThrust = targetVal[usedSide].thrust;

	if (thrustLocked) {
		*thrust = 0;
	} else {
		if (rawThrust > MIN_THRUST) {
			*thrust = rawThrust;
		} else {
			*thrust = 0;
		}

		if (rawThrust > MAX_THRUST) {
			*thrust = MAX_THRUST;
		}
	}

	commanderAdvancedWatchdog();
}

YawModeType commanderAdvancedGetYawMode(void) {
	return yawMode;
}

bool commanderAdvancedGetYawModeCarefreeResetFront(void) {
	return carefreeResetFront;
}

// Params for flight modes
PARAM_GROUP_START(flightmode) PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)
