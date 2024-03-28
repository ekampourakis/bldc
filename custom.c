#ifndef CUSTOM_
#define CUSTOM_

#include "custom.h"

// Control functions
bool SoftLock = false;
#define SoftLockDelay	500
#define SoftLockUnlock	0.15
systime_t LastSoftLock = 0;
void StopMotor() {
	// If motor is running
	if (mc_interface_get_state() != MC_STATE_OFF) {
		// Stop and lock motor
		mc_interface_release_motor();
		chThdSleepMilliseconds(50);
		MotorStopped = true;
		mc_interface_lock();
		// Sleep after locking motor to avoid sudden restart
		chThdSleepMilliseconds(50);
		SoftLock = true;
		LastSoftLock = chVTGetSystemTimeX();
	}
}

void StartMotor() {
	// Unlock the motor
	MotorStopped = false;
	SoftLock = false;
	mc_interface_unlock();
}

// LED Functions

void FaultBlink() {
	static systime_t LastFastBlink = 0;
	static bool LEDState = false;
	systime_t Current = chVTGetSystemTimeX();
	if ((float) ST2MS(chVTTimeElapsedSinceX(LastFastBlink)) > BlinkDelay) {
		LEDState = !LEDState;
		if (LEDState) {
			THROTTLE_LED_ON();
		}
		else {
			THROTTLE_LED_OFF();
		}
		LastFastBlink = Current;
	}
}

void LowBatteryBlink() {
	// Keep LED on until error is cleared
	THROTTLE_LED_ON();	
}

void FastBlink() {
	static systime_t LastFastBlink = 0;
	static bool LEDState = false;
	systime_t Current = chVTGetSystemTimeX();
	if ((float) ST2MS(chVTTimeElapsedSinceX(LastFastBlink)) > FastBlinkDelay) {
		LEDState = !LEDState;
		if (LEDState) {
			THROTTLE_LED_ON();
		}
		else {
			THROTTLE_LED_OFF();
		}
		LastFastBlink = Current;
	}
}

static uint8_t BlinksLeft = 0;
void BatteryBlink(float BatteryLevel) {
	static systime_t LastBatteryBlink = 0;
	static bool LEDState = false;
	systime_t Current = chVTGetSystemTimeX();
	if (BlinksLeft <= 0) {
		if ((float) ST2MS(chVTTimeElapsedSinceX(LastBatteryBlink)) > BatteryBlinkInterval) {
			BlinksLeft = round(utils_map(BatteryLevel, 0.0, 1.0, 10.0, 40.0) / 10.0);
			LastBatteryBlink = Current;
		}
		THROTTLE_LED_OFF();
	}
	else {
		if ((float) ST2MS(chVTTimeElapsedSinceX(LastBatteryBlink)) > BatteryBlinkDelay) {
			if (LEDState) {
				LEDState = false;
				BlinksLeft--;
				THROTTLE_LED_OFF();
			}
			else {
				LEDState = true;
				THROTTLE_LED_ON();
			}
			LastBatteryBlink = Current;
		}
	}
}

// Telemetry

uint8_t Telemetry_ThrottleDemand() {
	return (uint8_t)utils_map(CurrentThrottle, 0.0, 1.0, 0.0, 255.0);
}

uint8_t Telemetry_FilteredThrottle() {
	return (uint8_t)utils_map(FilteredThrottle, 0.0, 1.0, 0.0, 255.0);
}

int16_t Telemetry_FilteredRPM() {
	return (int16_t)CurrentRPM;
}

uint16_t Telemetry_Wh() {
	// Not yet implemented
	// On boot load from EEPROM previous value
	// Return previous value + current vesc wh measurement
	// On shutdown add the current vesc wh measurement to the EEPROM and store
	return 500;
}

uint16_t Telemetry_Boots() {
	// Not yet implemented
	
}

uint8_t Telemetry_DriveMode() {
	if (drivemode == DRIVE_MODE_CRUISE) {
		return 0;
	} else if (drivemode == DRIVE_MODE_RACE) {
		return 1;
	} else {
		// Code should never reach this point during normal operation
		return 2;
	}
}

uint8_t Telemetry_CruiseControlMode() {
	if (config.UsePowerCruise) {
		if (CruiseControlActive) {
			return 1;
		} else {
			return 0;
		}
	} else {
		if (CruiseControlActive) {
			return 3;
		} else {
			return 2;
		}
	}
}

static float CruiseRPM = 0.0;
uint16_t Telemetry_CruiseControlPayload() {
	if (Telemetry_CruiseControlMode() == 0 || Telemetry_CruiseControlMode() == 2) {
		return 0;
	} else if (Telemetry_CruiseControlMode() == 1) {
		// return target power
		return (uint16_t)PeakPower;
	} else if (Telemetry_CruiseControlMode() == 3) {
		// return targer rpm
		return (uint16_t)CruiseRPM;
	}
}

uint8_t Telemetry_RunningState() {
	if (MotorStopped) {
		return 0;
	} else {
		if (!CruiseControlActive) {
			if (FilteredThrottle > 0.01) {
				if (IsStarting) {
					if (CurrentRPM <= config.LowAmpLimitERPM) {
						return 1;
					} else if (CurrentRPM > config.LowAmpLimitERPM && CurrentRPM <= config.LowToHighERPM) {
						return 2;
					}
				} else {
					return 4;
				}
			} else {
				if (CurrentRPM > MCCONF_S_PID_MIN_RPM) {
					return 3;
				}
			}
		}
	}
	// This should never be returned in normal working conditions
	return 5;
}

#endif