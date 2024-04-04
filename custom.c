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

#endif