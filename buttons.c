// Blocking function
void VisualizeButtonPress(button_press_type presstype) {
	THROTTLE_LED_OFF();
	chThdSleep((float) MS2ST(50));
	switch (presstype) {
	case BUTTON_PRESS_SHORT:
		THROTTLE_LED_ON();
		chThdSleep((float) MS2ST(50));
		THROTTLE_LED_OFF();
		break;
	case BUTTON_PRESS_LONG:
		THROTTLE_LED_ON();
		chThdSleep((float) MS2ST(250));
		THROTTLE_LED_OFF();
		break;
	case BUTTON_PRESS_CONTINUOUS:
		THROTTLE_LED_ON();
		chThdSleep((float) MS2ST(450));
		THROTTLE_LED_OFF();
		break;
	}
}

void ShortPress() {
	// If throttle is not pressed
	if (CurrentThrottle < 0.01) {
		// Toggle cruise control
		if (CruiseControlActive) {
			CruiseControlActive = false;
			// Release but don't lock motor
			mc_interface_release_motor();
		}
		else {
			// Only activate cruise control above minimum ERPMs
			if (CurrentRPM > MCCONF_S_PID_MIN_RPM) {
				CruiseControlActive = true;
				if (!config.UsePowerCruise) {
					CruiseRPM = CurrentRPM;
					mc_interface_set_pid_speed(CruiseRPM);
				}	
			}
		}
	}
	VisualizeButtonPress(BUTTON_PRESS_SHORT);
}

void LongPress() {
	if (config.UseDriveModes) {
		// Change drive mode
		switch (drivemode) {
		case DRIVE_MODE_CRUISE:
			// Re-init to Race mode
			drivemode = DRIVE_MODE_RACE;
			break;
		case DRIVE_MODE_RACE:
			// Re-init to Cruise mode
			drivemode = DRIVE_MODE_CRUISE;
			break;
		}
		VisualizeButtonPress(BUTTON_PRESS_LONG);
	}
}

void ContinuousPress() {
	// Motor emergency stop
	StopMotor();
	VisualizeButtonPress(BUTTON_PRESS_CONTINUOUS);
}

void ProcessButtons() {
	// Reset button press timing
	static systime_t LastButtonPressed = 0;
	static bool PreviousButtonState = false;
	// Consider button locked to avoid booting with button pressed
	static bool ButtonLocked = true;
	// If button is pressed
	if (read_voltage < ButtonMaxVoltage) {
		// If button is not locked
		if (!ButtonLocked) {
			if (PreviousButtonState) {
				// If button is pressed continuously
				if ((float) ST2MS(chVTTimeElapsedSinceX(LastButtonPressed)) > ButtonTimeout) {
					// Lock button
					ButtonLocked = true;
					// Call continuous press function
					ContinuousPress();
				}
			}
			else {
				// Store last time button was pressed
				LastButtonPressed = chVTGetSystemTimeX();
			}
		}
	}
	else {
		// If button is not locked
		if (!ButtonLocked) {
			// If button was just released
			if (PreviousButtonState) {
				// Check for long or short press. Order matters!
				float ElapsedTime = (float) ST2MS(chVTTimeElapsedSinceX(LastButtonPressed));
				if (ElapsedTime >= LongPressTime && ElapsedTime <= MaxLongPressTime) {
					LongPress();
				}
				else if (ElapsedTime > ShortPressTime && ElapsedTime <= MaxShortPressTime) {
					ShortPress();
				}
			}
		}
		else {
			// If button was locked and now released unlock it
			ButtonLocked = false;
		}
	}
	// Store last button state
	PreviousButtonState = read_voltage < ButtonMaxVoltage;
}