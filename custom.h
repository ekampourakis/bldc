#ifndef CUSTOM_H_
#define CUSTOM_H_

#define RefreshRate			    1000		// The refresh rate of the custom application in Hertz

// Cruise control settings
//
#define C_CruiseControlERPM	    10000.0	// ERPM limit below which cruise control deactivates
#define C_NegativeERPMLimit	    -10000.0	// Negative ERPM limit below which motor will stop

#define C_UsePowerCruise		false
#define C_MaxCruisePower		500 // Watts

// Throttle settings
//
#define C_UseDriveModes		    true
#define C_DefaultDriveMode	    DRIVE_MODE_CRUISE
// Fraction of throttle to deadband in each drive mode 
#define C_DeadBand_Cruise		0.04	// Ignore until 4% of throttle
#define C_DeadBand_Race		    0.06	// Ignore until 6% of throttle
// Throttle ramping variables
#define C_LowAmpLimitERPM		3500	// Limit Amps below this ERPM limit to avoid motor cogging
#define C_LowAmpLimit			12.0	// Maximum Amps to use below #LowAmpLimitERPM
#define C_LowToHighERPM		    7500	// Progressively deliver torque until this ERPM limit to avoid violent jerking
// Ramping takes longer on ERPMs below #LowToHighERPM for smoother starting
// Ramping time is decreased depending on ERPMs to give smooth transition to normal ERPM ramping time
#define C_UseVariableRamping	true
// Variable ramping multiplier to use on low ERPM. Inversely proportional with time
#define C_RampingMultiplier	    0.1		// More than double the ramping time
// Maximum step towards throttle goal. Throttle range [0.0, 1.0]
// The smaller the step the longer it will take to increase throttle
// Calculate using formula below 
// DesiredTime = 1.0 / DesiredLength (in mS for 1kHz set frequency)
#define C_Ramp_Cruise			0.005		// 500ms for 0.0->1.0
#define C_Ramp_Race			    0.010       // 333ms for 0.0->1.0
// Dynamic throttle response expands the useful range of the throttle while
//		limiting power to allow better control over the limited throttle range
#define C_UseDynamicThrottle	    true
// Experimental
#define C_UseDynamicPower		    true
#define C_DynamicDeadband_Cruise    0.12
#define C_DynamicDeadband_Race	    0.16

#define C_MinIdleRPM            5000	// Minimum ERPM to consider motor running
#define C_IdleCurrentLimit		5.0		// Idle Current Amps

// Button settings
//
#define ButtonMaxVoltage	0.3		// Below that voltage button is considered pressed
#define ThrottleMinVoltage	0.6		// Zero level of throttle
#define ThrottleMaxVoltage	1.7		// Full level of throttle
#define ShortPressTime		40		// Minimum time at which a short press is triggered (in mS)
#define MaxShortPressTime	400		// Maximum time at which a short press is triggered (in mS)
#define AllowLongPress		true	// Allow long press
#define LongPressTime		1200	// Minimum time at which a long press is triggered (in mS)
#define MaxLongPressTime	2800	// Maximum time at which a long press is triggered (in mS)
#define AllowTimeoutPress	true	// Allow continuous press
#define ButtonTimeout		3500	// Maximum button hold time at which continuous press is triggered (in mS)


// LED Indicator settings
//
// Battery indicator
#define Battery4Blinks			47.5	// 4 blinks till this voltage
#define Battery3Blinks			45.0	// 3 blinks till this voltage
#define Battery2Blinks			42.0	// 2 blinks till this voltage
// LED timing
#define BatteryBlinkInterval 	3500	// Wait X milliseconds before indicating battery again
#define BatteryBlinkDelay 		350		// How long the LED stays on/off while indicating battery (in mS)
#define FastBlinkDelay			100		// How long the LED stays on/off while fast blinking (in mS)
#define BlinkDelay	            500 	// How long the LED stays on/off while low battery blinking (in mS)


// Default constants
//
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				35
#define BATTERY_FILTER_SAMPLES			5
#define TEMP_FILTER_SAMPLES				5



// LED Macros
//
#define THROTTLE_LED_ON()		palSetPad(GPIOB, 10)		// Set pin PB7 to logical high
#define THROTTLE_LED_OFF()		palClearPad(GPIOB, 10)	// Set pin PB7 to logical low


// Function declarations
//
void StopMotor(void);
void StartMotor(void);
void ProcessRPM(void);
void FastBlink(void);
void FaultBlink(void);
void BatteryBlink(float BatteryLevel);

#endif