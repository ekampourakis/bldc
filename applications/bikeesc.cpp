// ToDo
// ***	Implement temperature based fan control
// *	Fix compiler double promotion warning

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include "conf_general.h"

#ifdef MCCONF_DEFAULT_USER
#include MCCONF_DEFAULT_USER
#endif

#ifdef APP_CUSTOM_TO_USE

// Threads
static THD_FUNCTION(custom_thread, arg);
static THD_WORKING_AREA(custom_thread_wa, 1024);
// Thread variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile custom_config config;

// App variables
static float deadband = 0.0;				// Deadband value is calculated on runtime
static float ramp = 0.0;					// Ramping time is calculated on runtime
static bool CruiseControlActive = false;	// Cruise control off on boot
static volatile float CurrentThrottle = 0.0;			// Consider throttle depressed on boot
static float FilteredThrottle = 0.0;
static float BatteryLevel = 1.0;			// Consider battery full on boot
static bool LowBattery = false;
static bool MotorStopped = true;			// Consider motor locked on boot
static bool PreviousFault = true;			// Consider fault codes cleared on boot
static bool IsStarting = true;				// Consider motor stopped on boot
static float CurrentRPM = 0.0;
static int PeakPower = 0;
static float FanDuty = 0.0;

// Included below and with specific order due to variable dependencies
// It is important to include .c files instead of .h files to avoid compile errors
static volatile float read_voltage; // = ThrottleMinVoltage - 0.001;	// Use this to avoid false button press on boot
static volatile drive_modes drivemode; // = C_DefaultDriveMode;			// Set the drive mode to boot at
#include "custom.c"
#include "buttons.c"
#include "curves.c"

// Required app functions
void app_custom_start(void) {
	read_voltage = ThrottleMinVoltage - 0.001;
	drivemode = C_DefaultDriveMode;
	stop_now = false;
	chThdCreateStatic(custom_thread_wa, sizeof(custom_thread_wa), NORMALPRIO, custom_thread, NULL);
}
void app_custom_stop(void) {
	stop_now = true;
	while (is_running) {
		// Allow other threads to run on dead time
		chThdSleepMilliseconds(1);
	}
}
void app_custom_configure(custom_config *conf) {
	config = *conf;
	//drivemode = config.DefaultDriveMode;
}
bool app_custom_running() {
	return is_running;
}

// Custom control functions

bool ProcessThread() {
	// Sleep for a time according to the specified rate
	systime_t sleep_time = CH_CFG_ST_FREQUENCY / RefreshRate;
	chThdSleep(sleep_time == 0 ? 1 : sleep_time);
	// If thread must be stopped
	if (stop_now) {
		// Stop the thread
		is_running = false;
		return false;
	}
	return true;
}

void ProcessBattery() {
	// Read, filter and map the battery voltage from the interface only if motor is not running
	if (mc_interface_get_state() == MC_STATE_OFF) {
		// Apply running average filter on battery voltage
		static float filter_buffer_battery[BATTERY_FILTER_SAMPLES];
		static int filter_ptr_battery = 0;
		filter_buffer_battery[filter_ptr_battery++] = GET_INPUT_VOLTAGE();
		if (filter_ptr_battery >= BATTERY_FILTER_SAMPLES) {
			filter_ptr_battery = 0;
		}
		float battery_filtered = 0.0;
		for (int i = 0; i < BATTERY_FILTER_SAMPLES; i++) {
			battery_filtered += filter_buffer_battery[i];
		}
		float BatteryVoltage = (battery_filtered / BATTERY_FILTER_SAMPLES);
		// Check for low battery or for battery charge
		if (LowBattery && BatteryVoltage > MCCONF_L_BATTERY_CUT_START) {
			LowBattery = false;
		}
		if (!LowBattery && BatteryVoltage < MCCONF_L_BATTERY_CUT_END) {
			LowBattery = true;
		}
		// Convert battery voltage to LED blinks for battery indication
		// Below code cannot be replaced with a single map function cause battery discharge is not linear
		if (BatteryVoltage >= Battery4Blinks) {
			BatteryLevel = utils_map(BatteryVoltage, Battery4Blinks, MCCONF_L_BATTERY_CHARGED, 0.75, 1.0);
		}
		else if (BatteryVoltage >= Battery3Blinks) {
			BatteryLevel = utils_map(BatteryVoltage, Battery3Blinks, Battery4Blinks, 0.5, 0.75);
		}
		else if (BatteryVoltage >= Battery2Blinks) {
			BatteryLevel = utils_map(BatteryVoltage, Battery2Blinks, Battery3Blinks, 0.25, 0.5);
		}
		else {
			BatteryLevel = utils_map(BatteryVoltage, MCCONF_L_BATTERY_CUT_START, Battery2Blinks, 0.0, 0.25);
		}
		utils_truncate_number(&BatteryLevel, 0.0, 1.0);
	}
}

void ProcessRPM() {
	// Read and normalize current ERPM value using running average filter
	static float filter_buffer_rpm[RPM_FILTER_SAMPLES];
	static int filter_ptr_rpm = 0;
	filter_buffer_rpm[filter_ptr_rpm++] = mc_interface_get_rpm();
	if (filter_ptr_rpm >= RPM_FILTER_SAMPLES) {
		filter_ptr_rpm = 0;
	}
	float rpm_filtered = 0.0;
	for (int i = 0; i < RPM_FILTER_SAMPLES; i++) {
		rpm_filtered += filter_buffer_rpm[i];
	}
	CurrentRPM = (rpm_filtered / RPM_FILTER_SAMPLES);
	// If current ERPM climb above starting limit then we are no longer starting
	if (IsStarting && CurrentRPM > config.LowToHighERPM) {
		IsStarting = false;
	}
	// If current ERPM drop below starting threshold then switch to starting mode
	if (!IsStarting && CurrentRPM < ((config.LowAmpLimitERPM + config.LowToHighERPM) / 2)) {
		IsStarting = true;
		StopMotor();
	}
}

void ProcessTemperature() {
	// Read and normalize motor temperature using running average filter
	static float filter_buffer_temp_motor[TEMP_FILTER_SAMPLES];
	static int filter_ptr_temp_motor = 0;
	filter_buffer_temp_motor[filter_ptr_temp_motor++] = mc_interface_temp_motor_filtered();
	if (filter_ptr_temp_motor >= TEMP_FILTER_SAMPLES) {
		filter_ptr_temp_motor = 0;
	}
	float temp_filtered_motor = 0.0;
	for (int i = 0; i < TEMP_FILTER_SAMPLES; i++) {
		temp_filtered_motor += filter_buffer_temp_motor[i];
	}
	// Read and normalize FET temperatures using running average filter
	static float filter_buffer_temp_fet[TEMP_FILTER_SAMPLES];
	static int filter_ptr_temp_fet = 0;
	filter_buffer_temp_fet[filter_ptr_temp_fet++] = mc_interface_temp_fet_filtered();
	if (filter_ptr_temp_fet >= TEMP_FILTER_SAMPLES) {
		filter_ptr_temp_fet = 0;
	}
	float temp_filtered_fet = 0.0;
	for (int i = 0; i < TEMP_FILTER_SAMPLES; i++) {
		temp_filtered_fet += filter_buffer_temp_fet[i];
	}
	// Set fan duty cycle depending on the motor temperature
	if (temp_filtered_motor < 29.0 && temp_filtered_fet < 29.0) {
		FanDuty = 0.0;
		set_pw(0.0);
	} 
	if (temp_filtered_motor >= 30.0 || temp_filtered_fet >= 30.0) {
		float required_fan = utils_map(temp_filtered_motor, 30.0, 70.0, 2000.0, 3000.0);
		utils_truncate_number(&required_fan, 2000.0, 3000.0);
		FanDuty = utils_map(required_fan, 2000.0, 3000.0, 0.0, 1.0);
		set_pw(required_fan);
	}
}

void ProcessThrottle() {
	// Read input voltage
	read_voltage = (float) ADC_Value[ADC_IND_EXT] * V_REG / 4095;
	// Normalize the read voltage using running average filter
	static float filter_buffer_throttle[FILTER_SAMPLES];
	static int filter_ptr_throttle = 0;
	filter_buffer_throttle[filter_ptr_throttle++] = read_voltage;
	if (filter_ptr_throttle >= FILTER_SAMPLES) {
		filter_ptr_throttle = 0;
	}
	float pwr_filtered = 0.0;
	for (int i = 0; i < FILTER_SAMPLES; i++) {
		pwr_filtered += filter_buffer_throttle[i];
	}
	pwr_filtered /= FILTER_SAMPLES;
	// Convert input voltage to throttle percentage
	CurrentThrottle = utils_map(pwr_filtered, ThrottleMinVoltage, ThrottleMaxVoltage, 0.0, 1.0);
	utils_truncate_number(&CurrentThrottle, 0.0, 1.0);
	// Apply throttle curve depending on drive mode
	// Apply maximum power curves depending on drive mode only if variable ramping is not active
	switch (drivemode) {
	case DRIVE_MODE_CRUISE:
		// Apply cruise mode throttle curve
		CurrentThrottle = ThrottleCurve_Cruise(CurrentThrottle);
		// Limit maximum power output based on race power curve only if we are already moving
		utils_truncate_number(&CurrentThrottle, 0.0, IsStarting ? 1.0 : PowerCurve_Cruise(CurrentRPM));
		// Set race ramping time
		ramp = config.Ramp_Cruise;
		// Set throttle deadband depending on current state
		deadband = (config.UseDynamicThrottle && IsStarting) ? config.DynamicDeadband_Cruise : config.DeadBand_Cruise;
		break;
	case	DRIVE_MODE_RACE:
		// Apply race mode throttle curve
		CurrentThrottle = ThrottleCurve_Race(CurrentThrottle);
		// Limit maximum power output based on race power curve only if we are already moving
		utils_truncate_number(&CurrentThrottle, 0.0, IsStarting ? 1.0 : PowerCurve_Race(CurrentRPM));
		// Set cruise ramping time
		ramp = config.Ramp_Race;
		// Set throttle deadband depending on current state
		deadband = (config.UseDynamicThrottle && IsStarting) ? config.DynamicDeadband_Race : config.DeadBand_Race;
		break;
	}
	// Apply throttle deadband
	if (CurrentThrottle < deadband) {
		CurrentThrottle = 0.0;
	}
	// If we are starting from a standstill
	if (IsStarting) {
		// Smart ramping
		if (config.UseVariableRamping && CurrentRPM <= config.LowToHighERPM) {
			// Calculate variable ramp time
			float smart_ramp = utils_map(CurrentRPM, 0, config.LowToHighERPM, config.RampingMultiplier * ramp, ramp);
			// Normalize and set variable ramp time
			utils_truncate_number(&smart_ramp, config.RampingMultiplier * ramp, ramp);
			ramp = smart_ramp;
		}		
		// If we are in the slow phase of starting
		if (CurrentRPM <= config.LowAmpLimitERPM) {
			// Limit maximum current to LowAmpLimit
			utils_truncate_number(&CurrentThrottle, 0.0, config.LowAmpLimit / MCCONF_L_CURRENT_MAX);
		}
		// If we are in the smoothing phase of starting
		if (CurrentRPM > config.LowAmpLimitERPM && CurrentRPM <= config.LowToHighERPM) {
			// Limit maximum current with linear smoothing so it won't violently deliver maximum power after starting
			float max_amps = utils_map(CurrentRPM, config.LowAmpLimitERPM, config.LowToHighERPM, config.LowAmpLimit, MCCONF_L_CURRENT_MAX);
			if (config.UseDynamicThrottle) {
				// Map throttle in maximum range to avoid violent jerking
				CurrentThrottle = utils_map(CurrentThrottle, 0.0, 1.0, deadband, max_amps / MCCONF_L_CURRENT_MAX);
			}
			utils_truncate_number(&CurrentThrottle, 0.0, max_amps / MCCONF_L_CURRENT_MAX);
		}
	}
	// Move throttle towards set goal with maximum step
	if (CurrentThrottle < FilteredThrottle) {
		//ramp = ramp * 2;
		ramp = 1.0; // On release move without limitations
	}
	utils_step_towards(&FilteredThrottle, CurrentThrottle, ramp);
}


void ProcessPower() {
	// Read power sent to the motor and calculate peak value for the last 3 seconds
	#define AverageSize 20
	#define PeakMS 3000
	static int AverageIndex = 0;
	float CurrentPower = mc_interface_get_tot_current_filtered() * GET_INPUT_VOLTAGE();
	static int PowerBuffer[PeakMS / AverageSize];
	static int PowerIndex = 0;
	static int AverageBuffer[AverageSize];
	AverageBuffer[AverageIndex++] = CurrentPower;
	if (AverageIndex >= AverageSize) {
		AverageIndex = 0;
		float Average = 0;
		for (int i = 0; i < AverageSize; i++) {
			Average += AverageBuffer[i];
		}
		Average /= AverageSize;
		PowerBuffer[PowerIndex++] = Average;
		if (PowerIndex >= PeakMS / AverageSize) {
			PowerIndex = 0;
		}
		int MaxPower = 0;
		for (int i = 0 ; i < PeakMS / AverageSize; i++) {
			if (PowerBuffer[i] > MaxPower) {
				MaxPower = PowerBuffer[i];
			}
		}
		PeakPower = MaxPower;
	}
}

void DoBlink() {
	// If there are no faults registered
	if (mc_interface_get_fault() == FAULT_CODE_NONE) {
		// If battery is not low
		if (!LowBattery) {
			// If motor is not running
			if (mc_interface_get_state() == MC_STATE_OFF) {
				BatteryBlink(BatteryLevel);
			}
			else {
				// Reset battery indicator
				BlinksLeft = 0;
				// Depending on race mode blink LED or keep it off
				switch (drivemode) {
				case DRIVE_MODE_CRUISE:
					THROTTLE_LED_OFF();
					break;
				case DRIVE_MODE_RACE:
					// Blinking while battery is low will hide low battery error
					if (!LowBattery) {
						FastBlink();
					}
					break;
				}
			}
		}
		else {
			// On low battery kepp the LED on to indicate low battery
			LowBatteryBlink();
		}
	}
	else {
		// Fault codes are a priority
		FaultBlink();
	}
}

void PowerCruise() {
	static int LastPower = 100;
	static bool Active = false;
	if (CruiseControlActive && !Active) {
		LastPower = 0;
		Active = true;
	}
	if (!CruiseControlActive && Active) {
		Active = false;
	}
	#define PowerCruiseRamp 0.0008
	int TargetPower = PeakPower;
	if (TargetPower > 100.0) {
		utils_truncate_number(&TargetPower, 100.0, config.MaxCruisePower);
		utils_step_towards(&LastPower, TargetPower, PowerCruiseRamp);
		float RequiredCurrent = (float)LastPower / (float)GET_INPUT_VOLTAGE();
		utils_truncate_number(&RequiredCurrent, 2.0, 15.0);
		mc_interface_set_current(RequiredCurrent);
	}	
}

void DoControl() {
	// If motor is locked
	if (MotorStopped || SoftLock) {
		// Deactivate cruise control
		if (CruiseControlActive) {
			CruiseControlActive = false;
		}
		if (SoftLock) {
			// If throttle input is above limit and battery is not empty or timeout passed
			if ((FilteredThrottle > SoftLockUnlock || (float)ST2MS(chVTTimeElapsedSinceX(LastSoftLock)) > SoftLockDelay) && !LowBattery) {
				// Unlock the motor
				StartMotor();
			}
		} else {
			// If throttle input is active and battery is not empty
			if (FilteredThrottle > 0.01 && !LowBattery) {
				// Unlock the motor
				StartMotor();
			}
		}
	} else {
		if (CruiseControlActive) {
			// Stop cruise control on throttle input or on low RPMs
			if (FilteredThrottle > 0.01 || CurrentRPM < config.CruiseControlERPM) {
				CruiseControlActive = false;
				// Lock motor until throttle input is active again
				StopMotor();
			}
			// Added to override the PID cruise control
			if (config.UsePowerCruise) {
				PowerCruise();
			}
		} else {
			// If throttle input is active
			if (FilteredThrottle > 0.01) {
				if (CurrentRPM >= config.NegativeERPMLimit) {
					// Give power to motor
					mc_interface_set_current(FilteredThrottle * MCCONF_L_CURRENT_MAX);
				} else {
					// Stop motor below negative ERPM limit to avoid reverse braking
					StopMotor();
				}
			} else {
				if (CurrentRPM > MCCONF_S_PID_MIN_RPM) {
					// Give minimum amperage above minimum ERPM
					mc_interface_set_current(MCCONF_CC_MIN_CURRENT);
				} else {
					// Lock motor in case of ERPM drop
					StopMotor();
				}
			}
		}
	}
	// If motor drops below minimum ERPMs and no throttle is applied
	if (FilteredThrottle < 0.01 && CurrentRPM < config.CruiseControlERPM) {
		// Lock motor
		StopMotor();
	}
}

// Thread

static THD_FUNCTION(custom_thread, arg) {
	// Prepare the thread
	(void) arg;
	chRegSetThreadName("APP_CUSTOM");
	//palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
	is_running = true;
	chThdSleep(MS2ST(2500));
	set_pw(0.0);
	// Run continuously
	for (;;) {
		// Do the necessary thread keeping
		if (!ProcessThread()) return;

		// Process button patterns
		ProcessButtons();

		// Measure battery level
		ProcessBattery();

		// Process motor ERPM
		ProcessRPM();

		// Process motor temperature
		ProcessTemperature();

		// Process throttle input
		ProcessThrottle();

		if (config.UsePowerCruise && !CruiseControlActive) {
			// Process applied power for power cruise control
			ProcessPower();
		}
		// Blink the LED based on current status
		DoBlink();

		if (mc_interface_get_fault() == FAULT_CODE_NONE) {
			// Control motor only if no faults are present
			DoControl();
		}

		// Reset thread timeout. Used to avoid blocking threads.
		timeout_reset();
	}
}

#endif