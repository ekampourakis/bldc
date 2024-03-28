#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float read_voltage = 0.0;
static volatile bool range_ok = true;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"takis",
			"Print the number d",
			"[d]",
			terminal_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(terminal_test);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	// (void)conf;
	config = conf->app_adc_conf;
	ms_without_power = 0.0;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom Idle");

	is_running = true;

	for(;;) {

		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		// Read the external ADC pin voltage
		float pwr = ADC_VOLTS(ADC_IND_EXT);

		// Read voltage and range check
		static float read_filter = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(read_filter, pwr, FILTER_SAMPLES);

		float read_voltage = 0.0;
		if (config.use_filter) {
			read_voltage = read_filter;
		} else {
			read_voltage = pwr;
		}

		// This needs fixing for the button press that shortcuts ADC to GND
		range_ok = read_voltage >= config.voltage_min && read_voltage <= config.voltage_max;

		pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);

		// Optionally apply a filter
		static float pwr_filter = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(pwr_filter, pwr, FILTER_SAMPLES);

		if (config.use_filter) {
			pwr = pwr_filter;
		}

		// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage_inverted) {
			pwr = 1.0 - pwr;
		}		

		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		// Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);

		// Apply a power curve map to the pwr value
		// Map the pwr value from 0.0-1.0 to min and max Amps
		// TODO: Implement
		pwr = pwr * 20.0f;

		// Apply ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}

		float current_rel = 0.0;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();

		current_rel = pwr;

		timeout_reset(); // Reset timeout if everything is OK.

		mc_interface_set_current_rel(current_rel);



		// Run your logic here. A lot of functionality is available in mc_interface.h.

		// // Process button patterns
		// ProcessButtons();

		// // Measure battery level
		// ProcessBattery();

		// // Process motor ERPM
		// ProcessRPM();

		// // Process motor temperature
		// ProcessTemperature();

		// // Process throttle input
		// ProcessThrottle();

		// if (config.UsePowerCruise && !CruiseControlActive) {
		// 	// Process applied power for power cruise control
		// 	ProcessPower();
		// }
		// // Blink the LED based on current status
		// DoBlink();

		// if (mc_interface_get_fault() == FAULT_CODE_NONE) {
		// 	// Control motor only if no faults are present
		// 	DoControl();
		// }

	}
}

static void pwm_callback(void) {
	// Called for every control iteration in interrupt context.
}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if (d == 69) {
			commands_printf("Noice");
		} else {
			commands_printf("Takis have entered %d", d);
		}

		// For example, read the ADC inputs on the COMM header.
		commands_printf("ADC1: %.2f V ADC2: %.2f V",
				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
	} else {
		commands_printf("This command requires one argument.\n");
	}
}
