/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "app_pas_sensor.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(fdrive_thread, arg);
static THD_WORKING_AREA(fdrive_thread_wa, 2048);

// Private functions
static void pwm_callback(void);
static void start_plot(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	//mc_interface_set_pwm_callback(pwm_callback);
	app_pas_sensor_init();
	//palSetPadMode(HW_PAS_SENSOR_PORT, HW_PAS_SENSOR_PIN, PAL_MODE_INPUT_PULLUP);
	//palSetPad(HW_PAS_SENSOR_PORT, HW_PAS_SENSOR_PIN);
	stop_now = false;
	chThdCreateStatic(fdrive_thread_wa, sizeof(fdrive_thread_wa),
			NORMALPRIO, fdrive_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"start_plot",
			"Start plotting custom application 1 - enabled, 0 - disabled",
			"[d]",
			start_plot);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	//mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(start_plot);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

inline static void print_test_message(void)
{
	static uint32_t i = 0;
	if(i % 200 == 0U)
	{
		commands_printf("Test message: %lu", i);
	}	
	i++;
}

static volatile bool plotEnabled = false;

// Callback function for the terminal command with arguments.
static void start_plot(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		commands_printf("You have entered %d", d);

		if(d == 1){
			if(plotEnabled == false){
				commands_printf("Enabling plot");
				commands_init_plot("Sample", "Counter");
				commands_plot_add_graph("PAS Sensor");				// plot nr 1
				commands_plot_add_graph("motorStarted");			// plot nr 3		
				commands_plot_add_graph("Motor Start Threshold");	// plot nr 4
				plotEnabled = true;
			}else{
				commands_printf("Plot is already enabled!");
			}
		}else if(d == 0){
			commands_printf("Disabling plot");
			plotEnabled = false;
		}else
		{
			commands_printf("Argument is incorrect!");
		}

		// For example, read the ADC inputs on the COMM header.
		//commands_printf("ADC1: %.2f V ADC2: %.2f V",
		//		(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

#define THREAD_SLEEP_MS		10
#define UPDATE_PLOT_TIME	100 
#define UPDATE_PLOT_CNT		UPDATE_PLOT_TIME/THREAD_SLEEP_MS

#define MOTOR_START_THR		25U

static THD_FUNCTION(fdrive_thread, arg) {
	(void)arg;

	static uint8_t previousState = PAL_HIGH;
	static float samp = 0.0;	
	static bool motorStarted = FALSE;
	static uint32_t PAScounter = 0;

	chRegSetThreadName("Fdrive Custom");
	is_running = true;
	
	
	chThdSleepMilliseconds(10);
	previousState = palReadPad(HW_PAS_SENSOR_PORT, HW_PAS_SENSOR_PIN);
	chThdSleepMilliseconds(10);

	for(;;) {
		if (stop_now) {
			mc_interface_release_motor();
			is_running = false;
			return;
		}
		timeout_reset(); // Reset timeout if everything is OK.

		mov_type pas_state = PAS_GetCurrentState();

		if(((pas_state == mov_forward) || (pas_state == mov_backward) || (pas_state == mov_unspecified))
			&& (motorStarted == FALSE))
		{			
			motorStarted = TRUE;		
			commands_printf("t:%.1fms MOTOR START.\n", (float)chVTGetSystemTime()/10.0f);
		} else if((pas_state == mov_no_movement) && (motorStarted == TRUE)){
			mc_interface_release_motor();
			motorStarted = FALSE;
			commands_printf("t:%.1fms MOTOR STOP.\n", (float)chVTGetSystemTime()/10.0f);
		}

		if(motorStarted == TRUE){
			mc_interface_set_current((float)10.0);	//current is limited by current limit in configuration
		}
	
		static int counter = 0;
		if(plotEnabled == true){
			counter++;
			if(counter == UPDATE_PLOT_CNT){
				counter = 0;
				commands_plot_set_graph(0);
				commands_send_plot_points(samp, (float)pas_state);	
				chThdSleepMilliseconds(1U);
				commands_plot_set_graph(1);
				commands_send_plot_points(samp, (float)motorStarted * 10.0);	
				chThdSleepMilliseconds(1U);
				commands_plot_set_graph(3);
				commands_send_plot_points(samp, (float)MOTOR_START_THR);	
				samp++;					
			}			
		}
		chThdSleepMilliseconds(THREAD_SLEEP_MS-3U);
	}
}

static void pwm_callback(void) {
	// Called for every control iteration in interrupt context.
}

