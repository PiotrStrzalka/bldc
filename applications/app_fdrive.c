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
#include "app_pas_encoder.h"
#include "mcpwm_foc.h"
#include "chevents.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(fdrive_thread, arg);
static THD_WORKING_AREA(fdrive_thread_wa, 2048);

// Private functions
static void pwm_callback(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	stop_now = false;
	chThdCreateStatic(fdrive_thread_wa, sizeof(fdrive_thread_wa),
			NORMALPRIO, fdrive_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	//mc_interface_set_pwm_callback(0);
	// terminal_unregister_callback(start_plot);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

#define THREAD_SLEEP_MS		10

typedef enum
{
    ARMING_NOT_ARMED = 0U,
    ARMING_ARMED_IDLE,
    ARMING_ARMED_POWER
} arming_state_type;
static const char * arming_state_type_to_string(arming_state_type state)
{
    const char * abc = ""; 
    switch(state)
    {
        case ARMING_NOT_ARMED:
            abc = "ARMING_NOT_ARMED";
            break;
        case ARMING_ARMED_IDLE:
            abc = "ARMING_ARMED_IDLE";
            break;
        case ARMING_ARMED_POWER:
            abc = "ARMING_ARMED_POWER";
            break;
        default:
            break;
    }
    return abc;
}

static void make_motor_bip(void)
{
    float curr;
    float ld_lq_diff;
    mcpwm_foc_measure_inductance(0.85f, 100, &curr, &ld_lq_diff);
}

#define PAS_ENCODER_EVENT_MASK                  EVENT_MASK(0)

static THD_FUNCTION(fdrive_thread, arg) {
	(void)arg;
	chRegSetThreadName("Fdrive Custom");
    mc_interface_set_pwm_callback(0);
    
    static arming_state_type state = ARMING_NOT_ARMED;
    static arming_state_type previous_state = ARMING_NOT_ARMED;
    event_listener_t pas_encoder_listener;
    static int received_event_flag = -1;

    chEvtRegisterMaskWithFlags(&pas_encoder_event_source, &pas_encoder_listener,
        PAS_ENCODER_EVENT_MASK, PAS_ENCODER_EVENT_FLAG_TURN_BACK);


	for(;;) {
		if (stop_now) {
			mc_interface_release_motor();
			is_running = false;
			return;
		}
		timeout_reset(); // Reset timeout if everything is OK.

        eventmask_t event = chEvtWaitOneTimeout(PAS_ENCODER_EVENT_MASK, TIME_IMMEDIATE);
        if(event & PAS_ENCODER_EVENT_MASK)
        {
            eventflags_t flags = chEvtGetAndClearFlags(&pas_encoder_listener);
            if(flags & PAS_ENCODER_EVENT_FLAG_TURN_BACK)
            {
                received_event_flag = PAS_ENCODER_EVENT_FLAG_TURN_BACK;
            }
        }

        mc_fault_code fault = mc_interface_get_fault();

        if(fault != FAULT_CODE_NONE)
        {
            commands_printf("Fault code detected: %s", mc_interface_fault_to_string(fault));
            mc_interface_init();
        }
        

        switch(state)
        {
            case ARMING_NOT_ARMED:
                if(received_event_flag == PAS_ENCODER_EVENT_FLAG_TURN_BACK)
                {
                    received_event_flag = -1;
                    make_motor_bip();
                    state = ARMING_ARMED_IDLE;
                }                
                break;
            case ARMING_ARMED_IDLE:
                if(received_event_flag == PAS_ENCODER_EVENT_FLAG_TURN_BACK)
                {
                    received_event_flag = -1;
                    while(mc_interface_get_rpm() > 1.0f)
                    {
                        commands_printf("Waiting for stop, current stop: %f", mc_interface_get_rpm());
                        /*we cannot bip when motor is running */
                        chThdSleepMilliseconds(50);
                    }
                    make_motor_bip();
                    chThdSleepMilliseconds(50);
                    make_motor_bip();
                    state = ARMING_NOT_ARMED;
                }
                if(app_pas_encoder_get_current_state() == MOV_TYPE_MOV_FORWARD)
                {
                    commands_printf("motor_start()");
                    mc_interface_set_current((float)3.0);	
                    state = ARMING_ARMED_POWER;
                    //todo sleep below is too long, some counter needed
                    //chThdSleepMilliseconds(1000);
                }
                break;
            case ARMING_ARMED_POWER:
                if(app_pas_encoder_get_current_state() != MOV_TYPE_MOV_FORWARD)
                {
                    commands_printf("motor_stop()");
                    mc_interface_release_motor();
                    state = ARMING_ARMED_IDLE;
                }
                break;
            default:
                break;
        }
        
        if(state != previous_state)
        {
            commands_printf("Fdrive state transition %s -> %s", arming_state_type_to_string(previous_state), 
                arming_state_type_to_string(state));
            previous_state = state;
        }

        timeout_reset();
        
		chThdSleepMilliseconds(THREAD_SLEEP_MS);
	}
}

// static void pwm_callback(void) {
// 	// Called for every control iteration in interrupt context.
// }


