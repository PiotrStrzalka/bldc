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
#include "datatypes.h"
#include "conf_general.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(fdrive_thread, arg);
static THD_WORKING_AREA(fdrive_thread_wa, 2048);

// Private functions
// static void pwm_callback(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool reload_config = false;

typedef struct
{
    uint32_t attach_time_ms;
    uint32_t cool_down_time_ms;
    float attach_speed_final_per;
    float attach_current_thr;
    uint32_t attempt_count_limit;
    uint32_t speed_goal_kmh;
    uint32_t max_power_adc;
} app_fdrive_config_type;

char *config_desc_str = "1) attach_time_ms\n2) cool_down_time_ms\n3) attach_speed_final_per\n"
                        "4) attach_current_thr\n5) attempt_count_limit\n6) speed_goal_kmh\n7) max_power_adc";

#define APP_FDRIVE_CONFIG_LENGTH sizeof(app_fdrive_config_type) / sizeof(eeprom_var)
//todo check if it is really helpful
typedef union
{
    app_fdrive_config_type config_struct;
    eeprom_var config_table[APP_FDRIVE_CONFIG_LENGTH];
} config_union;

config_union app_fdrive_config;

static void load_config_default(config_union *config)
{
    commands_printf("Problem with loading config, default settings used");
    config->config_struct.attach_time_ms = 800UL;
    config->config_struct.cool_down_time_ms = 300.0f;
    config->config_struct.attach_speed_final_per = 80.0f;
    config->config_struct.attach_current_thr = 2.0f;
    config->config_struct.attempt_count_limit = 5U;
    config->config_struct.speed_goal_kmh = 25U;
    config->config_struct.max_power_adc = 250U;
}

static bool load_config_from_nvm(config_union *config)
{
    for (uint32_t i = 0; i < APP_FDRIVE_CONFIG_LENGTH; i++)
    {
        bool res = conf_general_read_eeprom_var_custom(&(config->config_table[i]), i);
        if (res == false)
        {
            load_config_default(config);
            return false;
        }
    }
    return true;
}

static bool save_config_to_nvm(config_union *config)
{
    for (uint32_t i = 0; i < APP_FDRIVE_CONFIG_LENGTH; i++)
    {
        bool res = conf_general_store_eeprom_var_custom(&(config->config_table[i]), i);
        if (res == false)
        {
            return false;
        }
    }
    return true;
}

static void f_dump_config(int argc, const char **argv)
{
    (void)argc;
    (void)argv;
    if (load_config_from_nvm(&app_fdrive_config) == true)
    {
        commands_printf("Current settings ( order [ %s ]):\n"
                        "[ %lu %lu %.2f %.2f %lu %lu %lu ]",
                        config_desc_str,
                        app_fdrive_config.config_struct.attach_time_ms, app_fdrive_config.config_struct.cool_down_time_ms,
                        (double)app_fdrive_config.config_struct.attach_speed_final_per, (double)app_fdrive_config.config_struct.attach_current_thr,
                        app_fdrive_config.config_struct.attempt_count_limit, app_fdrive_config.config_struct.speed_goal_kmh,
                        app_fdrive_config.config_struct.max_power_adc);
    }
    else
    {
        commands_printf("Problem with loading data from nvm");
    }
}

static void f_store_config(int argc, const char **argv)
{
    if (argc != APP_FDRIVE_CONFIG_LENGTH + 1)
    {
        commands_printf("Not enough arguments, should be %lu"
                        "[ %s ]",
                        APP_FDRIVE_CONFIG_LENGTH, config_desc_str);
        return;
    }
    else
    {
        sscanf(argv[1], "%lu", &app_fdrive_config.config_struct.attach_time_ms);
        sscanf(argv[2], "%lu", &app_fdrive_config.config_struct.cool_down_time_ms);
        sscanf(argv[3], "%f", &app_fdrive_config.config_struct.attach_speed_final_per);
        sscanf(argv[4], "%f", &app_fdrive_config.config_struct.attach_current_thr);
        sscanf(argv[5], "%lu", &app_fdrive_config.config_struct.attempt_count_limit);
        sscanf(argv[6], "%lu", &app_fdrive_config.config_struct.speed_goal_kmh);
        sscanf(argv[7], "%lu", &app_fdrive_config.config_struct.max_power_adc);

        commands_printf("Interpreted settings: "
                        "[ %.2lu %.2lu %.2f %.2f %lu %lu %lu]",
                        app_fdrive_config.config_struct.attach_time_ms, app_fdrive_config.config_struct.cool_down_time_ms,
                        (double)app_fdrive_config.config_struct.attach_speed_final_per, (double)app_fdrive_config.config_struct.attach_current_thr,
                        app_fdrive_config.config_struct.attempt_count_limit, app_fdrive_config.config_struct.speed_goal_kmh,
                        app_fdrive_config.config_struct.max_power_adc);

        if (save_config_to_nvm(&app_fdrive_config) == true)
        {
            reload_config = true;
            commands_printf("Config saved successfully.");
        }
        else
        {
            commands_printf("Problem with config saving!");
        }
    }
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
    stop_now = false;
    chThdCreateStatic(fdrive_thread_wa, sizeof(fdrive_thread_wa),
                      NORMALPRIO, fdrive_thread, NULL);
    terminal_register_command_callback(
        "f_dump_config",
        "Prints actual config",
        "",
        f_dump_config);

    terminal_register_command_callback(
        "f_store_config",
        "Stores new config data:\n",
        "1) attach_time_ms\n2) cool_down_time_ms\n3) attach_speed_final_per\n"
        "4) attach_current_thr\n5) attempt_count_limit\n6) speed_goal_kmh\n7) max_power_adc",
        f_store_config);
    reload_config = true;
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
    //mc_interface_set_pwm_callback(0);
    // terminal_unregister_callback(start_plot);

    stop_now = true;
    while (is_running)
    {
        chThdSleepMilliseconds(1);
    }
}

void app_custom_configure(app_configuration *conf)
{
    (void)conf;
}

#define THREAD_SLEEP_MS 10


#define PAS_ENCODER_EVENT_MASK EVENT_MASK(0)
//#define SPEED_GOAL_RPM                          2157.0f /*25km/h*/
#define MOTOR_POLES 7.0f
#define SPEED_GOAL_ERPM_FROM_KMH(kmh) (float)(((float)kmh * 86.311f) * MOTOR_POLES)
#define MOTOR_ATTACHED_LOG_INTERVAL_MS 1000U
#define SPEED_ATTACHED_THR (SPEED_GOAL_ERPM_FROM_KMH((float)app_fdrive_config.config_struct.speed_goal_kmh) * ((float)app_fdrive_config.config_struct.attach_speed_final_per / 100.0f))
#define FILTER_SAMPLES      5
#define NO_EVENT            -1


static bool motor_attached(void)
{
    float rpm = mc_interface_get_rpm();
    rpm = rpm > 0.0 ? rpm : -rpm; 
    float current = mc_interface_get_tot_current();
    static systime_t true_log_timestamp = 0U;

    if ((rpm > SPEED_ATTACHED_THR) &&
        (current < app_fdrive_config.config_struct.attach_current_thr))
    {
        commands_printf("Motor not attached rpm: %.2f, thr rpm: %.2f, goal_rpm: %.2f, current %.2f", (double)rpm, (double)SPEED_ATTACHED_THR,
                        (double)SPEED_GOAL_ERPM_FROM_KMH(app_fdrive_config.config_struct.speed_goal_kmh), (double)current);
        return false;
    }
    else
    {
        if ((MOTOR_ATTACHED_LOG_INTERVAL_MS * 10U) < chVTTimeElapsedSinceX(true_log_timestamp))
        {
            commands_printf("Motor attached rpm: %.2f, thr rpm: %.2f, goal_rpm: %.2f, current %.2f", (double)rpm, (double)SPEED_ATTACHED_THR,
                            (double)SPEED_GOAL_ERPM_FROM_KMH(app_fdrive_config.config_struct.speed_goal_kmh), (double)current);
            true_log_timestamp = chVTGetSystemTime();
        }
        return true;
    }
    //todo what in another states?
}

/* maybe we can deffer this task somewhere else to not stop main thread*/
static void make_motor_bip(uint32_t how_many_times)
{
    float curr;
    float ld_lq_diff;
    while(how_many_times--){
        mcpwm_foc_measure_inductance(0.85f, 100, &curr, &ld_lq_diff);
        chThdSleepMilliseconds(50);
    }
}

typedef enum
{
    DRIVE_NOT_ARMED = 0U,
    DRIVE_ARMED_IDLE,
    DRIVE_ARMED_TRIGGERED_JUMP,
    DRIVE_ARMED_TRIGGERED_RUNNING,
    DRIVE_ARMED_TRIGGERED_COOL_DOWN
} fdrive_state_type;
static const char *fdrive_state_type_to_string(fdrive_state_type state)
{
    switch (state)
    {
        case DRIVE_NOT_ARMED:
            return "DRIVE_NOT_ARMED";
        case DRIVE_ARMED_IDLE:
            return "DRIVE_ARMED_IDLE";
        case DRIVE_ARMED_TRIGGERED_JUMP:
            return "DRIVE_ARMED_TRIGGERED_JUMP";
        case DRIVE_ARMED_TRIGGERED_RUNNING:
            return "DRIVE_ARMED_TRIGGERED_RUNNING";
        case DRIVE_ARMED_TRIGGERED_COOL_DOWN:
            return "DRIVE_ARMED_TRIGGERED_COOL_DOWN";
        default:
            break;
    }
    return "?";
}

enum fdrive_events{
    FEVENTS_NO_EVENT = 0U,
    FEVENTS_TURN_BACK_DET,
    FEVENTS_RESTORE_STATE
};

static void store_error_in_nvm(void){
    commands_printf("To implement!");
}

static void app_fdrive_SM(enum fdrive_events event)
{
    static fdrive_state_type fdrive_state = DRIVE_NOT_ARMED;
    static fdrive_state_type fdrive_state_prev = DRIVE_NOT_ARMED;
    static bool entry_condition = TRUE;
    static uint32_t overall_errors = 0U;
    static uint32_t attempts_left = 0U;
    static systime_t time_pt = 0U;

    if ((reload_config == true) && ((fdrive_state == DRIVE_NOT_ARMED) || (fdrive_state == DRIVE_ARMED_IDLE))) {
        reload_config = false;
        load_config_from_nvm(&app_fdrive_config);
        commands_printf("Speed goal: %dkmh/h, erpm: %.2f ", app_fdrive_config.config_struct.speed_goal_kmh,
                        (double)SPEED_GOAL_ERPM_FROM_KMH(app_fdrive_config.config_struct.speed_goal_kmh));
    }

    if(event == FEVENTS_RESTORE_STATE){
        if(fdrive_state != DRIVE_NOT_ARMED){
            fdrive_state = DRIVE_ARMED_IDLE;
        }
        entry_condition = TRUE;
    }    

    switch(fdrive_state)
    {
        case DRIVE_NOT_ARMED:
            if(entry_condition){
                mc_interface_release_motor();
            }
            if(event == FEVENTS_TURN_BACK_DET){
                overall_errors = 0U;
                make_motor_bip(1U);
                fdrive_state = DRIVE_ARMED_IDLE;
            }
            break;

        case DRIVE_ARMED_IDLE:
            if(app_pas_encoder_get_current_state() == MOV_TYPE_MOV_FORWARD){
                if(motor_attached() == TRUE){
                    mc_interface_set_current_rel(1.0f);
                    // mc_interface_set_pid_speed(SPEED_GOAL_ERPM_FROM_KMH(app_fdrive_config.config_struct.speed_goal_kmh));
                    fdrive_state = DRIVE_ARMED_TRIGGERED_RUNNING;
                } else {
                    fdrive_state = DRIVE_ARMED_TRIGGERED_JUMP;
                }
            }
            else
            {
                mc_interface_release_motor();
            }

            if (event == FEVENTS_TURN_BACK_DET) {
                while (mc_interface_get_rpm() > 1.0f)
                {
                    commands_printf("Waiting for stop, current stop: %f", (double)mc_interface_get_rpm());
                    /*we cannot bip when motor is running */
                    chThdSleepMilliseconds(50);
                }
                make_motor_bip(2U);
                fdrive_state = DRIVE_NOT_ARMED;
            }

            /*exit condition */
            if(fdrive_state != DRIVE_ARMED_IDLE){
                attempts_left = app_fdrive_config.config_struct.attempt_count_limit;
            }
            break;

        case DRIVE_ARMED_TRIGGERED_JUMP:
            if(entry_condition == TRUE){
                mc_interface_set_current_rel(1.0f);
                //mc_interface_set_pid_speed(SPEED_GOAL_ERPM_FROM_KMH(app_fdrive_config.config_struct.speed_goal_kmh));
                time_pt = chVTGetSystemTime();
            }

            if(app_pas_encoder_get_current_state() != MOV_TYPE_MOV_FORWARD) {
                mc_interface_release_motor();
                fdrive_state = DRIVE_ARMED_IDLE;
            }

            if(((systime_t)app_fdrive_config.config_struct.attach_time_ms * 10U) < chVTTimeElapsedSinceX(time_pt)){
                if(motor_attached() == TRUE){
                    fdrive_state = DRIVE_ARMED_TRIGGERED_RUNNING;
                } else {
                    fdrive_state = DRIVE_ARMED_TRIGGERED_COOL_DOWN;               
                }
            }
            break;

        case DRIVE_ARMED_TRIGGERED_COOL_DOWN:
            if(entry_condition == TRUE){
                mc_interface_brake_now();
                time_pt = chVTGetSystemTime();
                attempts_left--;
            }

            if(app_pas_encoder_get_current_state() != MOV_TYPE_MOV_FORWARD) {
                mc_interface_release_motor();
                fdrive_state = DRIVE_ARMED_IDLE;
            }

            if(chVTTimeElapsedSinceX(time_pt) > (systime_t)app_fdrive_config.config_struct.cool_down_time_ms * 10U){
                if (attempts_left > 0U){
                    fdrive_state = DRIVE_ARMED_TRIGGERED_JUMP;
                } else {                    
                    overall_errors++;
                    if(overall_errors == 4U){ /* attaching definitive error */
                        overall_errors = 0U;
                        store_error_in_nvm();
                    }
                    fdrive_state = DRIVE_ARMED_IDLE;
                    make_motor_bip(3U);
                }
            }
            break;

        case DRIVE_ARMED_TRIGGERED_RUNNING:
            if(entry_condition == TRUE){
                time_pt = chVTGetSystemTime();
            }

            if(app_pas_encoder_get_current_state() != MOV_TYPE_MOV_FORWARD) {
                mc_interface_release_motor();
                fdrive_state = DRIVE_ARMED_IDLE;
            }

            if((attempts_left != 3U) && (chVTTimeElapsedSinceX(time_pt) > 10000U * 10U)){ /* 10 sec */
                attempts_left = 3U;
            }

            if(motor_attached() != TRUE) {
                fdrive_state = DRIVE_ARMED_TRIGGERED_COOL_DOWN;
            }
            break;
       
        default:
            break;
    }

    if (fdrive_state != fdrive_state_prev)
    {
        commands_printf("Fdrive state transition %s -> %s", fdrive_state_type_to_string(fdrive_state_prev), fdrive_state_type_to_string(fdrive_state));
        fdrive_state_prev = fdrive_state;
        entry_condition = TRUE;
    }
    else
    {
        entry_condition = FALSE;
    }
}

float get_filtered_adc_value(void)
{
    float pwr = (float)ADC_Value[ADC_IND_EXT];
    pwr /= 4095;
    pwr *= V_REG;
    static float filter_buffer[FILTER_SAMPLES];
    static int filter_ptr = 0;

    filter_buffer[filter_ptr++] = pwr;
    if (filter_ptr >= FILTER_SAMPLES) {
        filter_ptr = 0;
    }

    pwr = 0.0;
    for (int i = 0;i < FILTER_SAMPLES;i++) {
        pwr += filter_buffer[i];
    }
    pwr /= FILTER_SAMPLES;
    return pwr;
}

#define FDRIVE_APP_ADC_LOWER_THRESHOLD     0.080f
#define FDRIVE_APP_ADC_HIGHER_THRESHOLD    0.120f
typedef enum{
    FDRIVE_APP_ADC = 0,
    FDRIVE_APP_PAS
} app_state_type;

static const char *app_state_type_to_string(app_state_type state)
{
    const char *abc = "";
    switch (state)
    {
    case FDRIVE_APP_ADC:
        abc = "FDRIVE_APP_ADC";
        break;
    case FDRIVE_APP_PAS:
        abc = "FDRIVE_APP_PAS";
        break;
    default:
        break;
    }
    return abc;
}

static int receive_turn_back_event(event_listener_t * el){
    eventmask_t event = chEvtWaitOneTimeout(PAS_ENCODER_EVENT_MASK, TIME_IMMEDIATE);
    if (event & PAS_ENCODER_EVENT_MASK) {
        eventflags_t flags = chEvtGetAndClearFlags(el);
        if (flags & PAS_ENCODER_EVENT_FLAG_TURN_BACK)
        {
            return PAS_ENCODER_EVENT_FLAG_TURN_BACK;
        }
    }
    return -1;
}

static void reset_fault_code(void){
    mc_fault_code fault = mc_interface_get_fault();
    if (fault != FAULT_CODE_NONE) {
        commands_printf("Fault code detected: %s", mc_interface_fault_to_string(fault));
        mc_interface_init();
    }
};

#define LOG_TIME_MS     3000U

static volatile adc_config adcConfig;

#include "app_fdrive.h"
static volatile adc_config adcConfig;
void app_fdrive_adc_configure(adc_config *conf)
{
    adcConfig = *conf;
}

static float get_adc_max_current(){
    return ((float)app_fdrive_config.config_struct.max_power_adc/GET_INPUT_VOLTAGE());
}

static THD_FUNCTION(fdrive_thread, arg)
{
    (void)arg;
    chRegSetThreadName("Fdrive Custom");
    mc_interface_set_pwm_callback(0);

    event_listener_t pas_encoder_listener;
    static app_state_type app_state = FDRIVE_APP_PAS;
    static app_state_type previous_app_state = FDRIVE_APP_PAS;
    static bool app_state_entry_condition = TRUE;
    static systime_t log_timestamp = 0U;

    chEvtRegisterMaskWithFlags(&pas_encoder_event_source, &pas_encoder_listener,
                               PAS_ENCODER_EVENT_MASK, PAS_ENCODER_EVENT_FLAG_TURN_BACK);

    for (;;)
    {
        if (stop_now) {
            mc_interface_release_motor();
            is_running = false;
            return;
        }      
        int received_event_flag = receive_turn_back_event(&pas_encoder_listener);
        reset_fault_code();
        float adc = get_filtered_adc_value();       

        float pwr = utils_map(adc, adcConfig.voltage_start, adcConfig.voltage_end, 0.0f, get_adc_max_current());
        float pwr_rel = utils_map(adc, adcConfig.voltage_start, adcConfig.voltage_end, 0.0f, 1.0f);
        pwr = pwr < 0.0f ? 0.0f : pwr;
        pwr = pwr > 20.0f ? 20.0f : pwr;

        if(chVTTimeElapsedSinceX(log_timestamp) > (systime_t)LOG_TIME_MS * 10U)
        {
            log_timestamp = chVTGetSystemTime();
            commands_printf("Voltage start: %f, end: %f, max current: %f", adcConfig.voltage_start, adcConfig.voltage_end, get_adc_max_current());
            commands_printf("Current ADC value: %f , pwr: %f", (double)adc, (double)pwr);
        }
        
        switch(app_state)
        {
            case FDRIVE_APP_ADC:
                if(app_state_entry_condition == TRUE){
                    commands_printf("Frdive in adc mode");
                }

                if(received_event_flag != NO_EVENT){
                    commands_printf("Event rejected (%d), application in adc mode", received_event_flag);
                }
                mc_interface_set_current_rel(pwr_rel);
                //mc_interface_set_current(pwr);
                if(adc < adcConfig.voltage_start + FDRIVE_APP_ADC_LOWER_THRESHOLD){
                    app_state = FDRIVE_APP_PAS;
                    break;
                }
                break;
            
            case FDRIVE_APP_PAS:
                if(app_state_entry_condition == TRUE){
                    commands_printf("Restore fdrive state");
                    app_fdrive_SM(FEVENTS_RESTORE_STATE);
                }
                if(received_event_flag == PAS_ENCODER_EVENT_FLAG_TURN_BACK){
                    app_fdrive_SM(FEVENTS_TURN_BACK_DET);               /*todo there is possibility to omit event, redesign it*/ 
                } else {
                    app_fdrive_SM(FEVENTS_NO_EVENT);
                } 
                if(adc > adcConfig.voltage_start + FDRIVE_APP_ADC_HIGHER_THRESHOLD){
                    app_state = FDRIVE_APP_ADC;
                }
                break;

            default:
                break;
        }

        if (app_state != previous_app_state){
            commands_printf("App state transition %s -> %s", app_state_type_to_string(previous_app_state),
                            app_state_type_to_string(app_state));
            previous_app_state = app_state;
            app_state_entry_condition = TRUE;
        } else {
            app_state_entry_condition = FALSE;
        }

        timeout_reset();
        chThdSleepMilliseconds(THREAD_SLEEP_MS);
    }
}



// typedef enum
// {
//     DRIVE_NOT_ARMED = 0U,
//     DRIVE_ARMED_IDLE,
//     DRIVE_ATTACHING,
//     DRIVE_COOL_DOWN,
//     DRIVE_RUNNING
// } arming_state_type;
// static const char *arming_state_type_to_string(arming_state_type state)
// {
//     const char *abc = "";
//     switch (state)
//     {
//     case DRIVE_NOT_ARMED:
//         abc = "DRIVE_NOT_ARMED";
//         break;
//     case DRIVE_ARMED_IDLE:
//         abc = "DRIVE_ARMED_IDLE";
//         break;
//     case DRIVE_ATTACHING:
//         abc = "DRIVE_ATTACHING";
//         break;
//     case DRIVE_COOL_DOWN:
//         abc = "DRIVE_COOL_DOWN";
//         break;
//     case DRIVE_RUNNING:
//         abc = "DRIVE_RUNNING";
//         break;
//     default:
//         break;
//     }
//     return abc;
// }

// static void pwm_callback(void) {
// 	// Called for every control iteration in interrupt context.
// }


// static arming_state_type state = DRIVE_NOT_ARMED;
    // static arming_state_type previous_state = DRIVE_NOT_ARMED;
        // static bool entry_condition = FALSE;
    // static uint32_t attach_attemps_cnt = 0U;
    // static systime_t time_pt = 0U;

        // switch (state)
        // {
        //     case DRIVE_NOT_ARMED:
        //         if (received_event_flag == PAS_ENCODER_EVENT_FLAG_TURN_BACK)
        //         {
        //             received_event_flag = -1;
        //             make_motor_bip();
        //             state = DRIVE_ARMED_IDLE;
        //         }
        //         break;
        //     case DRIVE_ARMED_IDLE:
        //         if (entry_condition == TRUE)
        //         {
        //             mc_interface_release_motor();
        //         }

        //         if (received_event_flag == PAS_ENCODER_EVENT_FLAG_TURN_BACK)
        //         {
        //             received_event_flag = -1;
        //             while (mc_interface_get_rpm() > 1.0f)
        //             {
        //                 commands_printf("Waiting for stop, current stop: %f", mc_interface_get_rpm());
        //                 /*we cannot bip when motor is running */
        //                 chThdSleepMilliseconds(50);
        //             }
        //             make_motor_bip();
        //             chThdSleepMilliseconds(50);
        //             make_motor_bip();
        //             state = DRIVE_NOT_ARMED;
        //         }

        //         if (app_pas_encoder_get_current_state() == MOV_TYPE_MOV_FORWARD)
        //         {
        //             commands_printf("motor_start()");
        //             mc_interface_set_current((float)-3.5);
        //             attach_attemps_cnt = app_fdrive_config.config_struct.attempt_count_limit;
        //             state = DRIVE_ATTACHING;
        //             //todo sleep below is too long, some counter needed
        //             //chThdSleepMilliseconds(1000);
        //         }
        //         break;
        //     case DRIVE_ATTACHING:
        //         if (app_pas_encoder_get_current_state() != MOV_TYPE_MOV_FORWARD)
        //         {
        //             state = DRIVE_ARMED_IDLE;
        //         }

        //         if (entry_condition == TRUE)
        //         {
        //             mc_interface_set_pid_speed(SPEED_GOAL_ERPM_FROM_KMH(app_fdrive_config.config_struct.speed_goal_kmh));
        //             time_pt = chVTGetSystemTime();
        //         }

        //         if (((systime_t)app_fdrive_config.config_struct.attach_time_ms * 10U) < chVTTimeElapsedSinceX(time_pt))
        //         {
        //             if (motor_attached() == true)
        //             {
        //                 state = DRIVE_RUNNING;
        //             }
        //             else
        //             {
        //                 state = DRIVE_COOL_DOWN;
        //             }
        //         }
        //         break;
        //     case DRIVE_COOL_DOWN:
        //         if (app_pas_encoder_get_current_state() != MOV_TYPE_MOV_FORWARD)
        //         {
        //             state = DRIVE_ARMED_IDLE;
        //         }

        //         if (entry_condition == TRUE)
        //         {
        //             attach_attemps_cnt--;
        //             mc_interface_brake_now();
        //             time_pt = chVTGetSystemTime();
        //         }

        //         if (((systime_t)app_fdrive_config.config_struct.cool_down_time_ms * 10U) < chVTTimeElapsedSinceX(time_pt) {
        //             if (attach_attemps_cnt > 0U)
        //             {
        //                 state = DRIVE_ATTACHING;
        //             }
        //             else
        //             {
        //                 make_motor_bip();
        //                 chThdSleepMilliseconds(50);
        //                 make_motor_bip();
        //                 chThdSleepMilliseconds(50);
        //                 make_motor_bip();
        //                 chThdSleepMilliseconds(200);
        //                 state = DRIVE_ARMED_IDLE;
        //             }
        //         }
        //         break;
        //     case DRIVE_RUNNING:
        //         if (motor_attached() != true) {
        //             state = DRIVE_COOL_DOWN;
        //         }
        //         if (app_pas_encoder_get_current_state() != MOV_TYPE_MOV_FORWARD) {
        //             commands_printf("motor_stop()");
        //             state = DRIVE_ARMED_IDLE;
        //         }
        //         break;
        //     default:
        //         break;
        // }

        // if (state != previous_state)
        // {
        //     commands_printf("Fdrive state transition %s -> %s", arming_state_type_to_string(previous_state),
        //                     arming_state_type_to_string(state));
        //     previous_state = state;
        //     entry_condition = TRUE;
        // }
        // else
        // {
        //     entry_condition = FALSE;
        // }