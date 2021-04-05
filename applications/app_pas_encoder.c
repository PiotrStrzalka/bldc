#include "ch.h"
#include "hal.h"
#include "app_pas_sensor.h"
#include "stm32f4xx_conf.h"
#include "isr_vector_table.h"
#include "commands.h"
#include "terminal.h"
#include <stdio.h>
#include "app_pas_encoder.h"
#include "hw.h"
#include "encoder.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"

static THD_FUNCTION(pas_encoder_thread, arg);
static THD_WORKING_AREA(pas_encoder_thread_wa, 2048);
static bool plotEnabled = false;
static void pas_plot_init(int argc, const char **argv);
static void update_plot(uint32_t encoder_position);

#define ENCODER_RELOAD_VALUE        1000
#define FULL_CIRCLE_ENCODER_STEPS   24U
#define SPEED_AVG_CONSTANT          5
#define THREAD_SLEEP_TIME           50U
#define BACKWARD_MOVEMENT_TURN_THR  -24
#define MOV_FORWARD_SPEED_THR       70.0f
#define MOV_BACKWARD_SPEED_THR      -70.0f
#define MOV_SPEED_HYSTERESIS        30.0f

event_source_t pas_encoder_event_source;
static mov_type_enum state = MOV_TYPE_NO_MOVE_OR_TOO_SLOW;

void app_pas_encoder_init(void)
{
    chEvtObjectInit(&pas_encoder_event_source);
    chThdCreateStatic(pas_encoder_thread_wa, sizeof(pas_encoder_thread_wa),
        NORMALPRIO, pas_encoder_thread, NULL);
    
    encoder_init_abi(ENCODER_RELOAD_VALUE);

    // Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"pas_plot",
			"Start plotting pas sensor calculation results 1 - enabled, 0 - disabled",
			"[d]",
			pas_plot_init);
}

void app_pas_encoder_deinit(void)
{
    //todo strzalkap - disable interrupt here
}

static const char * mov_state_type_to_string(mov_type_enum st)
{
    const char * abc = ""; 
    switch(st)
    {
        case MOV_TYPE_NO_MOVE_OR_TOO_SLOW:
            abc = "MOV_TYPE_NO_MOVE_OR_TOO_SLOW";
            break;
        case MOV_TYPE_MOV_FORWARD:
            abc = "MOV_TYPE_MOV_FORWARD";
            break;
        case MOV_TYPE_MOV_BACKWARD:
            abc = "MOV_TYPE_MOV_BACKWARD";
            break;
        default:
            break;
    }
    return abc;
}

mov_type_enum app_pas_encoder_get_current_state(void)
{
    return state;
}

static void send_event_about_turn_back(void)
{
    chEvtBroadcastFlags(&pas_encoder_event_source, PAS_ENCODER_EVENT_FLAG_TURN_BACK);
    // chEvtBroadcast(&pas_encoder_event_source);
}

//{
//         typedef unsigned int uint32_t;
//     #define ENCODER_RELOAD_VALUE        10000
//     static int calculate_encoder_delta(uint32_t previous_value, uint32_t actual_value)
//     {
//         int delta = actual_value - previous_value;
//         if(delta < -(ENCODER_RELOAD_VALUE/2))
//         {
//             delta = delta + (int)ENCODER_RELOAD_VALUE;
//         }
//         else if (delta > ENCODER_RELOAD_VALUE/2)
//         {
//             delta = delta - ENCODER_RELOAD_VALUE;
//         }
//         return delta;
//     }
//     int main()
//     {
//         printf("Should be -20: %d\n", calculate_encoder_delta(10U, 9990U));
//         printf("Should be  10: %d\n", calculate_encoder_delta(0U, 10U));
//         printf("Should be -10: %d\n", calculate_encoder_delta(0U, 9990U));
//         printf("Should be -10: %d\n", calculate_encoder_delta(10U, 0U));
//         printf("Should be -20: %d\n", calculate_encoder_delta(10U, 9990U));
//         printf("Should be 10: %d\n", calculate_encoder_delta(9980U, 9990U));
//         printf("Should be -10: %d\n", calculate_encoder_delta(9990U, 9980U));
//         printf("Should be 30: %d\n", calculate_encoder_delta(9980U, 10U));
//         return 0;
//}
static float position_delta = 0.0f;
static float speed_running_avg = 0.0f;
static float speed = 0.0f;
static float angle_delta = 0.0f;
static int calculate_encoder_delta(uint32_t previous_value, uint32_t actual_value)
{
    int delta = actual_value - previous_value;

    if(delta < -(ENCODER_RELOAD_VALUE/2))
    {
        delta = delta + (int)ENCODER_RELOAD_VALUE;
    }
    else if (delta > ENCODER_RELOAD_VALUE/2)
    {
        delta = delta - ENCODER_RELOAD_VALUE;
    }
    return delta;
}

static void calculate_action_from_position(uint32_t encoder_position){
    static uint32_t previous_encoder_position = 0U;    
    static mov_type_enum previous_state = MOV_TYPE_NO_MOVE_OR_TOO_SLOW;
    static uint32_t backward_turn_detection_start_position = 0U;
    static bool turn_back_detected = false;
    
    position_delta = (float) calculate_encoder_delta(previous_encoder_position, encoder_position);
    angle_delta = ((float)position_delta/(float)FULL_CIRCLE_ENCODER_STEPS)*360.0f;
    speed = (angle_delta * 1000.0f)/ (float)THREAD_SLEEP_TIME;

    speed_running_avg = (speed_running_avg * (float)(SPEED_AVG_CONSTANT-1) + speed)/(float)SPEED_AVG_CONSTANT;

    switch(state)
    {
        case MOV_TYPE_NO_MOVE_OR_TOO_SLOW:
            if(speed_running_avg > MOV_FORWARD_SPEED_THR){
                state = MOV_TYPE_MOV_FORWARD;
            }
            else if(speed_running_avg < MOV_BACKWARD_SPEED_THR){
                state = MOV_TYPE_MOV_BACKWARD;
            }
            else{
                /*do nothing */
            }
            break;
        case MOV_TYPE_MOV_FORWARD:
            if(speed_running_avg < (MOV_FORWARD_SPEED_THR - MOV_SPEED_HYSTERESIS))
            {
                state = MOV_TYPE_NO_MOVE_OR_TOO_SLOW;
            }
            break;
        case MOV_TYPE_MOV_BACKWARD:
            if(speed_running_avg > (MOV_BACKWARD_SPEED_THR + MOV_SPEED_HYSTERESIS))
            {
                state = MOV_TYPE_NO_MOVE_OR_TOO_SLOW;
            }
            break;
        default:
            commands_printf("Something goes wrong");
            break;
    }

    // if(state != previous_state)
    // {
    //     commands_printf("State transition %s -> %s", 
    //         mov_state_type_to_string(previous_state), mov_state_type_to_string(state));
    // }
    
    if(state == MOV_TYPE_MOV_BACKWARD)
    {
        if(previous_state != MOV_TYPE_MOV_BACKWARD)
        {
            backward_turn_detection_start_position = encoder_position;
        }

        if(turn_back_detected == false){
            if(calculate_encoder_delta(backward_turn_detection_start_position, encoder_position) < BACKWARD_MOVEMENT_TURN_THR)
            {
                send_event_about_turn_back();
                commands_printf("Turn back detected");
                turn_back_detected = true;
            }
        }       
    }
    else
    {
        turn_back_detected = false;
    }

    previous_encoder_position = encoder_position;
    previous_state = state;
}

static uint32_t encoder_position = 0U;

static THD_FUNCTION(pas_encoder_thread, arg){
    (void)arg;
    chRegSetThreadName("PAS Encoder");

    for(;;){
        encoder_position = HW_ENC_TIM->CNT;
        if(plotEnabled == true){
            update_plot(encoder_position);
        }

        calculate_action_from_position(encoder_position);
        //32 ticks on one turn
        chThdSleepMilliseconds(THREAD_SLEEP_TIME);
    }
}


#define PLOT_UPDATE_TIME 200U
#define PLOT_UPDATE_CYCLES PLOT_UPDATE_TIME/THREAD_SLEEP_TIME
//Callback function for terminal
static void pas_plot_init(int argc, const char **argv)
{
    if (argc == 2) 
    {
        int d = -1;
        sscanf(argv[1], "%d", &d);
        commands_printf("PAS - You have entered %d", d);

        if(d == 1){
            if(plotEnabled == false){
                commands_printf("Enabling plot");
                commands_init_plot("Value", "Time");
                commands_plot_add_graph("Encoder Value");
                commands_plot_add_graph("Angle delta");
                commands_plot_add_graph("Position delta");
                commands_plot_add_graph("State Value");
                commands_plot_add_graph("Speed Value");
                commands_plot_add_graph("Avg speed Value");
                plotEnabled = true;
            }else{
                commands_printf("Plot is already enabled!");
            }
        }else if(d == 0){
            commands_printf("Disabling plot");
            plotEnabled = false;
        }else if (d == 2)
        {
            commands_printf("Counter value (tim4): %d", HW_ENC_TIM->CNT);
        }else
        {
            commands_printf("Argument is incorrect!");
        }
    } else {
        commands_printf("This command requires one argument.\n");
    }
}

static void update_plot(uint32_t encoder_pos)
{   
    static uint32_t cyclesCounter = 0U;
    cyclesCounter++;
    if(cyclesCounter >= PLOT_UPDATE_CYCLES)
    {
        cyclesCounter = 0U;
        float time = (float)((float)chVTGetSystemTimeX() / 10000.0F);

        commands_plot_set_graph(0);
        commands_send_plot_points(time, (float)encoder_pos);

        commands_plot_set_graph(1);
        commands_send_plot_points(time, (float)angle_delta);

        commands_plot_set_graph(2);
        commands_send_plot_points(time, (float)position_delta);
        
        commands_plot_set_graph(3);
        commands_send_plot_points(time, (float)state*100.0f);

        commands_plot_set_graph(4);
        commands_send_plot_points(time, (float)speed);

        commands_plot_set_graph(5);
        commands_send_plot_points(time, (float)speed_running_avg);


    }
}
