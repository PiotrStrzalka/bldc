#include "ch.h"
#include "hal.h"
#include "app_pas_sensor.h"
#include "stm32f4xx_conf.h"
#include "isr_vector_table.h"
#include "commands.h"
#include "terminal.h"
#include <stdio.h>

#define NUM_BUFFERS 32
//#define BUFFERS_SIZE 256

typedef struct {
    uint32_t timestamp;
    uint8_t pin_state;
} pas_edge;



static pas_edge buffers[NUM_BUFFERS]; 
static msg_t pas_events_queue[NUM_BUFFERS];
static mailbox_t pas_events;
static msg_t pas_events_queue_free[NUM_BUFFERS];
static mailbox_t pas_events_free;

static THD_FUNCTION(pas_sensor_thread, arg);
static THD_WORKING_AREA(pas_sensor_thread_wa, 2048);

const char * pinLevelToString(uint8_t pinLevel);
const char * movTypeToString(mov_type mov);

static volatile mov_type mov_current = mov_no_movement;

static void pas_plot_init(int argc, const char **argv);

mov_type PAS_GetCurrentState(void)
{
    return mov_current;
}


// PAS sensor input Interrupt Handler
CH_IRQ_HANDLER(HW_PAS_SENSOR_EXTI_ISR_VEC){
    if(EXTI_GetITStatus(HW_PAS_SENSOR_EXTI_LINE) != RESET){
        void *pbuf;
        chSysLockFromISR();
        uint8_t pinLevel = palReadPad(HW_PAS_SENSOR_PORT, HW_PAS_SENSOR_PIN);
        EXTI_ClearITPendingBit(HW_PAS_SENSOR_EXTI_LINE);
        if(chMBFetchI(&pas_events_free, (msg_t *) &pbuf) == MSG_OK){
            ((pas_edge *) pbuf)->pin_state = pinLevel;
            ((pas_edge *) pbuf)->timestamp = (uint32_t) chVTGetSystemTimeX();
            (void)chMBPostI(&pas_events, (msg_t) pbuf);
        }

        chSysUnlockFromISR();
    }
}

void app_pas_sensor_init(void)
{
    chMBObjectInit(&pas_events, pas_events_queue, NUM_BUFFERS);
    chMBObjectInit(&pas_events_free, pas_events_queue_free, NUM_BUFFERS);

    for(uint32_t i = 0; i < NUM_BUFFERS; i++){
        (void) chMBPostI(&pas_events_free, (msg_t)&buffers[i]);
    }

    palSetPadMode(HW_PAS_SENSOR_PORT, HW_PAS_SENSOR_PIN, PAL_MODE_INPUT_PULLUP);

    SYSCFG_EXTILineConfig(HW_PAS_SENSOR_PORT_INT, HW_PAS_SENSOR_PIN_INT);
    EXTI_InitTypeDef   EXTI_InitStructure;
    // Configure EXTI Line
    EXTI_InitStructure.EXTI_Line = HW_PAS_SENSOR_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    nvicEnableVector(HW_PAS_SENSOR_EXTI_CH, 2);

    chThdCreateStatic(pas_sensor_thread_wa, sizeof(pas_sensor_thread_wa),
        NORMALPRIO, pas_sensor_thread, NULL);

    // Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"pas_plot",
			"Start plotting pas sensor calculation results 1 - enabled, 0 - disabled",
			"[d]",
			pas_plot_init);
}

void pas_sensor_deinit(void)
{
    //todo strzalkap - disable interrupt here
}

#define THREAD_SLEEP_TIME 20U


static float mean_Tu_Td = 0.0f;
static float mean_frequency = 0.0f;

#define NO_MOVEMENT_DEBOUNCING_TIME 600U
#define NO_MOVEMENT_DEBOUNCING_CYCLES  NO_MOVEMENT_DEBOUNCING_TIME/THREAD_SLEEP_TIME
#define AVERAGE_PARAM   3.0f

void calculateParams(void)
{    
    static mov_type mov_previous = mov_no_movement;
    static uint32_t falling_slope_time = 0U;
    static uint32_t rising_slope_time = 0U;
    static uint32_t Tu = 1U;
    static uint32_t Td = 1U;
    static float frequency = 0.0f;
    static pas_edge previousEvent;

    msg_t ret;        
    do{
        pas_edge *eventBuf;
        ret = chMBFetchI(&pas_events, (msg_t *) &eventBuf);
        if(ret == MSG_OK){

            if((eventBuf->pin_state == 0U) && (previousEvent.pin_state == 1U)){ // falling slope 
                frequency =  (float)CH_CFG_ST_FREQUENCY/(float)(eventBuf->timestamp - falling_slope_time);
                falling_slope_time = eventBuf->timestamp;            
                Tu = falling_slope_time - rising_slope_time;
            }
            else if((eventBuf->pin_state == 1U) && (previousEvent.pin_state == 0U)){ // raising slope  
                frequency = (float)CH_CFG_ST_FREQUENCY/(float)(eventBuf->timestamp - rising_slope_time);
                rising_slope_time = eventBuf->timestamp;
                Td = rising_slope_time - falling_slope_time;
            }
            else{
                commands_printf("Same events! time: %lums, state: %u", eventBuf->timestamp/10U, eventBuf->pin_state);
            }    
            mean_frequency = (mean_frequency * (AVERAGE_PARAM - 1.0f) + frequency) / AVERAGE_PARAM;    
            float Tu_Td = (float)Tu*100.0f/(float)(Tu+Td);
            mean_Tu_Td = (mean_Tu_Td * (AVERAGE_PARAM - 1.0f) + Tu_Td) / AVERAGE_PARAM;

            (void) chMBPostI(&pas_events_free, (msg_t) eventBuf);
            previousEvent = *eventBuf;          
        }
    } while(ret == MSG_OK);

    //commands_printf("Timestamp: %lu, state: %lu", ((pas_edge *)pbuf)->timestamp, ((pas_edge *)pbuf)->pin_state);
    mov_type mov_temp = mov_unspecified;
    //static const char * result = "Not sure";
    if((mean_frequency > 0.8f)){
        if(mean_Tu_Td > 45.0f){
            mov_temp = mov_forward;
        } else {
            mov_temp = mov_backward;
        }            
    }

    mov_current = mov_temp;

    if(chVTGetSystemTime() > (previousEvent.timestamp + NO_MOVEMENT_DEBOUNCING_TIME)){
        //commands_printf("No action from two seconds!");
        mov_current = mov_no_movement;
        frequency = 0.0f;
        mean_Tu_Td = 0.0f;
        mean_frequency = 0.0f;
    }

    if(mov_current != mov_previous){
        commands_printf("t:%.1fms Change %s -> %s", ((float)chVTGetSystemTime())/10.0f, movTypeToString(mov_previous), movTypeToString(mov_current));
        mov_previous = mov_current;
    }
    //commands_printf("%s, t:%.1fms d: %.1fms, pin %s, freq: %.2fHz | %.2fHz, Tu: %.1f, Td: %.1f, Tu/Td: %.1f | Tu/Td %.1f" ,
      //  movTypeToString(result), (float)event->timestamp/10.0f, (float)(event->timestamp - previousEvent.timestamp)/10.0f,
        //pinLevelToString(event->pin_state), frequency, mean_frequency, Tu, Td, Tu_Td, mean_Tu_Td);
}


static bool plotEnabled = false;
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
                commands_plot_add_graph("Td/Tu ratio");
                commands_plot_add_graph("Frequency");	
                commands_plot_add_graph("Direction");
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
    } else {
        commands_printf("This command requires one argument.\n");
    }
}

#define PLOT_UPDATE_TIME 200U
#define PLOT_UPDATE_CYCLES PLOT_UPDATE_TIME/THREAD_SLEEP_TIME

static void updatePlot(void)
{   
    static uint32_t cyclesCounter = 0U;
    cyclesCounter++;
    if(cyclesCounter >= PLOT_UPDATE_CYCLES)
    {
        cyclesCounter = 0U;
        float time = (float)((float)chVTGetSystemTimeX() / 10000.0F);

        commands_plot_set_graph(0);
        commands_send_plot_points(time, mean_Tu_Td);	
        // chThdSleepMilliseconds(1U);
        commands_plot_set_graph(1);
        commands_send_plot_points(time, mean_frequency);	

        commands_plot_set_graph(2);
        commands_send_plot_points(time, (float)mov_current);					
    }
}

static THD_FUNCTION(pas_sensor_thread, arg){
    (void)arg;
    chRegSetThreadName("PAS Sensor");

    for(;;){
        calculateParams();

        if(plotEnabled == true){
            updatePlot();
        }
        chThdSleepMilliseconds(20U);
    }
}

const char * pinLevelToString(uint8_t pinLevel)
{
    const char * str;
    switch(pinLevel){
        case 0U:
            str = "LOW";
            break;
        case 1U:
            str = "HIGH";
            break;
        default:
            str = "NO_STATE";
            break;
    }
    return str;
}

const char * movTypeToString(mov_type mov)
{
    const char * str;
    switch(mov){
        case mov_forward:
            str = "FORWARD";
            break;
        case mov_backward:
            str = "BACKWARD";
            break;
        case mov_unspecified:
            str = "UNSPECIFIED";
            break;
        case mov_no_movement:
            str = "NO MOVEMENT";
            break;
        default:
            str = "Error";
            break;
    }
    return str;
}