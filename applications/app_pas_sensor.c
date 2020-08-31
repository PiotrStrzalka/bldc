#include "ch.h"
#include "hal.h"
#include "app_pas_sensor.h"
#include "stm32f4xx_conf.h"
#include "isr_vector_table.h"
#include "commands.h"

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
static thread_t * isr_pas;

const char * pinLevelToString(uint8_t pinLevel);
const char * movTypeToString(mov_type mov);

static volatile mov_type PAS_mov_current = mov_no_movement;

mov_type PAS_GetCurrentState(void)
{
    return PAS_mov_current;
}

CH_IRQ_HANDLER(HW_PAS_SENSOR_EXTI_ISR_VEC){
    if(EXTI_GetITStatus(HW_PAS_SENSOR_EXTI_LINE) != RESET){
        void *pbuf;
        uint8_t pinLevel = palReadPad(HW_PAS_SENSOR_PORT, HW_PAS_SENSOR_PIN);        

        chSysLockFromISR();
        if(chMBFetchI(&pas_events_free, (msg_t *) &pbuf) == MSG_OK){
            ((pas_edge *) pbuf)->pin_state = pinLevel;
            ((pas_edge *) pbuf)->timestamp = (uint32_t) chVTGetSystemTimeX();
            (void)chMBPostI(&pas_events, (msg_t) pbuf);
        }
        chSysUnlockFromISR();
        EXTI_ClearITPendingBit(HW_PAS_SENSOR_EXTI_LINE);
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
}

void pas_sensor_deinit(void)
{
    //todo strzalkap - disable interrupt here
}

mov_type CalculateParams(pas_edge * event)
{    
    static uint32_t previousTime = 0U;
    static uint32_t falling_slope_time = 0U;
    static uint32_t rising_slope_time = 0U;
    static uint32_t Tu = 1U;
    static uint32_t Td = 1U;
    static float frequency = 0.0f;
    static pas_edge previousEvent;
    
    if((event->pin_state == 0U) && (previousEvent.pin_state == 1U))
    { // falling slope 
        frequency =  (float)CH_CFG_ST_FREQUENCY/(float)(event->timestamp - falling_slope_time);
        falling_slope_time = event->timestamp;            
        Tu = falling_slope_time - rising_slope_time;
    }
    else if((event->pin_state == 1U) && (previousEvent.pin_state == 0U))
    { // raising slope  
        frequency = (float)CH_CFG_ST_FREQUENCY/(float)(event->timestamp - rising_slope_time);
        rising_slope_time = event->timestamp;
        Td = rising_slope_time - falling_slope_time;
    }
    else
    {
        commands_printf("Same events! time: %lums, state: %u", event->timestamp/10U, event->pin_state);
    }

    static float mean_Tu_Td = 0.0f;
    static float mean_frequency = 0.0f;
    
    mean_frequency = (mean_frequency * 2.0f + frequency) / 3.0f;
    float Tu_Td = (float)Tu*100.0f/(float)(Tu+Td);
    mean_Tu_Td = (mean_Tu_Td * 2.0f + Tu_Td)/3.0f;

    mov_type result = mov_unspecified;
    //static const char * result = "Not sure";
    if((mean_frequency > 0.8f)){
        if(mean_Tu_Td > 50.0f){
            result = mov_forward;
        } else {
            result = mov_backward;
        }            
    }

    //commands_printf("%s, t:%.1fms d: %.1fms, pin %s, freq: %.2fHz | %.2fHz, Tu: %.1f, Td: %.1f, Tu/Td: %.1f | Tu/Td %.1f" ,
      //  movTypeToString(result), (float)event->timestamp/10.0f, (float)(event->timestamp - previousEvent.timestamp)/10.0f,
        //pinLevelToString(event->pin_state), frequency, mean_frequency, Tu, Td, Tu_Td, mean_Tu_Td);

    previousEvent = *event;
    return result;
}

static THD_FUNCTION(pas_sensor_thread, arg){
    (void)arg;
    chRegSetThreadName("PAS Sensor");
    static uint32_t last_event = 0U;
    static bool idle_detected = false;
    static mov_type PAS_mov_previous = mov_no_movement;

    for(;;){
        msg_t ret;        

        do{
            void *pbuf;
            ret = chMBFetchI(&pas_events, (msg_t *) &pbuf);
            if(ret == MSG_OK){
                //commands_printf("Timestamp: %lu, state: %lu", ((pas_edge *)pbuf)->timestamp, ((pas_edge *)pbuf)->pin_state);
                last_event = ((pas_edge *)pbuf)->timestamp;
                idle_detected = false;
                // Calculations here!
                PAS_mov_current =  CalculateParams(((pas_edge *)pbuf));
                //Return buffer to pool
                (void) chMBPostI(&pas_events_free, (msg_t) pbuf);
            }
        } while(ret == MSG_OK);

        if(PAS_mov_current != PAS_mov_previous)
        {
            commands_printf("t:%.1fms Change %s -> %s", (float)chVTGetSystemTime()/10.0f,
                movTypeToString(PAS_mov_previous), movTypeToString(PAS_mov_current));
            PAS_mov_previous = PAS_mov_current;
        }

        if((chVTGetSystemTime() > (last_event + 5000U)) && (idle_detected == false)){
            //commands_printf("No action from two seconds!");
            idle_detected = true;
            PAS_mov_current = mov_no_movement;
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