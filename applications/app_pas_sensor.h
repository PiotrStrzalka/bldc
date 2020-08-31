#ifndef PAS_SENSOR_INIT_H
#define PAS_SENSOR_INIT_H


#define HW_PAS_SENSOR_PORT	        GPIOB
//#define HW_PAS_SENSOR_PIN 	        6
#define HW_PAS_SENSOR_PIN 	        6 //HAL 1

#define HW_PAS_SENSOR_PORT_INT	    EXTI_PortSourceGPIOB
#define HW_PAS_SENSOR_PIN_INT 	    EXTI_PinSource6
#define HW_PAS_SENSOR_EXTI_CH       EXTI9_5_IRQn
#define HW_PAS_SENSOR_EXTI_LINE     EXTI_Line6
#define HW_PAS_SENSOR_EXTI_ISR_VEC  EXTI9_5_IRQHandler

typedef enum {
    mov_forward = 0U,
    mov_backward,
    mov_unspecified,
    mov_no_movement
} mov_type;

mov_type PAS_GetCurrentState(void);

/*
*
*
*/
void app_pas_sensor_init(void);

#endif /* PAS_SENSOR_INIT_H */