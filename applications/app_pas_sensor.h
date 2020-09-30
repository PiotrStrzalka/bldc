#ifndef PAS_SENSOR_INIT_H
#define PAS_SENSOR_INIT_H

// PAS sensor pin and interrupt definitions
#define HW_PAS_SENSOR_PORT	        GPIOA
#define HW_PAS_SENSOR_PIN 	        6   
#define HW_PAS_SENSOR_PORT_INT	    EXTI_PortSourceGPIOA
#define HW_PAS_SENSOR_PIN_INT 	    EXTI_PinSource6
#define HW_PAS_SENSOR_EXTI_CH       EXTI9_5_IRQn
#define HW_PAS_SENSOR_EXTI_LINE     EXTI_Line6
#define HW_PAS_SENSOR_EXTI_ISR_VEC  EXTI9_5_IRQHandler

// Project.AddPathSubstitute /mnt/c/PROJEKTY/VESC C:\PROJEKTY\VESC
typedef enum {
    mov_no_movement = 0U,
    mov_forward,
    mov_backward,
    mov_unspecified    
} mov_type;

mov_type PAS_GetCurrentState(void);

/*
*
*
*/
void app_pas_sensor_init(void);

#endif /* PAS_SENSOR_INIT_H */