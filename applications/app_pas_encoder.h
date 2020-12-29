#ifndef PAS_ENCODER_H
#define PAS_ENCODER_H


/*
*
*
*/

typedef enum
{
    MOV_TYPE_NO_MOVE_OR_TOO_SLOW = 0U,
    MOV_TYPE_MOV_FORWARD,
    MOV_TYPE_MOV_BACKWARD
} mov_type_enum;

void app_pas_encoder_init(void);
void app_pas_encoder_deinit(void);
mov_type_enum app_pas_encoder_get_current_state(void);

#define PAS_ENCODER_EVENT_FLAG_TURN_BACK       1
extern event_source_t pas_encoder_event_source;

#endif /* PAS_SENSOR_INIT_H */