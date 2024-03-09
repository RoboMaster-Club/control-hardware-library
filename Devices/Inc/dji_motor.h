#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>
#include "motor.h"


#define MAX_DJI_MOTORS (16) // realloac will be called to shrink the array

#define DJI_MAX_TICKS (8191.0f)
#define DJI_HALF_MAX_TICKS (4096)

typedef struct DJI_Motor_Stats_s {
    /* CAN Frame Info */
    uint16_t current_tick;
	uint16_t last_tick;
	int16_t current_vel;
	int16_t current_torq;
    uint8_t temp;
	
    /* Function Varaibles */
    uint16_t encoder_offset;
    int32_t total_round;
    float absolute_angle_rad;
    float total_angle_rad;
} DJI_Motor_Stats_t;


typedef struct dji_motor
{
    /* Motor Config */
    Motor_Control_t control_type;
    Motor_Reversal_t is_reversed;
    DJI_Motor_Stats_t *stats;

    /* Motor Controller */
    PID_t *position_pid;
    PID_t *speed_pid;
    PID_t *torque_pid;

} DJI_Motor_Handle_t;

typedef enum{
    UP_UP_DOWN = 1, UP_UP_UP
} GM6020_CAN_ID_e;

typedef enum {
    HEAD = 1, TAIL = 0
} DJI_Send_Type_e;


DJI_Motor_Handle_t *DJI_Motor_Init(Motor_Config_t *config);
#endif
