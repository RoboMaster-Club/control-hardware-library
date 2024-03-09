#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include <stdint.h>

typedef enum Motor_Reversal_e
{
    MOTOR_REVERSAL_NORMAL,
    MOTOR_REVERSAL_REVERSED
} Motor_Reversal_t;

typedef enum Motor_Control_e
{
    SPEED_CONTROL = 0b001,
    POSITION_CONTROL = 0b010,
    TORQUE_CONTROL = 0b100,
    POSITION_SPEED_CONTROL = 0b011,
    MIT_CONTROL = 0b111,
} Motor_Control_t;

typedef struct Motor_Config_s
{
    uint8_t can_bus;
    uint8_t can_id;
    uint16_t offset;

    Motor_Reversal_t reversal;
    Motor_Control_t control_mode;

    PID_t position_pid;
    PID_t speed_pid;
    PID_t torque_pid;
} Motor_Config_t;

#endif // MOTOR_H