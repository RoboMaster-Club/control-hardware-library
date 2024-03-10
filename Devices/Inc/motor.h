#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include <stdint.h>

#define SPEED_CONTROL (0b001)
#define POSITION_CONTROL (0b010)
#define TORQUE_CONTROL (0b100)

typedef enum Motor_Reversal_e
{
    MOTOR_REVERSAL_NORMAL,
    MOTOR_REVERSAL_REVERSED
} Motor_Reversal_t;

typedef struct
{
    uint8_t can_bus;
    uint16_t tx_id; // ignore this for 
    uint16_t rx_id; // use can_id for general devices
    uint8_t speed_controller_id; // use speed_contrller_id for dji motors
    uint16_t offset;
    uint8_t vel_unit_rpm; // 0 for dps, 1 for rpm
    uint8_t pos_abs_ctrl; // 0 for total angle, 1 for absolute
    Motor_Reversal_t reversal;
    uint8_t control_mode;

    PID_t position_pid;
    PID_t speed_pid;
    PID_t torque_pid;
} Motor_Config_t;

#endif // MOTOR_H