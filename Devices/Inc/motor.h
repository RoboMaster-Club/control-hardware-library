#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include <stdint.h>
// MSB-LSB: is_serial for 1 (parellel for 0), torque, position, velocity)
#define VELOCITY_CONTROL (0b0001)
#define POSITION_CONTROL (0b0010)
#define TORQUE_CONTROL (0b0100)
#define POSITION_VELOCITY_SERIES (0b1011)
#define POSITION_VELOCITY_PARALLEL (0b0011)

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
    Motor_Reversal_t motor_reversal;
    uint8_t control_mode;

    // external sensor information like imu or external encoders
    uint8_t use_external_feedback;  // 0 for no, 1 for yes
    int8_t external_feedback_dir;  // 0 for no, 1 for yes
    float *external_angle_feedback_ptr;  // pointer to the external angle feedback
    float *external_velocity_feedback_ptr;  // pointer to the external velocity feedback

    PID_t angle_pid;
    PID_t velocity_pid;
    PID_t torque_pid;
} Motor_Config_t;

#endif // MOTOR_H
