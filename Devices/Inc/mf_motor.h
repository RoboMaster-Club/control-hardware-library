#ifndef MF_MOTOR_H
#define MF_MOTOR_H

#include "bsp_can.h"

#define SINGLE_MOTOR_CTRL_STD 0x140

typedef struct MF_MOTOR_INFO
{
    uint8_t enabled;
    uint16_t angle;
    int16_t velocity;
    int16_t current;
    int8_t temp;

    uint16_t kp_ang;
    uint16_t ki_ang;
    uint16_t kp_vel;
    uint16_t ki_vel;
    uint16_t kp_torq;
    uint16_t ki_torq;
} MF_MOTOR_INFO_t;

void MF_Motor_Decode(uint8_t data[8], MF_MOTOR_INFO_t *motor_info);
void MF_Motor_GetPIDParam(uint8_t can_bus, uint8_t id);
void MF_Motor_PIDToRam(uint8_t can_bus, uint8_t id,
                              uint8_t kp_ang, uint8_t ki_ang,
                              uint8_t kp_vel, uint8_t ki_vel,
                              uint8_t kp_torq, uint8_t ki_torq);
void MF_Motor_EnableMotor(uint8_t can_bus, uint8_t id);
void MF_Motor_DisableMotor(uint8_t can_bus, uint8_t id);

/**
 * Write the PID parameters to RAM, parameters are deleted after a power cycle
*/
void MF_Motor_PIDToRam(uint8_t can_bus, uint8_t id, 
                                                        uint8_t kp_ang, uint8_t ki_ang,
                                                        uint8_t kp_vel, uint8_t ki_vel,
                                                        uint8_t kp_torq, uint8_t ki_torq);

/**
 * close loop torq control
 * torq: (-2000, 2000) -> current (-32A, 30A)
*/
void MF_Motor_TorqueCtrl(uint8_t can_bus, uint8_t id, int16_t torq);

/**
 * close loop velocity control
 * vel: 0.01 deg/bit
*/
void MF_Motor_VelocityCtrl(uint8_t can_bus, uint8_t id, int32_t vel);

/**
 * close loop position control
 * 0.01 deg/bit
*/
void MF_Motor_PositionCtrl(uint8_t can_bus, uint8_t id, int32_t pos);


#endif
