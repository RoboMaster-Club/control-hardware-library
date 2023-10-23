#include "mf_motor.h"

void MF_Motor_Decode(uint8_t data[8], MF_MOTOR_INFO_t *motor_info);
void MF_Motor_GetPIDParam(CAN_HandleTypeDef *hcanx, uint8_t id);
void MF_Motor_PIDToRam(CAN_HandleTypeDef *hcanx, uint8_t id,
                              uint8_t kp_ang, uint8_t ki_ang,
                              uint8_t kp_vel, uint8_t ki_vel,
                              uint8_t kp_torq, uint8_t ki_torq);
void MF_Motor_EnableMotor(CAN_HandleTypeDef *hcanx, uint8_t id);
void MF_Motor_DisableMotor(CAN_HandleTypeDef *hcanx, uint8_t id);
void MF_Motor_TorqueCtrl(CAN_HandleTypeDef *hcanx, uint8_t id, int16_t torq);
void MF_Motor_VelocityCtrl(CAN_HandleTypeDef *hcanx, uint8_t id, int32_t vel);
void MF_Motor_PositionCtrl(CAN_HandleTypeDef *hcanx, uint8_t id, int32_t pos);

void MF_Motor_Decode(uint8_t data[8], MF_MOTOR_INFO_t *motor_info)
{
    switch (data[0])
    {
    case 0x30:
        motor_info->kp_ang = data[2];
        motor_info->ki_ang = data[3];
        motor_info->kp_vel = data[4];
        motor_info->ki_vel = data[5];
        motor_info->kp_torq = data[6];
        motor_info->ki_torq = data[7];
        break;
    case 0x80:
        motor_info->enabled = 0;
        break;

    case 0x88:
        motor_info->enabled = 1;
        break;

    case 0xA1:
    case 0xA2:
        motor_info->angle = (data[7] << 8) + data[6];
        motor_info->velocity = (data[5] << 8) + data[4];
        motor_info->current = (data[3] << 8) + data[2];
        motor_info->temp = data[1];
        break;
    }
}

void MF_Motor_EnableMotor(CAN_HandleTypeDef *hcanx, uint8_t id)
{
    uint8_t data[8];
    data[0] = 0x88;
    CAN_BSP_SendTOQueue(hcanx, SINGLE_MOTOR_CTRL_STD + id, data);
}

void MF_Motor_DisableMotor(CAN_HandleTypeDef *hcanx, uint8_t id)
{
    uint8_t data[8];
    data[0] = 0x80;
    CAN_BSP_SendTOQueue(hcanx, SINGLE_MOTOR_CTRL_STD + id, data);
}

void MF_Motor_GetPIDParam(CAN_HandleTypeDef *hcanx, uint8_t id)
{
    uint8_t data[8];
    data[0] = 0x30;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    CAN_BSP_SendTOQueue(hcanx, SINGLE_MOTOR_CTRL_STD + id, data);
}

void MF_Motor_PIDToRam(CAN_HandleTypeDef *hcanx, uint8_t id,
                              uint8_t kp_ang, uint8_t ki_ang,
                              uint8_t kp_vel, uint8_t ki_vel,
                              uint8_t kp_torq, uint8_t ki_torq)
{
    uint8_t data[8];
    data[0] = 0x31;
    data[1] = 0x00;
    data[2] = kp_ang;
    data[3] = ki_ang;
    data[4] = kp_vel;
    data[5] = ki_vel;
    data[6] = kp_torq;
    data[7] = ki_torq;
    CAN_BSP_SendTOQueue(hcanx, SINGLE_MOTOR_CTRL_STD + id, data);
}

void MF_Motor_TorqueCtrl(CAN_HandleTypeDef *hcanx, uint8_t id, int16_t torq)
{
    uint8_t data[8];
    data[0] = 0xA1;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = torq & 0xFF;
    data[5] = (torq >> 8) & 0xFF;
    data[6] = 0x00;
    data[7] = 0x00;
    CAN_BSP_SendTOQueue(hcanx, SINGLE_MOTOR_CTRL_STD + id, data);
}

void MF_Motor_VelocityCtrl(CAN_HandleTypeDef *hcanx, uint8_t id, int32_t vel)
{
    uint8_t data[8];
    data[0] = 0xA2;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = vel & 0xFF;
    data[5] = (vel >> 8) & 0xFF;
    data[6] = (vel >> 16) & 0xFF;
    data[7] = (vel >> 24) & 0xFF;
    CAN_BSP_SendTOQueue(hcanx, SINGLE_MOTOR_CTRL_STD + id, data);
}

void MF_Motor_PositionCtrl(CAN_HandleTypeDef *hcanx, uint8_t id, int32_t pos)
{
    uint8_t data[8];
    data[0] = 0xA3;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = pos & 0xFF;
    data[5] = (pos >> 8) & 0xFF;
    data[6] = (pos >> 16) & 0xFF;
    data[7] = (pos >> 24) & 0xFF;
    CAN_BSP_SendTOQueue(hcanx, SINGLE_MOTOR_CTRL_STD + id, data);
}
