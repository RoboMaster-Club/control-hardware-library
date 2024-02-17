/**
 * @file GM6020_Motor.h
 * @author Leo Liu
 * @brief header file for GM6020
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __GM6020_MOTOR_H
#define __GM6020_MOTOR_H

#include "can.h"
#include "bsp_can.h"
#include <stdint.h>
#include <stdio.h>

#define GM6020_OUTPUT_MAX 30000.0f         // GM6020 motor maximum output current
#define GM6020_MECH_ANGLE_MAX 8192.0f      // GM6020 maximum mechanical angle
#define GM6020_ANGLE_CONVERT 0.0439453125f //(360/GM6020_MECH_ANGLE_MAX)

// #define GM6020_Func_Init    \
//   {                               \
//     &GM6020_Get_Data,             \
//     &GM6020_Send_Data, \
//     &GM6020_Check_Status             \
//   }

void GM6020_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor_container);
void GM6020_Send_Data(int16_t Pitch_Output, int16_t Yaw_Output);

// typedef struct
// {
//   void (*GM6020_Get_Data)(CAN_Export_Data_t RxMessage, Motor_Container_t *motor_container);
//   void (*GM6020_Send_Data)(int16_t Trigger_Current);
//   void (*GM6020_Check_Status)(Motor_Container_t *motor_container);
// } GM6020_Func_t;

// extern GM6020_Func_t GM6020_Func;

#endif
