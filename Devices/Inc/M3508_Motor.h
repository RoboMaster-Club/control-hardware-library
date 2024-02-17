/**
 * @file M3508_Motor.h
 * @author Leo Liu
 * @brief header file for M3508
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include "can.h"
#include "bsp_can.h"
#include <stdint.h>
#include <stdio.h>

#define M3508_MECH_ANGLE_MAX 8192.0f
#define M3508_SPEED_MAX 9000.0f	  // M3508 maximum speed
#define M3508_OUTPUT_MAX 16384.0f // M3508 maximum output current

// #define M3508_Func_Init       \
// 	{                         \
// 		&M3508_Get_Data,      \
// 			&M3508_Send_Data, \
// 	}

// typedef struct
// {
// 	void (*M3508_Get_Data)(CAN_Export_Data_t RxMessage);
// 	void (*M3508_Send_Data)(int16_t Motor_1_Current, int16_t Motor_2_Current, int16_t Motor_3_Current, int16_t Motor_4_Current);
// } M3508_Func_t;

// extern M3508_Func_t M3508_Func;
// extern Motor_Init_t M3508_Chassis[4];
// extern Motor_Init_t M3508_Fric_Wheel[2];

void M3508_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor);
void M3508_Send_Data(int16_t Motor_1_Current, int16_t Motor_2_Current, int16_t Motor_3_Current, int16_t Motor_4_Current);

#endif
