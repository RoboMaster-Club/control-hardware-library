/**
 * @file M3508_Motor.c
 * @author Leo Liu
 * @brief M3508 communication
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "M3508_Motor.h"
#include <stdio.h>
#include "Motor.h"

void M3508_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor);
void M3508_Send_Data(int16_t Motor_1_Current, int16_t Motor_2_Current, int16_t Motor_3_Current, int16_t Motor_4_Current);

// M3508_Func_t M3508_Func = M3508_Func_Init;
// #undef M3508_Func_Init

void M3508_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor_container)
{
	motor_container->prev_angle = motor_container->actual_angle;
	motor_container->actual_angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
	motor_container->actual_speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
	motor_container->actual_current = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
	motor_container->temperature = RxMessage.CANx_Export_RxMessage[6];
	if ((motor_container->actual_angle - motor_container->prev_angle) < -6500)
		motor_container->turn_count++;
	else if ((motor_container->actual_angle - motor_container->prev_angle) > 6500)
		motor_container->turn_count--;
	motor_container->total_angle = motor_container->actual_angle + (M3508_MECH_ANGLE_MAX * motor_container->turn_count);
	motor_container->info_update_frame++;
}

// Send chassis data through specified identifier
void M3508_Send_Data(int16_t Motor_1_Current, int16_t Motor_2_Current, int16_t Motor_3_Current, int16_t Motor_4_Current)
{
	CAN_Func.CAN_0x200_Send_Data(&hcan1, Motor_1_Current, Motor_2_Current, Motor_3_Current, Motor_4_Current);
}