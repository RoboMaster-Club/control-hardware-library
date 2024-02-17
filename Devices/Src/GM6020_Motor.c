/**
 * @file GM6020_Motor.c
 * @author Leo Liu
 * @brief GM6020 communication
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "GM6020_Motor.h"
#include <stdio.h>
#include "Motor.h"

void GM6020_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor_container);
void GM6020_Send_Data(int16_t Pitch_Output, int16_t Yaw_Output);

// GM6020_Func_t GM6020_Func = GM6020_Func_Init;
#undef GM6020_Func_Init

void GM6020_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor_container)
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
	motor_container->total_angle = motor_container->actual_angle + (GM6020_MECH_ANGLE_MAX * motor_container->turn_count);
	motor_container->info_update_frame++;
}

// TODO send data and CAN refactor
void GM6020_Send_Data(int16_t Pitch_Output, int16_t Yaw_Output)
{
	CAN_Func.CAN_0x2FF_Send_Data(&hcan1, 0, Pitch_Output, Yaw_Output, 0);
}