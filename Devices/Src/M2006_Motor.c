/**
 * @file M2006_Motor.c
 * @author Leo Liu
 * @brief M2006 communication
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "M2006_Motor.h"
#include <stdio.h>
#include "Motor.h"

void M2006_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor_container);
void M2006_Send_Data(int16_t Trigger_Current);

// M2006_Func_t M2006_Func = M2006_Func_Init;
// #undef M2006_Func_Init

// Obatin trigger motor data from CAN
void M2006_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor_container)
{
	motor_container->prev_angle = motor_container->actual_angle;
	motor_container->actual_angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
	motor_container->actual_speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
	if ((motor_container->actual_angle - motor_container->prev_angle) < -6500)
		motor_container->turn_count++;
	else if ((motor_container->actual_angle - motor_container->prev_angle) > 6500)
		motor_container->turn_count--;
	motor_container->total_angle = motor_container->actual_angle + (M2006_MECH_ANGLE_MAX * motor_container->turn_count);
	motor_container->info_update_frame++;
}

// Send trigger data through specified identifier
void M2006_Send_Data(int16_t Trigger_Current)
{
	CAN_Func.CAN_0x1FF_Send_Data(&hcan2, 0, Trigger_Current, 0, 0);
}
