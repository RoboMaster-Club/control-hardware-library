/**
 * @file Motor.h
 * @author Amanjyoti Mridha and Irving Wang
 * @brief General Motor Control File
 * @version 1.0
 * @date February 10, 2024
 * 
 * @copyright Copyright (c) 2024 
*/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "bsp_can.h"

/**
enum to control motor type
*/
typedef enum Motor_Type_e {
    GM6020,
    M2006,
    M3508
} Motor_Type_e;

/**
general struct containing motor type, id, and motor data
*/
typedef struct Motor_Container_t
{
    /* data */
    int can_ID;
    Motor_Type_e motor_type;

    // Motor Data
    int16_t actual_angle;
    int16_t prev_angle;
    int16_t actual_speed;
    int16_t actual_current;
    int8_t temperature;
    
    int32_t target_angle;
    int16_t target_speed;
    int32_t total_angle;
    int16_t turn_count;
    int16_t output_current;
    
    uint16_t info_update_frame; //TODO learn what this is
    uint8_t offline_flag;
} Motor_Container_t;

Motor_Container_t* new_motor(int can_ID, Motor_Type_e motor_type);
void Motor_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor);
int Motor_Check_Status(Motor_Container_t *motor_container);

#endif