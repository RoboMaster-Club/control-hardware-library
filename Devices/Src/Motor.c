#include "Motor.h"
#include "GM6020_Motor.h"
#include "M2006_Motor.h"
#include "M3508_Motor.h"

//
Motor_Container_t *new_motor(int can_ID, Motor_Type_e motor_type)
{
    Motor_Container_t *motor = (Motor_Container_t *)malloc(sizeof(Motor_Container_t));
    motor->can_ID = can_ID;
    motor->motor_type = motor_type;
    return motor;
}

void Motor_Get_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor)
{
    switch (motor->motor_type)
    {
    case GM6020:
        GM6020_Get_Data(RxMessage, motor);
        break;
    case M2006:
        M2006_Get_Data(RxMessage, motor);
        break;
    case M3508:
        M3508_Get_Data(RxMessage, motor);
        break;
    default:
        // you did something very wrong
        break;
    }
}

int Motor_Check_Status(Motor_Container_t *motor_container)
{
	if (motor_container->info_update_frame < 1)
		motor_container->offline_flag = 1;
	else
		motor_container->offline_flag = 0;
	motor_container->info_update_frame = 0;
}

//TODO Send data
// void Send_Data(CAN_Export_Data_t RxMessage, Motor_Container_t *motor)
// {
//     switch (motor->motor_type)
//     {
//     case GM6020:
//         GM6020_Send_Data(motor);
//         break;
//     case M2006:
//         M2006_Send_Data(Trigger_Current);
//         break;
//     case M3508:
//         break;
//     default:
//         // you did something very wrong
//         break;
//     }
// }

//