#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"

#define DJI_MAX_TICKS (8191.0f)
#define DJI_HALF_MAX_TICKS (4096)

typedef struct dji_motor
{
    /* Motor Config */
    uint8_t can_bus;
    uint8_t can_id;

    /* CAN Frame Info */
    uint16_t current_tick;
	uint16_t last_tick;
	int16_t current_vel;
	int16_t current_torq;
    uint8_t temp;
	
    /* Function Varaibles */
    uint16_t encoder_offset;
    int32_t total_round;
    float absolute_angle_rad;
    float total_angle_rad;
} DJI_Motor_t;

typedef enum{
    UP_UP_DOWN = 1, UP_UP_UP
} GM6020_CAN_ID_e;

typedef enum {
    HEAD = 1, TAIL = 0
} DJI_Send_Type_e;

void DJI_Motor_Init(DJI_Motor_t *motor, uint8_t can_bus, uint8_t can_id, uint16_t offset);
void DJI_Motor_Send(DJI_Send_Type_e send_type, uint8_t can_bus, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void DJI_Motor_Decode(DJI_Motor_t *motor, CAN_Rx_Pack_t *rx_pack);
#endif
