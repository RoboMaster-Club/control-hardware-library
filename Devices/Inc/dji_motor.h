#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "motor.h"


#define MAX_DJI_MOTORS (16) // realloac will be called to shrink the array
#define MAX_DJI_MOTOR_GROUPS (6) // realloac will be called to shrink the array
#define DJI_TX_ID_PLACEHOLDER (0x00)

#define DJI_MAX_TICKS (8191.0f)
#define DJI_HALF_MAX_TICKS (4096)

#define M3508_REDUCTION_RATIO (187.0f/3591.0f)
#define GM6020_REDUCTION_RATIO (1)
#define M2006_REDUCTION_RATIO (1.0f/36.0f)

typedef enum DJI_Motor_Type {
    GM6020,
    M3508,
    M2006,
} DJI_Motor_Type_t;

typedef struct DJI_Motor_Stats_s {
    /* CAN Frame Info */
    uint16_t current_tick;
	uint16_t last_tick;
	int16_t current_vel_rpm;
	int16_t current_torq;
    uint8_t temp;
	
    /* Function Varaibles */
    uint16_t encoder_offset;
    int32_t total_round;
    float current_vel_dps;
    float prev_vel_dps;
    float absolute_angle_rad;
    float total_angle_rad;
    float reduction_ratio;
} DJI_Motor_Stats_t;


typedef struct dji_motor
{
    DJI_Motor_Type_t motor_type;
    uint8_t can_bus;
    uint8_t speed_controller_id;
    
    /* Motor Config */
    uint8_t control_type;
    Motor_Reversal_t is_reversed;
    uint8_t vel_unit_rpm;
    uint8_t pos_abs_ctrl;
    DJI_Motor_Stats_t *stats;

    /* Motor Controller */
    PID_t *position_pid;
    PID_t *speed_pid;
    PID_t *torque_pid;

    int16_t output_current;

    void (*set_current)(struct dji_motor *motor, float current);
    void (*set_speed)(struct dji_motor *motor, float speed);
    void (*set_position)(struct dji_motor *motor, float position);
} DJI_Motor_Handle_t;

typedef enum{
    UP_UP_DOWN = 1, UP_UP_UP
} GM6020_CAN_ID_e;

typedef enum {
    HEAD = 1, TAIL = 0
} DJI_Send_Type_e;

typedef struct _DJI_Send_Group_s {
    uint8_t register_device_indicator;
    CAN_Instance_t *can_instance;
    int16_t *motor_torq[4];
} DJI_Send_Group_t;

DJI_Motor_Handle_t *DJI_Motor_Init(Motor_Config_t *config, DJI_Motor_Type_t type);
void DJI_Motor_Send(void);
#endif
