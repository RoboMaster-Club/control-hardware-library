#ifndef DM4310_H
#define DM4310_H

#include "struct_typedef.h"
#include "bsp_can.h"
#include "robot_param.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

enum CONTROL_MODE
{
    MIT = 0,
    POS_VEL,
    VEL,
    ENABLE_MOTOR,
    DISABLE_MOTOR
};

typedef struct
{


    /* MIT Control Frame */
    struct MIT_ControlFrame
    {
        float target_pos;
        float target_vel;
        float kp;
        float ki;
        float kd;
        float torq;
    } mit_control_frame;

    /* Position & Velocity Control Frame */
    struct POS_VEL_ControlFrame
    {
        float target_pos;
        float target_vel;
    } pos_vel_control_frame;

    struct VEL_ControlFrame
    {
        float target_vel;
    } vel_control_frame;

    uint8_t id;
    uint16_t pos_int;
    uint16_t vel_int;
    uint16_t torq_int;

    uint8_t state;
    float pos;
    float vel;
    float torq;
    uint16_t t_mos;
    uint16_t t_rotor;

} DM4310_Info_t;

extern DM4310_Info_t thigh[4];
extern void DM4310_DecodeCAN(uint8_t data[8], DM4310_Info_t *data_frame);
extern void DM4310_EnableMotor(CAN_HandleTypeDef *hcanx, uint32_t id);
extern void DM4310_DisableMotor(CAN_HandleTypeDef *hcanx, uint32_t id);
extern void DM4310_CtrlMIT(CAN_HandleTypeDef *hcanx, uint32_t id,
                     float target_pos, float target_vel,
                     float kp, float kd,
                     float torq);
extern void DM4310_CtrlPosVel(void);
extern void DM4310_CtrlVel(void);
#endif
