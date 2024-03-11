#include "dm4310.h"


void DM4310_DecodeCAN(uint8_t data[8], DM4310_Info_t *data_frame);
void DM4310_EnableMotor(uint8_t can_bus, uint32_t id);
void DM4310_DisableMotor(uint8_t can_bus, uint32_t id);
void DM4310_CtrlMIT(uint8_t can_bus, uint32_t id,
                    float target_pos, float target_vel,
                    float kp, float kd,
                    float torq);
void DM4310_CtrlPosVel(void);
void DM4310_CtrlVel(void);
/**
 * int float_to_uint(float x, float x_min, float x_max, int bits)
 *
 * convert a float into int by spanning the range of the variable and
 * linearly assign the value
 *
 * Input:
 * float x: the float variable to be converted into int
 * float x_min: the min value of x
 * float x_max: the max value of x
 * int bits: bit number of converted int
 *
 * Output:
 * int: the converted int
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}



void DM4310_DecodeCAN(uint8_t data[8], DM4310_Info_t *data_frame)
{
    data_frame->id = (data[0]) & 0x0F;
    data_frame->state = (data[0]) >> 4;
    data_frame->pos_int = (data[1] << 8) | data[2];
    data_frame->vel_int = (data[3] << 4) | (data[4] >> 4);
    data_frame->torq_int = ((data[4] & 0xF) << 8) | data[5];
    data_frame->pos = uint_to_float(data_frame->pos_int, P_MIN, P_MAX, 16);   // (-12.5,12.5)
    data_frame->vel = uint_to_float(data_frame->vel_int, V_MIN, V_MAX, 12);   // (-45.0,45.0)
    data_frame->torq = uint_to_float(data_frame->torq_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
    data_frame->t_mos = (float)(data[6]);
    data_frame->t_rotor = (float)(data[7]);
}

void DM4310_EnableMotor(uint8_t can_bus, uint32_t id)
{
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    CAN_SendTOQueue(can_bus, id, data);
}

void DM4310_DisableMotor(uint8_t can_bus, uint32_t id)
{
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;
    CAN_SendTOQueue(can_bus, id, data);
}

void DM4310_CtrlMIT(uint8_t can_bus, uint32_t id,
                    float target_pos, float target_vel,
                    float kp, float kd,
                    float torq)
{
    uint16_t pos_temp, vel_temp, kp_temp, kd_temp, torq_temp;
    uint8_t data[8];
    pos_temp = float_to_uint(target_pos, P_MIN, P_MAX, 16);
    vel_temp = float_to_uint(target_vel, V_MIN, V_MAX, 12);
    kp_temp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_temp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    torq_temp = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = (pos_temp >> 8);
    data[1] = pos_temp;
    data[2] = (vel_temp >> 4);
    data[3] = ((vel_temp & 0xF) << 4) | (kp_temp >> 8);
    data[4] = kp_temp;
    data[5] = (kd_temp >> 4);
    data[6] = ((kd_temp & 0xF) << 4) | (torq_temp >> 8);
    data[7] = torq_temp;

    CAN_SendTOQueue(can_bus, id, data);
}
void DM4310_CtrlPosVel()
{
    //TODO:
}

void DM4310_CtrlVel()
{
    // TODO:
}
