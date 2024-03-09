#include "dji_motor.h"
#include "bsp_can.h"
#include "user_math.h"

void DJI_Motor_Init(DJI_Motor_t *motor, uint8_t can_bus, uint8_t can_id, uint16_t offset) {
    motor->can_bus = can_bus;
    motor->can_id = can_id;
    motor->encoder_offset = offset;
}

/**
 * Sends current values for up to four DJI motors over a CAN bus.
 *
 * This function packages the current values of up to four motors into a single CAN bus message
 * and sends it to the appropriate queue for transmission. The message includes a type specifier 
 * and is sent to one of two possible CAN buses, based on the provided bus identifier.
 *
 * @param send_type The type of message being sent, specified by the DJI_Send_Type_e enumeration. 
 *                  Send can id 1-4 with @ref DJI_Send_Type_e.HEAD, 5-8 with @ref DJI_Send_Type_e.TAIL.
 *                  GM6020 reverse the 8th motor control fram bytes.
 * @param can_bus   Valid values are 1 or 2, representing the two available buses.
 * @param motor1    The current value for the 1st motor, can id 1 or 5
 * @param motor2    The current value for the 2nd motor, can id 2 or 6
 * @param motor3    The current value for the 3rd motor, can id 3 or 7
 * @param motor4    The current value for the 4th motor, can id 4 or 8 (GM6020 does not have 8)
 *
 */
void DJI_Motor_Send(DJI_Send_Type_e send_type, uint8_t can_bus, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    uint8_t data[8];
    data[0] = motor1 << 8;
    data[1] = motor1;
    data[2] = motor2 << 8;
    data[3] = motor2;
    data[4] = motor3 << 8;
    data[5] = motor3;
    data[6] = motor4 << 8;
    data[7] = motor4;
    CAN_SendTOQueue(can_bus, 0x1ff + send_type, data); 
}


/**
 * Decode CAN frame for DJI motor
 * 
 * encoder range [0, 8191]
 * current range [-16384, 16384]
 * speed unit rmpm
 * temp unit degree celcius
*/
void DJI_Motor_Decode(DJI_Motor_t *motor, uint8_t data[8]) {
    /* CAN Frame Process*/
    motor->last_tick = motor->current_tick;
    motor->current_tick = data[0] << 8 | data[1];
    motor->current_vel = data[2] << 8 | data[3];
    motor->current_torq = data[4] << 8 | data[5];
    motor->temp = data[6];
    
    /* absolute angle */
    motor->absolute_angle_rad = (motor->current_tick + motor->encoder_offset) / DJI_MAX_TICKS * (2 * PI);
    
    /* angle wrap */
    if (motor->current_tick - motor->last_tick > DJI_HALF_MAX_TICKS) {
        motor->total_round--;
    }
    else if (motor->current_tick - motor->last_tick < -4096) {
        motor->total_round++;
    }
    motor->total_angle_rad = (motor->total_round) * 2 * PI + motor->absolute_angle_rad;
}