#include "dji_motor.h"

#include <stdlib.h>
#include "bsp_can.h"
#include "user_math.h"
#include "motor.h"

DJI_Motor_Handle_t *g_dji_motors[MAX_DJI_MOTORS] = {NULL};
uint8_t g_dji_motor_count = 0;

DJI_Send_Group_t *g_dji_send_group[MAX_DJI_MOTOR_GROUPS] = {NULL};
uint8_t g_dji_motor_group_count = 0;

void DJI_Motor_Decode(CAN_Instance_t *can_instance);
void DJI_Set_Position(DJI_Motor_Handle_t *motor_handle, float pos);
void DJI_Set_Speed(DJI_Motor_Handle_t *motor_handle, float speed);
void DJI_Set_Torque(DJI_Motor_Handle_t *motor_handle, float torque);
void DJI_Motor_Disable(DJI_Motor_Handle_t *motor_handle);

uint8_t DJI_Motor_Assign_To_Group(DJI_Motor_Handle_t *motor_handle, uint16_t tx_id)
{
    uint8_t can_bus = motor_handle->can_bus;

    // if (g_dji_send_group == NULL)
    // {
    //     g_dji_send_group = (DJI_Send_Group_t**) calloc(MAX_DJI_MOTOR_GROUPS, sizeof(DJI_Send_Group_t));
    //     if (g_dji_send_group == NULL)
    //     {
    //         // Log Memory allocation failed
    //     }
    // }
    for (int i = 0; i < g_dji_motor_group_count; i++)
    {
        if ((g_dji_send_group[i]->can_instance->can_bus == can_bus) && (g_dji_send_group[i]->can_instance->tx_header->StdId == tx_id))
        {
            g_dji_send_group[i]->register_device_indicator |= (1 << (((motor_handle->speed_controller_id) % 4) - 1));
            g_dji_send_group[i]->motor_torq[motor_handle->speed_controller_id % 4 - 1] = &motor_handle->output_current;
            return 0;
        }
    }
    // if reach here, create a new group
    DJI_Send_Group_t *new_group = (DJI_Send_Group_t *)malloc(sizeof(DJI_Send_Group_t));
    new_group->can_instance = calloc(sizeof(CAN_Instance_t), 1);
    new_group->can_instance->can_bus = can_bus;
    new_group->can_instance->tx_header = malloc(sizeof(CAN_TxHeaderTypeDef));
    new_group->can_instance->tx_header->StdId = tx_id;
    new_group->can_instance->tx_header->IDE = CAN_ID_STD;
    new_group->can_instance->tx_header->RTR = CAN_RTR_DATA;
    new_group->can_instance->tx_header->DLC = 8;
    new_group->register_device_indicator = (1 << (motor_handle->speed_controller_id % 4 - 1));
    new_group->motor_torq[motor_handle->speed_controller_id % 4 - 1] = &motor_handle->output_current;
    motor_handle->output_current = 0;
    // assign new send group to the global array
    g_dji_send_group[g_dji_motor_group_count++] = new_group;
    return 1;
}

DJI_Motor_Handle_t *DJI_Motor_Init(Motor_Config_t *config, DJI_Motor_Type_t type)
{
    DJI_Motor_Handle_t *motor_handle = malloc(sizeof(DJI_Motor_Handle_t));
    motor_handle->motor_type = type;
    motor_handle->can_bus = config->can_bus;
    motor_handle->speed_controller_id = config->speed_controller_id;
    motor_handle->vel_unit_rpm = config->vel_unit_rpm;
    motor_handle->pos_abs_ctrl = config->pos_abs_ctrl;
    motor_handle->control_type = config->control_mode;
    motor_handle->is_reversed = config->reversal;

    motor_handle->set_current = DJI_Set_Torque;
    motor_handle->set_speed = DJI_Set_Speed;
    motor_handle->set_position = DJI_Set_Position;
    motor_handle->disable = DJI_Motor_Disable;

    DJI_Motor_Stats_t *motor_stats = malloc(sizeof(DJI_Motor_Stats_t));
    motor_stats->encoder_offset = config->offset;
    motor_stats->total_round = 0;
    motor_stats->current_tick = 0;
    motor_stats->current_vel_rpm = 0;
    motor_stats->current_vel_dps = 0;
    motor_stats->prev_vel_dps = 0;
    motor_stats->current_torq = 0;
    motor_stats->temp = 0;
    motor_handle->stats = motor_stats;
    motor_handle->output_current = 0;

    switch (config->control_mode)
    {
    case SPEED_CONTROL:
        motor_handle->speed_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->speed_pid, &config->speed_pid, sizeof(PID_t));
        break;
    case POSITION_CONTROL:
        motor_handle->position_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->position_pid, &config->position_pid, sizeof(PID_t));
        break;
    case SPEED_CONTROL | POSITION_CONTROL:
        motor_handle->speed_pid = malloc(sizeof(PID_t));
        motor_handle->position_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->speed_pid, &config->speed_pid, sizeof(PID_t));
        memcpy(motor_handle->position_pid, &config->position_pid, sizeof(PID_t));
        break;
    case TORQUE_CONTROL:
        motor_handle->torque_pid = malloc(sizeof(PID_t));
        memmove(motor_handle->torque_pid, &config->torque_pid, sizeof(PID_t));
        break;
    case SPEED_CONTROL | POSITION_CONTROL | TORQUE_CONTROL:
        motor_handle->speed_pid = malloc(sizeof(PID_t));
        motor_handle->position_pid = malloc(sizeof(PID_t));
        motor_handle->torque_pid = malloc(sizeof(PID_t));
        memmove(motor_handle->speed_pid, &config->speed_pid, sizeof(PID_t));
        memmove(motor_handle->position_pid, &config->position_pid, sizeof(PID_t));
        memmove(motor_handle->torque_pid, &config->torque_pid, sizeof(PID_t));
        break;
    default:
        // TODO: LOG ERROR
        break;
    }

    

    // // allocate memory for the g_dji_motors
    // if (g_dji_motors == NULL) {
    //     g_dji_motors = calloc(MAX_DJI_MOTORS, sizeof(DJI_Motor_Handle_t*));
    //     if (g_dji_motors == NULL) {
    //         //Log Memory allocation failed
    //     }
    // }
    
    CAN_Instance_t *receiver_can_instance = NULL;
    // Send Group assignment
    switch (motor_handle->motor_type)
    {
    case GM6020:
        receiver_can_instance = CAN_Device_Register(config->can_bus, config->tx_id,
                                                                0x204 + config->speed_controller_id, DJI_Motor_Decode);
        receiver_can_instance->binding_motor_stats = motor_stats;
        motor_stats->reduction_ratio = GM6020_REDUCTION_RATIO;
        switch (motor_handle->speed_controller_id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            DJI_Motor_Assign_To_Group(motor_handle, 0x1FF);
            break;
        case 5:
        case 6:
        case 7:
            DJI_Motor_Assign_To_Group(motor_handle, 0x2FF);
            break;
        default:
            // TODO: LOG ERROR
            break;
        }
        break;
    case M3508:
        receiver_can_instance = CAN_Device_Register(config->can_bus, config->tx_id,
                                                                0x200 + config->speed_controller_id, DJI_Motor_Decode);
        receiver_can_instance->binding_motor_stats = motor_stats;
        motor_stats->reduction_ratio = M3508_REDUCTION_RATIO;
        switch (motor_handle->speed_controller_id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            DJI_Motor_Assign_To_Group(motor_handle, 0x200);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
            DJI_Motor_Assign_To_Group(motor_handle, 0x1FF);
            break;
        default:
            break;
        }
        break;
    case M2006:
        receiver_can_instance = CAN_Device_Register(config->can_bus, config->tx_id,
                                                                0x200 + config->speed_controller_id, DJI_Motor_Decode);
        receiver_can_instance->binding_motor_stats = motor_stats;
        motor_stats->reduction_ratio = M2006_REDUCTION_RATIO;
        switch (motor_handle->speed_controller_id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            DJI_Motor_Assign_To_Group(motor_handle, 0x200);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
            DJI_Motor_Assign_To_Group(motor_handle, 0x1FF);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    g_dji_motors[g_dji_motor_count++] = motor_handle;
    return motor_handle;
}

// void DJI_Motor_Wrap_Up()
// {
//     DJI_Motor_Handle_t **dji_motor_shrink_attempt = realloc(g_dji_motors,
//                             g_dji_motor_count * sizeof(DJI_Motor_Handle_t*));
//     DJI_Send_Group_t **dji_send_group_shrink_attempt = realloc(g_dji_send_group,
//                             g_dji_motor_group_count * sizeof(DJI_Send_Group_t*));
//     if (dji_motor_shrink_attempt != NULL)
//     {
//         // if shrink failed, use original g_dji_motors
//         g_dji_motors = dji_motor_shrink_attempt;
//     }
//     if (dji_send_group_shrink_attempt != NULL)
//     {
//         // if shrink failed, use original g_dji_send_group
//         g_dji_send_group = dji_send_group_shrink_attempt;
//     }
// }

void DJI_Set_Position(DJI_Motor_Handle_t *motor_handle, float pos)
{
    motor_handle->disabled = 0;
    motor_handle->position_pid->ref = pos;
}

void DJI_Set_Speed(DJI_Motor_Handle_t *motor_handle, float speed)
{
    motor_handle->disabled = 0;
    motor_handle->speed_pid->ref = speed;
}

void DJI_Set_Torque(DJI_Motor_Handle_t *motor_handle, float torque)
{
    motor_handle->disabled = 0;
    motor_handle->torque_pid->ref = torque;
}

void DJI_Motor_Disable(DJI_Motor_Handle_t *motor_handle)
{
    motor_handle->disabled = 1;
}

void DJI_Motor_Current_Calc()
{
    for (int i = 0; i < g_dji_motor_count; i++)
    {
        DJI_Motor_Handle_t *motor = g_dji_motors[i];
        switch (motor->disabled)
        {
        case 1:
            motor->output_current = 0;
            break;
        case 0:
            switch (motor->control_type)
            {
            case SPEED_CONTROL:
                switch (motor->vel_unit_rpm)
                {
                case 1:
                    motor->output_current = PID(motor->speed_pid, motor->speed_pid->ref - (float)motor->stats->current_vel_rpm);
                    break;
                case 0:
                    motor->output_current = PID(motor->speed_pid, motor->speed_pid->ref - motor->stats->current_vel_dps);
                    break;
                default:
                    // Log Error
                    break;
                }
                break;
            case POSITION_CONTROL:
                float error = motor->position_pid->ref - motor->stats->absolute_angle_rad;
                if (error >= PI) {
                    error -= 2 * PI;
                } else if (error < -PI) {
                    error += 2 * PI;
                }
                motor->output_current = PID(motor->position_pid, error);
                break;
            case TORQUE_CONTROL:
                break;
            case SPEED_CONTROL | POSITION_CONTROL:
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
    }
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
void DJI_Motor_Send()
{
    DJI_Motor_Current_Calc();
    for (int i = 0; i < g_dji_motor_group_count; i++)
    {
        DJI_Send_Group_t *group = g_dji_send_group[i];
        uint8_t register_indicator = group->register_device_indicator;
        uint8_t *data = group->can_instance->tx_buffer;
        if (register_indicator & 0b0001)
        {
            static int16_t motor1 = 0;
            motor1 = (*(group->motor_torq[0]));
            data[0] = motor1 >> 8;
            data[1] = motor1;
        }
        if (register_indicator & 0b0010)
        {
            static int16_t motor2 = 0;
            motor2 = (*(group->motor_torq[1]));
            data[2] = motor2 >> 8;
            data[3] = motor2;
        }
        if (register_indicator & 0b0100)
        {
            static int16_t motor3 = 0;
            motor3 = (*(group->motor_torq[2]));
            data[4] = motor3 >> 8;
            data[5] = motor3;
        }
        if (register_indicator & 0b1000)
        {
            static int16_t motor4 = 0;
            motor4 = (*(group->motor_torq[3]));
            data[6] = motor4 >> 8;
            data[7] = motor4;
        }
        if (CAN_Transmit(group->can_instance) != HAL_OK)
        {
            // Log Error
        }
    }
}

/**
 * Decode CAN frame for DJI motor
 *
 * encoder range [0, 8191]
 * current range [-16384, 16384]
 * speed unit rmpm
 * temp unit degree celcius
 */
void DJI_Motor_Decode(CAN_Instance_t *can_instance)
{
    DJI_Motor_Stats_t *motor = (DJI_Motor_Stats_t *)can_instance->binding_motor_stats; // binding in @ref DJI_Motor_Init
    uint8_t *data = can_instance->rx_buffer;

    /* CAN Frame Process*/
    motor->last_tick = motor->current_tick;
    motor->current_tick = (data[0] << 8 | data[1]);
    motor->current_vel_rpm = (int16_t) ((data[2] << 8 | data[3])) * motor->reduction_ratio;
    motor->current_torq = (int16_t)( data[4] << 8 | data[5]);
    motor->temp = data[6];

    motor->absolute_angle_rad = motor->current_tick - motor->encoder_offset;
    /* absolute angle */
    __MAP(motor->absolute_angle_rad, 0, 8192, -PI, PI);

    /* angle wrap */
    if (motor->current_tick - motor->last_tick > DJI_HALF_MAX_TICKS)
    {
        motor->total_round--;
    }
    else if (motor->current_tick - motor->last_tick < -4096)
    {
        motor->total_round++;
    }
    motor->total_angle_rad = ((motor->total_round) * 2 * PI + motor->absolute_angle_rad) * motor->reduction_ratio;
}