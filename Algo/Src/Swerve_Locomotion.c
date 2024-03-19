#include "Swerve_Locomotion.h"
#include <stdbool.h>
#include "user_math.h"
#include "dji_motor.h"

Swerve_Module_t g_swerve_fl, g_swerve_fr, g_swerve_rl, g_swerve_rr;
float last_swerve_angle[4] = {.0f, .0f, .0f, .0f};
float azimuth_zero_offset_array[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // encoder ticks
Module_State_t kanzhege;
Module_State_t yekankanzhege;
// Inverse kinematics matrix for a 4 module swerve, modules defined counterclockwise (like quadrants)
float swerve_state_matrix[8][3] = {
    {0, 1, -(WHEEL_BASE / 2)}, // front left id 1
    {1, 0, -(TRACK_WIDTH / 2)},
    {0, 1, -(WHEEL_BASE / 2)}, // rear left id 2
    {1, 0, +(TRACK_WIDTH / 2)},
    {0, 1, +(WHEEL_BASE / 2)}, // rear right id 3
    {1, 0, +(TRACK_WIDTH / 2)},
    {0, 1, +(WHEEL_BASE / 2)}, // front right id 4
    {1, 0, -(TRACK_WIDTH / 2)}};

void Swerve_Init(void);
void Swerve_Processing(void);
void Set_Module_Output(Swerve_Module_t *swerve_module, Module_State_t desired_state);

/* Initialize physical constants of each module */
void Swerve_Init()
{
    Motor_Config_t azimuth_motor_config = {
        .control_mode = POSITION_CONTROL,
        .angle_pid =
            {
                .kp = 28000.0f,
                .ki = .0f,
                .output_limit = GM6020_MAX_CURRENT,
                .integral_limit = 1000.0f,
            },
    };

    Motor_Config_t drive_motor_config = {
        .control_mode = VELOCITY_CONTROL,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT,
            }};
    azimuth_motor_config.motor_reversal = MOTOR_REVERSAL_REVERSED;
    // left side
    azimuth_motor_config.can_bus = 1;
    drive_motor_config.can_bus = 1;

    azimuth_motor_config.offset = 6070;
    azimuth_motor_config.speed_controller_id = 1;
    drive_motor_config.speed_controller_id = 1;
    drive_motor_config.motor_reversal = MOTOR_REVERSAL_REVERSED;

    g_swerve_fl.azimuth_motor = DJI_Motor_Init(&azimuth_motor_config, GM6020);
    g_swerve_fl.drive_motor = DJI_Motor_Init(&drive_motor_config, M3508);

    azimuth_motor_config.offset = 4830;
    azimuth_motor_config.speed_controller_id = 2;
    drive_motor_config.speed_controller_id = 2;
    drive_motor_config.motor_reversal = MOTOR_REVERSAL_REVERSED;

    g_swerve_rl.azimuth_motor = DJI_Motor_Init(&azimuth_motor_config, GM6020);
    g_swerve_rl.drive_motor = DJI_Motor_Init(&drive_motor_config, M3508);

    // right side
    azimuth_motor_config.can_bus = 2;
    drive_motor_config.can_bus = 2;

    azimuth_motor_config.offset = 1940;
    azimuth_motor_config.speed_controller_id = 3;
    drive_motor_config.speed_controller_id = 3;
    drive_motor_config.motor_reversal = MOTOR_REVERSAL_NORMAL;

    g_swerve_rr.azimuth_motor = DJI_Motor_Init(&azimuth_motor_config, GM6020);
    g_swerve_rr.drive_motor = DJI_Motor_Init(&drive_motor_config, M3508);

    azimuth_motor_config.offset = 5450;
    azimuth_motor_config.speed_controller_id = 4;
    drive_motor_config.speed_controller_id = 4;
    drive_motor_config.motor_reversal = MOTOR_REVERSAL_NORMAL;

    g_swerve_fr.azimuth_motor = DJI_Motor_Init(&azimuth_motor_config, GM6020);
    g_swerve_fr.drive_motor = DJI_Motor_Init(&drive_motor_config, M3508);
}

/* Scale wheel speeds to max possible speed while preserving ratio between modules.*/
Module_State_Array_t Desaturate_Wheel_Speeds(Module_State_Array_t module_state_array)
{
    float highest_speed = fabsf(module_state_array.states[0].speed);
    for (int i = 1; i < NUMBER_OF_MODULES; i++) // start from 1 to find the highest speed
    {
        highest_speed = fmaxf(highest_speed, fabsf(module_state_array.states[i].speed));
    }
    if (highest_speed > 0.01f) // avoid division by zero
    {
        float desaturation_coefficient = fabsf(SWERVE_MAX_SPEED / highest_speed);
        Module_State_Array_t desaturated_module_states = {0}; // initialize the struct to zero

        for (int i = 0; i < NUMBER_OF_MODULES; i++)
        {
            desaturated_module_states.states[i].speed = module_state_array.states[i].speed * desaturation_coefficient;
            desaturated_module_states.states[i].angle = module_state_array.states[i].angle;
        }

        return desaturated_module_states;
    }
    return module_state_array;
}


/* Convert chassis speeds to module states using inverse kinematics */
Module_State_Array_t Chassis_Speeds_To_Module_States(Chassis_Speeds_t chassis_speeds)
{
    Module_State_Array_t calculated_module_states = {0};
    if (chassis_speeds.x == 0 && chassis_speeds.y == 0 && chassis_speeds.omega == 0)
    {
        for (int i = 0; i < NUMBER_OF_MODULES; i++)
        {
            calculated_module_states.states[i].speed = 0;
            calculated_module_states.states[i].angle = last_swerve_angle[i];
        }
    }
    else
    {
        // Multiply the inverse kinematics matrix by the chassis speeds vector
        float module_states_matrix[8] = {
            swerve_state_matrix[0][0] * chassis_speeds.x + swerve_state_matrix[0][1] * chassis_speeds.y + swerve_state_matrix[0][2] * chassis_speeds.omega,
            swerve_state_matrix[1][0] * chassis_speeds.x + swerve_state_matrix[1][1] * chassis_speeds.y + swerve_state_matrix[1][2] * chassis_speeds.omega,
            swerve_state_matrix[2][0] * chassis_speeds.x + swerve_state_matrix[2][1] * chassis_speeds.y + swerve_state_matrix[2][2] * chassis_speeds.omega,
            swerve_state_matrix[3][0] * chassis_speeds.x + swerve_state_matrix[3][1] * chassis_speeds.y + swerve_state_matrix[3][2] * chassis_speeds.omega,
            swerve_state_matrix[4][0] * chassis_speeds.x + swerve_state_matrix[4][1] * chassis_speeds.y + swerve_state_matrix[4][2] * chassis_speeds.omega,
            swerve_state_matrix[5][0] * chassis_speeds.x + swerve_state_matrix[5][1] * chassis_speeds.y + swerve_state_matrix[5][2] * chassis_speeds.omega,
            swerve_state_matrix[6][0] * chassis_speeds.x + swerve_state_matrix[6][1] * chassis_speeds.y + swerve_state_matrix[6][2] * chassis_speeds.omega,
            swerve_state_matrix[7][0] * chassis_speeds.x + swerve_state_matrix[7][1] * chassis_speeds.y + swerve_state_matrix[7][2] * chassis_speeds.omega,
        };

        // Convert module x,y matrix to wheel speed and angle
        for (int i = 0; i < NUMBER_OF_MODULES; i++)
        {
            float x = module_states_matrix[i * 2 + 1];
            float y = module_states_matrix[i * 2];
            float speed = hypotf(x, y);

            calculated_module_states.states[i].speed = speed;

            if (speed > 1e-6f)
            {
                float angle = atan2f(y, x);
                calculated_module_states.states[i].angle = angle;
                last_swerve_angle[i] = angle;
            }
            else
            {
                calculated_module_states.states[i].angle = last_swerve_angle[i];
            }
        }
    }
    return calculated_module_states;
}

/* Set the desired modules state of each module */
void Set_Desired_States(Module_State_Array_t desired_states)
{
    g_swerve_fl.module_state = desired_states.states[0];
    g_swerve_rl.module_state = desired_states.states[1];
    g_swerve_rr.module_state = desired_states.states[2];
    g_swerve_fr.module_state = desired_states.states[3];
}

/* Commands modules to stop moving and reset angle to 0. Should be called on robot enable */
void Reset_Modules()
{
    Module_State_Array_t desired_states = {0};
    Set_Desired_States(desired_states);
}

/* Optimize wheel angle so wheel doesn't have to rotate more than 90deg*/
Module_State_t Optimize_Module_Angle(Module_State_t input_state, float measured_angle)
{
    Module_State_t optimized_module_state = {0};
    float wheel_angle_delta = input_state.angle - measured_angle;

    if (wheel_angle_delta > PI / 2 || wheel_angle_delta < -PI / 2)
    { // if the delta is more than 90 degrees
        optimized_module_state.speed = -1.0f * input_state.speed * 60.0f / (PI * Wheel_Diameter);
        optimized_module_state.angle = input_state.angle + ((wheel_angle_delta > 0) ? -PI : PI);
    }
    // else if(wheel_angle_delta>PI || wheel_angle_delta<-PI)
    // {
    //     optimized_module_state.speed = 1.0f * input_state.speed * 60.0f / (PI * Wheel_Diameter);
    // 	optimized_module_state.angle = input_state.angle + ((wheel_angle_delta>0) ? -2*PI:2*PI);
    // }
    else
    {
        optimized_module_state.speed = 1.0f * input_state.speed * 60.0f / (PI * Wheel_Diameter);
        optimized_module_state.angle = input_state.angle;
    }

    return optimized_module_state;
}

/*Command motors to output calculated module state*/
void Set_Module_Output(Swerve_Module_t *swerve_module, Module_State_t desired_state)
{
    // DJI_Motor_Set_Angle(swerve_module->azimuth_motor,desired_state.angle);
    // DJI_Motor_Set_Velocity(swerve_module->drive_motor,desired_state.speed* 60 / (PI * Wheel_Diameter));

    Module_State_t optimized_module_state = Optimize_Module_Angle(desired_state, DJI_Motor_Get_Angle(swerve_module->azimuth_motor));
    kanzhege = optimized_module_state;
    DJI_Motor_Set_Angle(swerve_module->azimuth_motor, optimized_module_state.angle);
    DJI_Motor_Set_Velocity(swerve_module->drive_motor, optimized_module_state.speed);
}

#pragma message "change this comment"
/* Takes driver input (-1 to 1) and sets respective module outputs */
void Swerve_Drive(float x, float y, float omega)
{
    x *= SWERVE_MAX_SPEED; // convert to m/s
    y *= SWERVE_MAX_SPEED;
    omega *= SWERVE_MAX_ANGLUAR_SPEED; // convert to rad/s
    Chassis_Speeds_t desired_chassis_speeds = {.x = x, .y = y, .omega = omega};
    Set_Desired_States(Chassis_Speeds_To_Module_States(desired_chassis_speeds));

    Set_Module_Output(&g_swerve_fr, g_swerve_fr.module_state);
    Set_Module_Output(&g_swerve_fl, g_swerve_fl.module_state);
    Set_Module_Output(&g_swerve_rr, g_swerve_rr.module_state);
    Set_Module_Output(&g_swerve_rl, g_swerve_rl.module_state);
}

void Swerve_Disable()
{
    DJI_Motor_Disable(g_swerve_fr.azimuth_motor);
    DJI_Motor_Disable(g_swerve_fr.drive_motor);
    DJI_Motor_Disable(g_swerve_fl.azimuth_motor);
    DJI_Motor_Disable(g_swerve_fl.drive_motor);
    DJI_Motor_Disable(g_swerve_rr.azimuth_motor);
    DJI_Motor_Disable(g_swerve_rr.drive_motor);
    DJI_Motor_Disable(g_swerve_rl.azimuth_motor);
    DJI_Motor_Disable(g_swerve_rl.drive_motor);
}
