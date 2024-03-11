#include "Swerve_Locomotion.h"
#include "stdbool.h"
#include "user_math.h"

Swerve_t swerve;
float last_swerve_angle[4] = {.0f, .0f, .0f, .0f};
bool azimuth_encoder_reversed_array[4] = {false, false, false, false}; // TODO check polarity
float azimuth_zero_offset_array[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // encoder ticks

// Inverse kinematics matrix for a 4 module swerve, modules defined counterclockwise (like quadrants)
float swerve_state_matrix[8][3] = {
    {1, 0, -(WHEEL_BASE / 2)}, // front right 0
    {0, 1, -(TRACK_WIDTH / 2)},
    {1, 0, -(WHEEL_BASE / 2)}, // front left 1
    {0, 1, +(TRACK_WIDTH / 2)},
    {1, 0, +(WHEEL_BASE / 2)}, // back left 2
    {0, 1, +(TRACK_WIDTH / 2)},
    {1, 0, +(WHEEL_BASE / 2)}, // back right 3
    {0, 1, -(TRACK_WIDTH / 2)}
};

void Init_Modules(void);
void Swerve_Processing(Swerve_t *swerve);

/* Initialize physical constants of each module */
void Init_Modules() {} //TODO init modules

/* Scale wheel speeds to max possible speed while preserving ratio between modules.*/
Module_State_Array_t Desaturate_Wheel_Speeds(Module_State_Array_t module_state_array)
{
    float highest_speed = fabsf(module_state_array.states[0].speed);
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        if (fabsf(module_state_array.states[i].speed) > fabsf(highest_speed))
        {
            highest_speed = module_state_array.states[i].speed;
        }
    }
    if (fabs(highest_speed) > 0.01f)
    {
        float desaturation_coefficient = fabs(SWERVE_MAX_SPEED / highest_speed);
        // 1.4/(1/1.4)
        Module_State_Array_t desaturated_module_states;

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
        float module_states_matrix[8][1] = {
            {swerve_state_matrix[0][0] * chassis_speeds.x + swerve_state_matrix[0][1] * chassis_speeds.y + swerve_state_matrix[0][2] * chassis_speeds.omega},
            {swerve_state_matrix[1][0] * chassis_speeds.x + swerve_state_matrix[1][1] * chassis_speeds.y + swerve_state_matrix[1][2] * chassis_speeds.omega},
            {swerve_state_matrix[2][0] * chassis_speeds.x + swerve_state_matrix[2][1] * chassis_speeds.y + swerve_state_matrix[2][2] * chassis_speeds.omega},
            {swerve_state_matrix[3][0] * chassis_speeds.x + swerve_state_matrix[3][1] * chassis_speeds.y + swerve_state_matrix[3][2] * chassis_speeds.omega},
            {swerve_state_matrix[4][0] * chassis_speeds.x + swerve_state_matrix[4][1] * chassis_speeds.y + swerve_state_matrix[4][2] * chassis_speeds.omega},
            {swerve_state_matrix[5][0] * chassis_speeds.x + swerve_state_matrix[5][1] * chassis_speeds.y + swerve_state_matrix[5][2] * chassis_speeds.omega},
            {swerve_state_matrix[6][0] * chassis_speeds.x + swerve_state_matrix[6][1] * chassis_speeds.y + swerve_state_matrix[6][2] * chassis_speeds.omega},
            {swerve_state_matrix[7][0] * chassis_speeds.x + swerve_state_matrix[7][1] * chassis_speeds.y + swerve_state_matrix[7][2] * chassis_speeds.omega},
        };

        // Convert module x,y matrix to wheel speed and angle
        for (int i = 0; i < NUMBER_OF_MODULES; i++)
        {
            float x = module_states_matrix[i * 2][0];
            float y = module_states_matrix[i * 2 + 1][0];

            float speed = hypotf(x, y);
            if (speed > 1e-6f)
            {
                y /= speed;
                x /= speed;
                float angle = atan2f(x, y);

                calculated_module_states.states[i].angle = angle;
                last_swerve_angle[i] = angle;
            }
            else
            {
                x = 0.0f;
                y = 1.0f;
            }

            calculated_module_states.states[i].speed = speed;
            calculated_module_states.states[i].angle = last_swerve_angle[i];
        }
        return calculated_module_states;
    }
}

/* Set the desired modules state of each module */
void Set_Desired_States(Module_State_Array_t desired_states)
{
    desired_states = Desaturate_Wheel_Speeds(desired_states);
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        swerve.modules[i].module_state = desired_states.states[i];
    }
}

/* Takes driver input (-1 to 1) and sets respective module outputs */
void Drive(Swerve_t *swerve, float x, float y, float omega)
{
    x *= SWERVE_MAX_SPEED; // convert to m/s
    y *= SWERVE_MAX_SPEED;
    omega *= SWERVE_MAX_ANGLUAR_SPEED; // convert to rad/s
    Chassis_Speeds_t desired_chassis_speeds = {.x = x, .y = y, .omega = omega};

    Set_Desired_States(Chassis_Speeds_To_Module_States(desired_chassis_speeds));

    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        Set_Module_Output(&(swerve->modules[i]), swerve->modules[i].module_state);
    }
}

/* Commands modules to stop moving and reset angle to 0. Should be called on robot enable */
void Reset_Modules()
{
    Module_State_t zero_state = {.speed = 0, .angle = 0};
    Module_State_Array_t desired_states = {zero_state, zero_state, zero_state, zero_state};

    Set_Desired_States(desired_states);
}

void Set_Module_Output(Swerve_Module_t *swerve_module, Module_State_t desired_state);

/* Initialize physical swerve constants */
void Init_Swerve_Module(Swerve_Module_t *swerve_module, bool azimuth_encoder_reversed, int azimuth_can_id)
{ // TODO add relevant constants
	swerve_module->azimuth_encoder_reversed = azimuth_encoder_reversed;
	swerve_module->azimuth_can_id = azimuth_can_id;
}

/* Optimize wheel angle so wheel doesn't have to rotate more than 90deg*/
Module_State_t Optimize_Module_Angle(Module_State_t input_state, float measured_angle)
{
	Module_State_t optimized_module_state;
	float wheel_angle_delta = input_state.angle - measured_angle;

	if (fabsf(wheel_angle_delta) > PI / 2)
	{ // if the delta is more than 90 degrees
		optimized_module_state.speed = -1 * input_state.speed;
		optimized_module_state.angle = fmodf(input_state.angle + PI, 2 * PI); // rotate the target by 180 degrees
	}

	return optimized_module_state;
}

/*Command motors to output calculated module state*/
void Set_Module_Output(Swerve_Module_t *swerve_module, Module_State_t desired_state)
{
	// TODO
}