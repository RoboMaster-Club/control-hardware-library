#ifndef SWERVE_MODULE_h
#define SWERVE_MODULE_h

#include "pid.h"
#include "stdbool.h"
#include "dji_motor.h"
#include "math.h"

// PHYSICAL CONSTANTS
#define SWERVE_MAX_SPEED 1.0f          // m/s
#define SWERVE_MAX_ANGLUAR_SPEED 3.14f // rad/s
#define TRACK_WIDTH 0.34f              // m, measured wheel to wheel (side to side)
#define WHEEL_BASE 0.34f               // m, measured wheel to wheel (up and down)
#define Wheel_Diameter 0.12f
#define Azimuth_Gear_Ratio 1.0f

#define NUMBER_OF_MODULES 4

typedef struct
{
    float speed; // m/s
    float angle; // deg
} Module_State_t;

typedef struct
{
    Module_State_t module_state;
    DJI_Motor_Handle_t *azimuth_motor;
    DJI_Motor_Handle_t *drive_motor;
}	Swerve_Module_t;

// typedef struct
// {
//     // Swerve_Module_t modules[NUMBER_OF_MODULES];
// } Swerve_t;

typedef struct
{
    float x, y, omega;
} Chassis_Speeds_t;

typedef struct
{
    Module_State_t states[NUMBER_OF_MODULES];
} Module_State_Array_t;

void Swerve_Init(void);
void Swerve_Drive(float x, float y, float omega);
void Swerve_Disable(void);
#endif
