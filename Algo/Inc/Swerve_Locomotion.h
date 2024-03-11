#ifndef SWERVE_MODULE_h
#define SWERVE_MODULE_h

#include "pid.h"
#include "stdbool.h"
#include "dji_motor.h"
#include "math.h"

// PHYSICAL CONSTANTS
#define SWERVE_MAX_SPEED 2.0f          // m/s
#define SWERVE_MAX_ANGLUAR_SPEED 90.0f // deg/s
#define TRACK_WIDTH 0.23f              // m, measured wheel to wheel (side to side)
#define WHEEL_BASE 0.23f               // m, measured wheel to wheel (up and down)
#define Wheel_Diameter 0.25f
#define Azimuth_Gear_Ratio 1.0f
#define Drive_Gear_Ratio 16.8f

#define NUMBER_OF_MODULES 4

// PID Constants
#define Azimuth_kP 10000.0f // TODO proper units
#define Azimuth_kI 0.0f
#define Azimuth_kD 0.0f
#define Azimuth_Output_Max 18000.0f

typedef struct
{
    bool azimuth_encoder_reversed;
    float azimuth_zero_encoder_offset;
    uint16_t azimuth_can_id;
    PID_t azimuth_pid;
    DJI_Motor_t azimuth_motor;

    uint16_t drive_can_id;
    PID_t drive_pid;
    DJI_Motor_t drive_motor;

    Module_State_t module_state;
}	Swerve_Module_t;

typedef struct
{
    Swerve_Module_t modules[NUMBER_OF_MODULES];
} Swerve_t;

typedef struct
{
    float x, y, omega;
} Chassis_Speeds_t;

typedef struct
{
    float speed; // m/s
    float angle; // deg
} Module_State_t;

typedef struct
{
    Module_State_t states[NUMBER_OF_MODULES];
} Module_State_Array_t;

extern void Init_Swerve_Module(Swerve_Module_t *swerve_module, bool azimuth_encoder_reversed, int azimuth_can_id);
extern void Set_Module_Output(Swerve_Module_t *swerve_module, Module_State_t desired_state);

#endif