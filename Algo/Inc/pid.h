#ifndef PID_H
#define PID_H

#include "math.h"

typedef struct pid_t
{
    float kp;
    float ki;
    float kd;
    float ki_max;
    float i_out;
    float output_max;
    float dead_zone;
    
    float error_last;
    float error_sum;

    float output;
} PID_t;

extern void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_max, float ki_max, float dead_zone);
extern void PID_Reset(PID_t *pid);
extern float PID(PID_t *pid, float error);
#endif
