#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit, float dead_zone);
void PID_Reset(PID_t *pid);
float PID(PID_t *pid, float error);

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit, float dead_zone)
{
    pid->kp = kp;
    pid->kd = kd;
    pid->ki = ki;
    pid->prev_error = 0;
    pid->error_sum = 0;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->dead_zone = dead_zone;
}

void PID_Reset(PID_t *pid)
{
    pid->prev_error = 0;
    pid->error_sum = 0;
    pid->output = 0;
    pid->error_sum = 0;
}

float PID(PID_t *pid, float error)
{
    if (fabs(error) < pid->dead_zone) error = 0;
    pid->error_sum += error;
    pid->i_out = pid->error_sum * pid->ki;
    
    pid->output = pid->kp * error + pid->ki * pid->error_sum + pid->kd * (error - pid->prev_error);
    
    pid->prev_error = error;
    
    if (fabs(pid->output) >= pid->output_limit)
    {
        pid->output = (fabs(pid->output)/pid->output) * pid->output_limit;
    }
    return pid->output;
}
