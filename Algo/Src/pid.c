#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_max, float ki_max, float dead_zone);
void PID_Reset(PID_t *pid);
float PID(PID_t *pid, float error);

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_max, float ki_max, float dead_zone)
{
    pid->kp = kp;
    pid->kd = kd;
    pid->ki = ki;
    pid->error_last = 0;
    pid->error_sum = 0;
    pid->ki_max = ki_max;
    pid->output_max = output_max;
    pid->dead_zone = dead_zone;
}

void PID_Reset(PID_t *pid)
{
    pid->error_last = 0;
    pid->error_sum = 0;
    pid->output = 0;
    pid->error_sum = 0;
}

float PID(PID_t *pid, float error)
{
    if (fabs(error) < pid->dead_zone) error = 0;
    pid->error_sum += error;
    pid->i_out = pid->error_sum * pid->ki;
    
    pid->output = pid->kp * error + pid->ki * pid->error_sum + pid->kd * (error - pid->error_last);
    
    pid->error_last = error;
    
    if (fabs(pid->output) >= pid->output_max)
    {
        pid->output = (fabs(pid->output)/pid->output) * pid->output_max;
    }
    return pid->output;
}
