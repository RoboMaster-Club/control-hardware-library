#include "pid.h"
#include <math.h>
#include "user_math.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit, float dead_zone);
void PID_Reset(PID_t *pid);
float PID(PID_t *pid, float error);

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit, float dead_zone)
{
    pid->kp = kp;
    pid->kd = kd;
    pid->ki = ki;
    pid->prev_error = 0;
    pid->i_out = 0;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->dead_zone = dead_zone;
}

void PID_Reset(PID_t *pid)
{
    pid->prev_error = 0;
    pid->i_out = 0;
    pid->output = 0;
}

float PID(PID_t *pid, float error)
{
    if (fabs(error) < pid->dead_zone) error = 0;
    pid->i_out += error * pid->ki;
    __MAX_LIMIT(pid->i_out, -pid->integral_limit, pid->integral_limit);
    
    pid->output = pid->kp * error + pid->i_out + pid->kd * (error - pid->prev_error) + pid->kf * pid->ref;
    
    pid->prev_error = error;
    
    if (pid->output >= pid->output_limit)
    {
        pid->output =  pid->output_limit;
    } else if (pid->output <= -pid->output_limit)
    {
        pid->output = -pid->output_limit;
    }

    return pid->output;
}
