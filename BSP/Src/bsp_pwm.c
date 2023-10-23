#include "bsp_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim10;

void BSP_PWM_SetPWM()
{
    
}

void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
