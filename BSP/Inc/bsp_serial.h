#ifndef BSP_SERIAL
#define BSP_SERIAL

#include "robot_param.h"
#include "usart.h"
#include <stdio.h>

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    //while(!huart6.Instance->SR & UART_FLAG_IDLE);
    return ch;
}
#endif
