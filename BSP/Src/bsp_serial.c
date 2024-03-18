#include "bsp_serial.h"

#include "usart.h"

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    //while(!huart6.Instance->SR & UART_FLAG_IDLE);
    return len;
}