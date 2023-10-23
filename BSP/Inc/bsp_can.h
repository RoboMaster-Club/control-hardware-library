#ifndef CAN_BSP_H
#define CAN_BSP_H

#include "FreeRTOS.h"
#include "can.h"
#include "freertos.h"
#include "queue.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_param.h"

#include "dm4310.h"
#include "mf_motor.h"

typedef struct 
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
} CAN_Tx_Pack_t;

typedef struct
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
} CAN_Rx_Pack_t;

extern osMessageQId can1_tx_queueHandle;
extern osMessageQId can1_rx_queueHandle;
/*
 * Init the filter and start CAN communication 
 */
extern void CAN_BSP_Init(CAN_HandleTypeDef *hcanx);

/*
 * Send the can message to Message Queue
 */
extern void CAN_BSP_SendTOQueue(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8]);

/*
 * Send the can message
 */
extern void CAN_BSP_Send(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8]);

extern void CAN_BSP_Receive(CAN_HandleTypeDef *hcanx);
#endif
