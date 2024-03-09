#ifndef CAN_H
#define CAN_H

#include "FreeRTOS.h"
#include "can.h"
#include "freertos.h"
#include "queue.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "dm4310.h"
#include "mf_motor.h"

#define CAN_MAX_DEVICE 5

typedef struct 
{
    uint8_t can_bus;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
} CAN_Tx_Pack_t;

typedef struct _
{
     can_bus;
    CAN_RxHeaderTypeDef rx_header;
    uint16_t rx_id;
    uint8_t data[8];
    void (*can_module_callback)(struct _ *);
} CAN_Rx_Pack_t;

extern osMessageQId can1_tx_queueHandle;
extern osMessageQId can2_tx_queueHandle;
extern osMessageQId can1_rx_queueHandle;
extern osMessageQId can2_rx_queueHandle;
/*
 * Init the filter and start CAN communication 
 */
void CAN_Init(CAN_HandleTypeDef *hcanx);

/*
 * Send the can message to Message Queue
 */
void CAN_SendTOQueue(uint8_t can_bus, uint32_t id, uint8_t data[8]);

/*
 * Send the can message
 */
void CAN_Send(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8]);

void CAN_Receive(CAN_HandleTypeDef *hcanx);
#endif
