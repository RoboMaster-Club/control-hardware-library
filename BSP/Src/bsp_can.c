#include "bsp_can.h"

CAN_RxHeaderTypeDef g_rx_header;
uint8_t g_can_data[8];

void CAN_BSP_Init(CAN_HandleTypeDef *hcanx);
void CAN_BSP_SendTOQueue(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8]);
void CAN_BSP_Send(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8]);
void CAN_BSP_Receive(CAN_HandleTypeDef *hcanx);
void CAN_BSP_Init(CAN_HandleTypeDef *hcanx)
{
    /* set the CAN filter */
    CAN_FilterTypeDef can_filter;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
    can_filter.SlaveStartFilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterActivation = CAN_FILTER_ENABLE;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    HAL_CAN_ConfigFilter(hcanx, &can_filter);

    /* Start CAN Communication */
    HAL_CAN_Start(hcanx);

    /* Activate Interrupt */
    HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_BSP_SendTOQueue(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8])
{
    /* Initialize the data pack for queuing*/
    CAN_Tx_Pack_t can_tx_pack;

    /* Set tx_header */
    can_tx_pack.tx_header.StdId = id;
    can_tx_pack.tx_header.IDE = CAN_ID_STD;
    can_tx_pack.tx_header.RTR = CAN_RTR_DATA;
    can_tx_pack.tx_header.DLC = 0x08;

    /* Set data */
    memcpy(can_tx_pack.data, data, sizeof(uint8_t[8]));

    if (hcanx->Instance == CAN1)
        xQueueSend(can1_tx_queueHandle, &can_tx_pack, 0);
}

void CAN_BSP_Send(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8])
{
    /* Initialize send header*/
    CAN_TxHeaderTypeDef tx_header;

    /* Set tx_header */
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x08;

    HAL_CAN_AddTxMessage(hcanx, &tx_header, data, (uint32_t *)CAN_TX_MAILBOX0);
}

void CAN_BSP_Receive(CAN_HandleTypeDef *hcanx)
{

    CAN_Rx_Pack_t can_rx_pack;

    HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0, &g_rx_header, g_can_data);
    can_rx_pack.rx_header = g_rx_header;
    memcpy(can_rx_pack.data, g_can_data, sizeof(uint8_t[8]));
    xQueueSendToBackFromISR(can1_rx_queueHandle, &can_rx_pack, 0);
}

void CAN_BSP_CAN1Tx(void const *argument)
{
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
//    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
    CAN_Tx_Pack_t can_tx_pack;

    for (;;)
    {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3)
        {
            xQueueReceive(can1_tx_queueHandle, &can_tx_pack, portMAX_DELAY);
            HAL_CAN_AddTxMessage(&hcan1, &can_tx_pack.tx_header, can_tx_pack.data, (uint32_t *)CAN_TX_MAILBOX0);
        }
//        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
