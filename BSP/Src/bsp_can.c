#include "bsp_can.h"

static CAN_Rx_Pack_t *g_can1_rx_packs[CAN_MAX_DEVICE] = {NULL};
static uint8_t g_can1_device_count = 0;
static CAN_Rx_Pack_t *g_can2_rx_packs[CAN_MAX_DEVICE] = {NULL};
static uint8_t g_can2_device_count = 0;

CAN_RxHeaderTypeDef g_rx_header;
uint8_t g_can_data[8];

void CAN_Init(CAN_HandleTypeDef *hcanx);
void CAN_SendTOQueue(uint8_t can_bus, uint32_t id, uint8_t data[8]);
void CAN_Receive(CAN_HandleTypeDef *hcanx);

void CAN_Filter_Init(CAN_Rx_Pack_t *rx_pack)
{
    /* set the CAN filter */
    CAN_FilterTypeDef can_filter;
    can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter.FilterFIFOAssignment = (rx_pack->rx_id % 1 == 0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1; // match even can id to FIFO0, odd to FIFO1
    can_filter.FilterBank = (rx_pack->can_bus == 1) ? g_can1_device_count : g_can2_device_count;
    can_filter.SlaveStartFilterBank = 0;
    can_filter.FilterActivation = CAN_FILTER_ENABLE;
    can_filter.FilterIdHigh = (rx_pack->rx_id << 5); // standard id is 11 bit, so shift 5 bits
    can_filter.FilterIdLow = 0x0000;                 // the second id is not used
    can_filter.FilterMaskIdHigh = 0x0000;            // the third id is not used
    can_filter.FilterMaskIdLow = 0x0000;             // the fourth id is not used
    HAL_CAN_ConfigFilter((rx_pack->can_bus == 1) ? &hcan1 : &hcan2, &can_filter);
}
/**
 * @brief  CAN Device Registration Function
 * 
 * @param can_bus can bus number (1 or 2)
 * @param can_id can id of the device (0x000 to 0x7FF)
 * @param can_module_callback callback function for the can module
 * 
 * @return CAN_Rx_Pack_t* the pointer to the rx_pack
*/
CAN_Rx_Pack_t *CAN_Device_Register(uint8_t can_bus, uint16_t can_id, void (*can_module_callback)(CAN_Rx_Pack_t *rx_pack))
{
    CAN_Rx_Pack_t *rx_pack = malloc(sizeof(CAN_Rx_Pack_t));
    rx_pack->can_bus = can_bus;
    rx_pack->rx_id = can_id;
    rx_pack->can_module_callback = can_module_callback;
    switch (can_bus)
    {
    case 1:
        g_can1_device_count++;
        g_can1_rx_packs[can_id] = rx_pack;
        break;
    case 2:
        g_can2_device_count++;
        g_can2_rx_packs[can_id] = rx_pack;
        break;
    default:
        // TODO: LOG can bus need to be 1 or 2
        break;
    }
    CAN_Filter_Init(&rx_pack);
    return rx_pack;
}

void CAN_Service_Init()
{
    /* Start CAN Communication */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    /* Activate Interrupt */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN_SendTOQueue(uint8_t can_bus, uint32_t id, uint8_t data[8])
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

    switch (can_bus)
    {
    case 1:
        xQueueSend(can1_tx_queueHandle, &can_tx_pack, 0);
        break;
    case 2:
        xQueueSend(can2_tx_queueHandle, &can_tx_pack, 0);
    default:
        break;
    }
}

void CAN_Transmit(void const *argument)
{
    CAN_Tx_Pack_t can_tx_pack;
    for (;;)
    {
        // Wait for the message to be received
        xQueueReceive(can_tx_queueHandle, &can_tx_pack, portMAX_DELAY);
        CAN_HandleTypeDef *hcanx = (can_tx_pack.can_bus == 1) ? &hcan1 : &hcan2;
        // Wait for available mailbox
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcanx) == 0);
        HAL_CAN_AddTxMessage(&hcanx, &can_tx_pack.tx_header, can_tx_pack.data, (uint32_t *)CAN_TX_MAILBOX0);
    }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Rx_Pack_t rx_pack;
    if (HAL_CAN_GetRxMessage(hcan, CAN_FilterFIFO0, &rx_pack.rx_header, rx_pack.data) == HAL_OK)
    {
        uint8_t can_bus = (hcan->Instance == hcan1.Instance) ? 1 : 2;
        switch (can_bus)
        {
        case 1:
            for (uint8_t i = 0; i < g_can1_device_count; i++)
            {
                if (g_can1_rx_packs[i]->rx_id == rx_pack.rx_header.StdId)
                {
                    rx_pack.can_bus = 1;
                    rx_pack.can_module_callback(&rx_pack);
                    break;
                }
            }
            break;
        case 2:
            for (uint8_t i = 0; i < g_can2_device_count; i++)
            {
                if (g_can2_rx_packs[i]->rx_id == rx_pack.rx_header.StdId)
                {
                    rx_pack.can_bus = 2;
                    rx_pack.can_module_callback(&rx_pack);
                    break;
                }
            }
            break;
        default:
        // TODO: LOG can bus need to be 1 or 2
            break;
        }
    }
}