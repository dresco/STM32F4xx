/*

  can.c - CAN bus driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2022 Jon Escombe

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "can.h"
#include "grbl/hal.h"

/*
 * Static function prototypes
 */

/*
 * Static variables
 */
static CAN_HandleTypeDef hcan1;
static CAN_FilterTypeDef sFilterConfig;
static CAN_TxHeaderTypeDef TxHeader;
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t TxData[8];
static uint8_t RxData[8];
static uint32_t TxMailbox;
static bool rxPendingData;

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#ifdef CAN_QUEUE_RX_IN_IRQ
    can_get();
#else
    rxPendingData = true;
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
}

bool can_rx_pending(void)
{
    return rxPendingData;
}

void can_get(void)
{
    /* May either be called in interrupt context from the RX fifo callback, or from the polling
     * loop of the higher level canbus driver, depending whether CAN_QUEUE_RX_IN_IRQ is defined.
     */

    canbus_message_t message;

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

        printf("can_get(), adding pending RX data queue..\n");
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader,RxData) == HAL_OK) {

            /* Attempt to add incoming message to the RX queue.
             *
             * Note: not currently checking for success, if there is no space available we
             * would just end up dropping messages somewhere else (i.e. in the CAN RX mailbox)..
             */
            message.id = RxHeader.StdId;
            message.len = RxHeader.DLC;
            memcpy(message.data, RxData, message.len);

            canbus_queue_rx(message);
        }
    }
#ifndef CAN_QUEUE_RX_IN_IRQ
    rxPendingData = false;
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
}

uint8_t can_put(canbus_message_t message)
{
    TxHeader.DLC = message.len;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = message.id;

    memcpy(TxData, message.data, message.len);

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        printf("can_put(), error sending message..\n");
        return (0);
    }

    return (1);
}

uint8_t can_stop(void)
{
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeInit(&hcan1);

    return (0);
}

uint8_t can_start(uint32_t baud)
{
    /* Initialisation */
    hcan1.Instance = CAN1;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    switch (baud) {

        /*
         * Can bit time calculations taken from http://www.bittiming.can-wiki.info/#bxCAN
         *
         * todo: get the CAN peripheral clock speed and use that instead of board definition?
         *
         */

#ifdef NUCLEO_F446
        /* Nucleo F446 running at 180MHz with 45MHz APB1 clock */

        case 125000:
            hcan1.Init.Prescaler = 20;
            hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
            hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;

        case 250000:
            hcan1.Init.Prescaler = 10;
            hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
            hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;

        case 500000:
            hcan1.Init.Prescaler = 5;
            hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
            hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;

        case 1000000:
            hcan1.Init.Prescaler = 3;
            hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
            hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;
#endif

        default:
            /* Unsupported baud rate */
            printf("can_start(), error - unsupported baud rate\n");
            return(0);
    }

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return(0);
    }

    /* Filter configuration */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        return(0);
    }

    /* Start the CAN peripheral (calls the MspInit function) */
    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return(0);
    }

    /* Add the callback for received data */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return(0);
    }

    /* Successfully initialised */
    return(1);
}

/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hcan->Instance==CAN1)
    {
    /* USER CODE BEGIN CAN1_MspInit 0 */

    /* USER CODE END CAN1_MspInit 0 */

    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* USER CODE BEGIN CAN1_MspInit 1 */

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_CAN,
        .port = GPIOA,
        .pin = 11,
        .mode = { .mask = PINMODE_NONE },
        .description = "CAN"
    };

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_CAN,
        .port = GPIOA,
        .pin = 12,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "CAN"
    };
    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    /* USER CODE END CAN1_MspInit 1 */
    }
}

/**
* @brief CAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
    if(hcan->Instance==CAN1)
    {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */

    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
    }
}
