/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>

uint8_t ubKeyNumber = 0x0;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox = 0;

CAN_FilterTypeDef sFilterConfig;


/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    /*##-2- Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14; // how many filters to assign to the CAN1 (master can)

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    /*##-3- Start the CAN peripheral ###########################################*/
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        /* Start Error */
        Error_Handler();
    }

    const uint32_t error_filter = CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
            | CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;

    if (HAL_CAN_ActivateNotification(&hcan, error_filter) != HAL_OK) {

        /* Notification Error */
        Error_Handler();
    }

    /*##-5- Configure Transmission process #####################################*/
    TxHeader.StdId = 0x321;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

bool can_send(uint32_t id, bool extended, bool remote, uint8_t* data, size_t length)
{
    HAL_StatusTypeDef status;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox = 0;

    uint32_t free= HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    if (free > 0) {
        tx_header.DLC = length;
        tx_header.RTR = CAN_RTR_DATA;

        if(extended){
            tx_header.ExtId = id;
            tx_header.IDE = CAN_ID_EXT;
        }else{
            tx_header.StdId = id;
            tx_header.IDE = CAN_ID_STD;
        }

        status = HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox);
    }else{
        status = HAL_BUSY;
    }

    return (HAL_OK == status);
}

//{
//    (void)arg;
//    chRegSetThreadName("CAN rx");
//    while (1) {
//        wait_on_request();
//        CANRxFrame rxf;
//        msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(10));
//        if (m != MSG_OK) {
//            continue;
//        }
//        led_set(CAN1_STATUS_LED);
//        struct can_frame_s* fp = (struct can_frame_s*)chPoolAlloc(&can_rx_pool);
//        if (fp == NULL) {
//            chSysHalt("CAN driver out of memory");
//        }
//        fp->timestamp = timestamp_get() / 1000;
//        if (rxf.IDE) {
//            fp->id = rxf.EID;
//            fp->extended = 1;
//        } else {
//            fp->id = rxf.SID;
//            fp->extended = 0;
//        }
//        if (rxf.RTR) {
//            fp->remote = 1;
//        } else {
//            fp->remote = 0;
//        }
//        fp->length = rxf.DLC;
//        memcpy(&fp->data[0], &rxf.data8[0], rxf.DLC);
//        can_rx_queue_post(fp);
//    }
//}

struct {
    uint32_t bitrate;
    uint32_t prescaler;
} const can_bitrate_map[10] = {
        {.bitrate=10000, .prescaler=200},
        {.bitrate=20000, .prescaler=100},
        {.bitrate=25000, .prescaler=80},
        {.bitrate=50000, .prescaler=40},
        {.bitrate=100000, .prescaler=20},
        {.bitrate=125000, .prescaler=16},
        {.bitrate=250000, .prescaler=8},
        {.bitrate=500000, .prescaler=4},
        {.bitrate=1000000, .prescaler=2},

};


bool can_set_bitrate(uint32_t bitrate)
{
    if(HAL_CAN_DeInit(&hcan) != HAL_OK){
        return false;
    }

    for(int i = 0; i < 8; i++){
        if(can_bitrate_map[i].bitrate == bitrate){
            hcan.Init.Prescaler = can_bitrate_map[i].prescaler;
            break;
        }
    }

    return (HAL_CAN_Init(&hcan) == HAL_OK);
}

bool can_open(int mode)
{
    if (HAL_CAN_Init(&hcan) == HAL_OK){
        hcan.Init.Mode = mode;

    }else{
      return false;
    }

    return (HAL_CAN_Start(&hcan) == HAL_OK);
}

void can_close(void)
{
    HAL_CAN_Stop(&hcan);
}

void can_init(void)
{
}

/* USER CODE END 1 */
