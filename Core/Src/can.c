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
#include "can_driver.h"
#include "ringbuffer.h"

uint8_t ubKeyNumber = 0x0;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef _rx_header;
uint8_t TxData[8];
uint8_t _rx_data[8];
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
  hcan.Init.Prescaler = 96;
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

    const uint32_t error_filter = CAN_IT_ERROR_WARNING
            | CAN_IT_ERROR_PASSIVE
            | CAN_IT_BUSOFF
            | CAN_IT_LAST_ERROR_CODE
            | CAN_IT_ERROR
            | CAN_IT_RX_FIFO0_MSG_PENDING;

    if (HAL_CAN_ActivateNotification(&hcan, error_filter) != HAL_OK) {

        /* Notification Error */
        Error_Handler();
    }

    /*##-5- Configure Transmission process #####################################*/
//    TxHeader.StdId = 0x321;
//    TxHeader.ExtId = 0x01;
//    TxHeader.RTR = CAN_RTR_DATA;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.DLC = 2;
//    TxHeader.TransmitGlobalTime = DISABLE;

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

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(CEC_CAN_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
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

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#define CAN_ERROR_MSG_MAPPING_LEN 23

const struct {
    uint32_t error_id;
    const char* human_readable_msg;
} can_error_msg_mapping[CAN_ERROR_MSG_MAPPING_LEN] = {
{HAL_CAN_ERROR_NONE            , "No error                                            "},
{HAL_CAN_ERROR_EWG             , "Protocol Error Warning                              "},
{HAL_CAN_ERROR_EPV             , "Error Passive                                       "},
{HAL_CAN_ERROR_BOF             , "Bus-off error                                       "},
{HAL_CAN_ERROR_STF             , "Stuff error                                         "},
{HAL_CAN_ERROR_FOR             , "Form error                                          "},
{HAL_CAN_ERROR_ACK             , "Acknowledgment error                                "},
{HAL_CAN_ERROR_BR              , "Bit recessive error                                 "},
{HAL_CAN_ERROR_BD              , "Bit dominant error                                  "},
{HAL_CAN_ERROR_CRC             , "CRC error                                           "},
{HAL_CAN_ERROR_RX_FOV0         , "Rx FIFO0 overrun error                              "},
{HAL_CAN_ERROR_RX_FOV1         , "Rx FIFO1 overrun error                              "},
{HAL_CAN_ERROR_TX_ALST0        , "TxMailbox 0 transmit failure due to arbitration lost"},
{HAL_CAN_ERROR_TX_TERR0        , "TxMailbox 0 transmit failure due to transmit error  "},
{HAL_CAN_ERROR_TX_ALST1        , "TxMailbox 1 transmit failure due to arbitration lost"},
{HAL_CAN_ERROR_TX_TERR1        , "TxMailbox 1 transmit failure due to transmit error  "},
{HAL_CAN_ERROR_TX_ALST2        , "TxMailbox 2 transmit failure due to arbitration lost"},
{HAL_CAN_ERROR_TX_TERR2        , "TxMailbox 2 transmit failure due to transmit error  "},
{HAL_CAN_ERROR_TIMEOUT         , "Timeout error                                       "},
{HAL_CAN_ERROR_NOT_INITIALIZED , "Peripheral not initialized                          "},
{HAL_CAN_ERROR_NOT_READY       , "Peripheral not ready                                "},
{HAL_CAN_ERROR_NOT_STARTED     , "Peripheral not started                              "},
{HAL_CAN_ERROR_PARAM           ,"Parameter error                                      "}
};

uint32_t __CANProcessErrors(CAN_HandleTypeDef *hcan) {
    uint32_t error = HAL_CAN_GetError(hcan);
    if (error != HAL_CAN_ERROR_NONE) {
        for (int i = 0; i < CAN_ERROR_MSG_MAPPING_LEN; i++) {
            if (error & can_error_msg_mapping[i].error_id) {
//                print("CAN Error is: ");
//                println("%s", can_error_msg_mapping[i].human_readable_msg);
            }
        }
        HAL_CAN_ResetError(hcan);
    }
    return error;
}


ring_buffer_t can_rx_ring_buffer;
#define RX_FIFO  0
struct can_frame_s new;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error = __CANProcessErrors(hcan);
    if(error == HAL_CAN_ERROR_NONE) {
        uint32_t filllevel = HAL_CAN_GetRxFifoFillLevel(hcan, RX_FIFO);
        while(filllevel--){
            HAL_CAN_GetRxMessage(hcan, RX_FIFO, &_rx_header, new.data);
            if(_rx_header.IDE == CAN_ID_STD){
                new.id = _rx_header.StdId;
                new.extended = 0;
            }else{
                new.id = _rx_header.ExtId;
                new.extended = 1;
            }
            new.length = _rx_header.DLC;
            new.remote = _rx_header.RTR;
            new.timestamp = _rx_header.Timestamp;

            // fill ring buffer only if it has free space. Otherwise miss the CAN frame.
            size_t free = RING_BUFFER_SIZE - ring_buffer_num_items(&can_rx_ring_buffer);
            if(free >= sizeof(struct can_frame_s)){
                ring_buffer_queue_arr(&can_rx_ring_buffer, (char*)&new, sizeof(new));
            }
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){

}

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

#ifdef STM32F042x6
struct {
    uint32_t bitrate;
    uint32_t prescaler;
} const can_bitrate_map[10] = {
        {.bitrate=10000, .prescaler=800},
        {.bitrate=20000, .prescaler=400},
        {.bitrate=25000, .prescaler=240},
        {.bitrate=50000, .prescaler=120},
        {.bitrate=100000, .prescaler=80},
        {.bitrate=125000, .prescaler=64},
        {.bitrate=250000, .prescaler=32},
        {.bitrate=500000, .prescaler=16},
        {.bitrate=800000, .prescaler=10},
        {.bitrate=1000000, .prescaler=8}

};
#else
#error "Unsupported chip! CAN bitrate prescalers should be precalculated."
#endif

bool can_set_bitrate(uint32_t bitrate)
{
    if(HAL_CAN_DeInit(&hcan) != HAL_OK){
        return false;
    }

    bool is_in_list = false;
    for(int i = 0; i < 9; i++){
        if(can_bitrate_map[i].bitrate == bitrate){
            is_in_list = true;
            break;
        }
    }

    if(is_in_list){
        for(int i = 0; i < 10; i++){
            if(can_bitrate_map[i].bitrate == bitrate){
                hcan.Init.Prescaler = can_bitrate_map[i].prescaler;
                break;
            }
        }
    }

    if(HAL_CAN_Init(&hcan) != HAL_OK){
        return false;
    }

    return is_in_list;
}

bool can_open(int mode)
{
    /* Create and initialize ring buffer */
    ring_buffer_init(&can_rx_ring_buffer);

    hcan.Init.Mode = mode;
    if (HAL_CAN_Init(&hcan) != HAL_OK){
        return false;
    }

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
        return false;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        return false;
    }

    const uint32_t error_filter = CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
            | CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR | CAN_IT_RX_FIFO0_MSG_PENDING;

    if (HAL_CAN_ActivateNotification(&hcan, error_filter) != HAL_OK) {
        return false;
    }

    return true;
}


bool can_get_next_frame(struct can_frame_s* can_frame){

    size_t elements = 0;
    //critical section begin
    {
        HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
        elements = ring_buffer_num_items(&can_rx_ring_buffer);
        HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
    }//clritical section end

    if(elements > sizeof(struct can_frame_s)){
        ring_buffer_dequeue_arr(&can_rx_ring_buffer, (char*)can_frame, sizeof(struct can_frame_s));
        return true;
    }

    return false;
}

void can_close(void)
{
    HAL_CAN_Stop(&hcan);
    HAL_CAN_DeInit(&hcan);
}

void can_init(void)
{
}

/* USER CODE END 1 */
