/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "print.h"
#include "slcan.h"
#include "ringbuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static GPIO_InitTypeDef GPIO_InitStruct;


#define PRINT_BUFFER_SIZE 256
char printBuffer[PRINT_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PRINT_BUFFER_LEN 512
char __print_buffer[PRINT_BUFFER_LEN];
char __buffer[PRINT_BUFFER_LEN];

int __print_service(const char *str) {

    int error = 0;

    uint32_t __buffer_len = strlen(__print_buffer) + strlen(str);
    if(__buffer_len < PRINT_BUFFER_LEN){
        strncat(__print_buffer, str, PRINT_BUFFER_LEN - __buffer_len);
    }else{
        error = -1;
    }

    HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart2);
    if(state & HAL_UART_STATE_READY){
        strncpy(__buffer, __print_buffer, PRINT_BUFFER_LEN);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*) __buffer, strlen(__buffer));
        __print_buffer[0] = '\0';

        error = 0;
    }

    return error;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED3 on: Transfer in reception process is correct */
	BSP_LED_Toggle(LED3);
}


//-------------------------------------------------------------------------

#define MAX_BUFFER_LEN 128

char __in_buffer[MAX_BUFFER_LEN];
static uint32_t __index = 0;

int push(uint8_t byte){

    __in_buffer[__index++] = byte;
    if(__index < MAX_BUFFER_LEN) {
        return __index;
    }else{
        return -1;
    }
}

const char* get_buffer(){
    return __in_buffer;
}

void flush_buffer(){
    __index = 0;
    memset(__in_buffer, 0, MAX_BUFFER_LEN);
}

//char* slcan_getline(void* arg)
//{
//    static char line_buffer[500];
//    static size_t pos = 0;
//    size_t i;
//    for (i = pos; i < sizeof(line_buffer); i++) {
//        int c = chnGetTimeout((BaseChannel*)arg, TIME_INFINITE);
//        if (c == STM_TIMEOUT) {
//            /* no more data, continue */
//            pos = i;
//            return NULL;
//        }
//        if (c == '\n' || c == '\r' || c == '\0') {
//            /* line found */
//            line_buffer[i] = 0;
//            pos = 0;
//            led_set(STATUS_LED); // show USB activity
//            return line_buffer;
//        } else {
//            line_buffer[i] = c;
//        }
//    }
//
//    /* reset */
//    pos = 0;
//    return NULL;
//}

char* get_line() {

    char* where = strchr(__in_buffer, '\r');
    if(where != NULL){
        return __in_buffer;
    } else {
        return NULL;
    }
}


ring_buffer_t uart_rx_ring_buffer;


uint8_t uart2_buffer[8];
volatile int global_error;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ring_buffer_queue(&uart_rx_ring_buffer, uart2_buffer[0]);
    if(HAL_OK != HAL_UART_Receive_IT(huart, uart2_buffer, 1)){
        global_error = 1;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	print_init(__print_service);

	/* Create and initialize ring buffer */
	ring_buffer_init(&uart_rx_ring_buffer);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	LED3_GPIO_CLK_ENABLE();

	/* -2- Configure IO in output push-pull mode to drive external LEDs */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = LED3_PIN;
	HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);
	BSP_LED_Off(LED3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    flush_buffer();

    HAL_UART_Receive_IT(&huart2, uart2_buffer, 2);
	while (1) {
	    char ch;
        if(0 < ring_buffer_dequeue(&uart_rx_ring_buffer, &ch)) {
            push(ch);
            char* line = get_line();

            if(line){
                BSP_LED_On(LED3);

                slcan_decode_line(line);
                HAL_UART_Transmit(&huart2, (uint8_t*)line, strlen(line), 10);
                flush_buffer();
                BSP_LED_Off(LED3);
            }
        }

        slcan_can_rx_process();

//		HAL_Delay(100);

//		int pendingRequests = HAL_CAN_IsTxMessagePending(&hcan, TxMailbox);
//		if (pendingRequests == 0) {
//			TxHeader.DLC = 1;
//			TxHeader.StdId = 0x001;
//			TxHeader.IDE = CAN_ID_STD;
//			RxData[0] = 0xAA;
//			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, RxData, &TxMailbox)
//					!= HAL_OK) {
//				Error_Handler();
//			}
//		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
