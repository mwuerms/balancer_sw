/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA15 (JTDI)   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 1 */
#include "../../../mmlib/mmutils.h"
#include "../../../mmlib/fifo.h"

#define LOG_UART USART2
#define UART_BUFFER_SIZE (256)
static volatile uint8_t uart_tx_buffer[UART_BUFFER_SIZE];
static fifo_t uart_tx_fifo = {0};

void uart_init(void) {
	fifo_init(&uart_tx_fifo, uart_tx_buffer, UART_BUFFER_SIZE);
}

void uart_process_irq(void) {
	if(LL_USART_IsActiveFlag_TXE(LOG_UART)) {
		if(fifo_try_get(&uart_tx_fifo) == false) {
			// fifo is empty, stop here
			LL_USART_DisableIT_TXE(LOG_UART);
			return;
		}
		LL_USART_TransmitData8(LOG_UART, ((uint8_t *)(uart_tx_fifo.data))[uart_tx_fifo.rd_proc]);
		fifo_finalize_get(&uart_tx_fifo);
	}
}

uint16_t uart_send_buffer(uint8_t *buffer, uint16_t length) {
	// disable IRQ
    uint32_t primask;
    primask = __get_PRIMASK();  // Save current interrupt state
    __disable_irq();            // Disable all interrupts

    // later use mutex lock
    uint16_t n;
    for(n = 0; n < length; n++) {
        if(fifo_try_append(&uart_tx_fifo) == false) {
            // fifo is full, stop here
            break;
        }
        ((uint8_t *)(uart_tx_fifo.data))[uart_tx_fifo.wr_proc] = buffer[n];
        fifo_finalize_append(&uart_tx_fifo);
    }

    if(fifo_is_empty(&uart_tx_fifo) == false) {
        // fifo is not empty, start transmitting now
    	LL_USART_EnableIT_TXE(LOG_UART);
    }
    // enable IRQ
    __set_PRIMASK(primask);     // Restore previous interrupt state
    // mutex unlock
    return n;
}

uint16_t uart_send_string(char *str) {
	// disable IRQ
    uint32_t primask;
    primask = __get_PRIMASK();  // Save current interrupt state
    __disable_irq();            // Disable all interrupts

    // later use mutex lock
    uint16_t n;
    for(n = 0; str[n] != 0; n++) {
    	if(fifo_try_append(&uart_tx_fifo) == false) {
			// fifo is full, stop here
			break;
		}
		((uint8_t *)(uart_tx_fifo.data))[uart_tx_fifo.wr_proc] = str[n];
		fifo_finalize_append(&uart_tx_fifo);
    }

    if(fifo_is_empty(&uart_tx_fifo) == false) {
        // fifo is not empty, start transmitting now
    	LL_USART_EnableIT_TXE(LOG_UART);
    }
    // enable IRQ
    __set_PRIMASK(primask);     // Restore previous interrupt state
    // mutex unlock
    return n;
}

void uart_send_string_blocking(char *str) {
	while(*str != '\0') {
		while(!LL_USART_IsActiveFlag_TXE(LOG_UART));
		LL_USART_TransmitData8(LOG_UART, *str);
		str++;
	}
}

/* USER CODE END 1 */

