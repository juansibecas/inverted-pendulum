/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "main.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART3){
	 uint8_t data = UART3_rxBuffer[0];
	 if(uart_index >= SIZE_COM) uart_index = 0;
	 switch(data){
	 case ':':  	//Command start
		 uart_index = 0;
		 break;
	 case '\r':  	//End of command char
	 case ';':
		 command[uart_index] = 0;
		 Read_Command();
		 break;
	 case 8:  		//Backspace
		 if (uart_index) uart_index--;
		 break;
	 default:
		 command[uart_index++] = data;
		 break;
	 }
	 HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 1);
	}
}

void Read_Command(){
	float aux = 0;
	//start the rise up sequence
	if(strstr(command, "start")){
		printf("Start\r\n");
		startTime = HAL_GetTick();
		prevTime = startTime;
		elapsedTime = 0;
		state = rise_up;
		run = on;

		if (filteredAngle > 0.0){
			is_angle_positive = 1;
		} else{
			is_angle_positive = 0;
		}

	} else if(strstr(command, "stop")){
		printf("Stop\r\n");
		run = off;

	//write values to terminal
	} else if(strstr(command, "read")){
		printf("accX = %f\r\n"
				"accY = %f\r\n"
				"accZ = %f\r\n"
				"GyX = %f\r\n"
				"GyY = %f\r\n"
				"GyZ = %f\r\n"
				"accPitch = %f\r\n"
				"fAngle = %f\r\n", (float)AccX, (float)AccY, (float)AccZ, (float)GyX,
									(float)GyY, (float)GyZ, (float)accPitch, (float)filteredAngle);

	//change PID constants
	} else{
		switch(command[0]){
			case 'P':
			case 'p':
				aux = atof(&command[1]);
				kp = aux;
				printf("Change P\r\n");
				break;
			case 'I':
			case 'i':
				aux = atof(&command[1]);
				ki = aux;
				printf("Change I\r\n");
				break;
			case 'D':
			case 'd':
				aux = atof(&command[1]);
				kd = aux;
				printf("Change D\r\n");
				break;
			default:
				break;

		}
	}
}

int __io_putchar(int msg){		//UART3 as stdio
	 uint8_t c[1];
	 c[0] = msg & 0x00FF;
	 HAL_UART_Transmit(&huart3, &*c, 1, 100);
	 return c[0];
}

/* USER CODE END 1 */
