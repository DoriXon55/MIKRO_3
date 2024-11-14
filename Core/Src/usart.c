/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
typedef struct {
	uint8_t* buffer;
	uint32_t readIndex;
	uint32_t writeIndex;
	uint32_t mask;
}ring_buffer;

ring_buffer rxRingBuffer;
ring_buffer txRingBuffer;
uint8_t USART_TxBuf[TX_BUFFER_SIZE];
uint8_t USART_RxBuf[RX_BUFFER_SIZE];

extern void FrameRd();

void ring_buffer_setup(ring_buffer* rb, uint8_t* buffer, uint32_t size)
{
	rb->buffer = buffer;
	rb->readIndex = 0;
	rb->writeIndex = 0;
	rb->mask = size - 1; // zakładając, że zmienna size jest potęgą 2
}

uint8_t USART_kbhit(){
	if(rxRingBuffer.writeIndex == rxRingBuffer.readIndex){
		return 0;
	}else{
		return 1;
	}
}

int16_t USART_getchar() {
    if (rxRingBuffer.writeIndex != rxRingBuffer.readIndex) {
        int16_t tmp = USART_RxBuf[rxRingBuffer.readIndex];
        rxRingBuffer.readIndex = (rxRingBuffer.readIndex + 1) % rxRingBuffer.mask;
        return tmp;
    }
    return -1; // Buffer empty
}

uint8_t USART_getline(char *buf){
	static uint8_t buffer[128];
	static uint8_t index = 0;
	int i;
	uint8_t ret;

	// dopóki są dane w buforze
	while(USART_kbhit()){
		buffer[index] = USART_getchar(); //odczyt znaku
		if(((buffer[index] == 0x0A)||(buffer[index] == 0x0D))) //napotyka koniec wiersza
		{
			buffer[index] = 0;
			for(i = 0;i <= index; i++){
				buf[i] = buffer[i]; //kopiowanie zawartości buffera do buf
			}
			ret = index;
			index = 0;
			return ret; //zwracanie długości linii
		}else{
			index++;
			if(index >= sizeof(buffer)) index = 0; //jeżeli nie ma końca znaku to zawijamy
		}
	}
	return 0;
}
void USART_send(uint8_t message[]){

    uint16_t i,idx = txRingBuffer.writeIndex;


    for(i=0; message[i] != '\0'; i++){ // przenosimy tekst z wywołania funkcji USART_send do tablicy BUF_TX[]

            USART_TxBuf[idx] = message[i];
            idx++;
            if(idx >= TX_BUFFER_SIZE) idx = 0;
            if(idx == txRingBuffer.readIndex) txRingBuffer.readIndex++;
            //if(busy_TX >= sizeof(BUF_TX)) busy_TX=0;
            if(txRingBuffer.readIndex >= TX_BUFFER_SIZE) txRingBuffer.readIndex=0;

    } // cały tekst ze zmienne message[] znajduje się już teraz w BUF_TX[]

        __disable_irq(); //wyłączamy przerwania, bo poniżej kilka linii do zrobienia

        if (txRingBuffer.readIndex == txRingBuffer.writeIndex)
        {
            txRingBuffer.writeIndex = idx;
            uint8_t tmp = USART_TxBuf[txRingBuffer.readIndex];
            txRingBuffer.readIndex++;
            if(txRingBuffer.readIndex >= TX_BUFFER_SIZE) txRingBuffer.readIndex = 0;
            HAL_UART_Transmit_IT(&huart2, &tmp, 1);

            //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
        else
        {
        	txRingBuffer.readIndex = idx;
        }

        __enable_irq(); //ponownie aktywujemy przerwania
}
void USART_fsend(char* format,...){
	char tmp_rs[128];
	int i;
	volatile int idx;
	va_list arglist;
	  va_start(arglist,format);
	  vsprintf(tmp_rs,format,arglist);
	  va_end(arglist);
	  idx=txRingBuffer.writeIndex;
	  for(i=0;i<strlen(tmp_rs);i++){
		  USART_TxBuf[idx]=tmp_rs[i];
		  idx++;
		  if(idx >= TX_BUFFER_SIZE)idx=0;
	  }
	  __disable_irq();//wyłączamy przerwania
	  if((txRingBuffer.writeIndex==txRingBuffer.readIndex)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){//sprawdzic dodatkowo zajetosc bufora nadajnika
		  txRingBuffer.writeIndex=idx;
		  uint8_t tmp=USART_TxBuf[txRingBuffer.readIndex];
		  txRingBuffer.readIndex++;
		  if(txRingBuffer.readIndex >= TX_BUFFER_SIZE)txRingBuffer.readIndex=0;
		  HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	  }else{
		  txRingBuffer.writeIndex=idx;
	  }
	  __enable_irq();
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
   if(huart==&huart2){
	   if(txRingBuffer.writeIndex!=txRingBuffer.readIndex){
		   uint8_t tmp = USART_TxBuf[txRingBuffer.readIndex];
		   txRingBuffer.readIndex++;
		   if(txRingBuffer.readIndex >= TX_BUFFER_SIZE) txRingBuffer.readIndex=0;
		   HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	   }
   }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 if(huart==&huart2){
		 rxRingBuffer.writeIndex++;
		 if(rxRingBuffer.writeIndex >= RX_BUFFER_SIZE) rxRingBuffer.writeIndex=0;
		 HAL_UART_Transmit_IT(&huart2,&USART_RxBuf[rxRingBuffer.writeIndex - 1],1);
		 HAL_UART_Receive_IT(&huart2,&USART_RxBuf[rxRingBuffer.writeIndex],1);
		 FrameRd();
	 }
}
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  ring_buffer_setup(&rxRingBuffer, USART_RxBuf, RX_BUFFER_SIZE);
  ring_buffer_setup(&txRingBuffer, USART_TxBuf, TX_BUFFER_SIZE);
  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[0], 1);
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
