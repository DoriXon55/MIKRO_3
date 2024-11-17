/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	Waiting,
	Command,
	Argument,
	number,
	End
} FrameDetection;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t znak;
uint16_t komunikat, dlugosc;


volatile int ld2State = 0;
volatile uint16_t slow = 0;
uint8_t tempINDX = 0;
char NumArr[128]; // LED[BLINK,2] --> "2"
char cmd[128]; // komendy przesłane przez usera (np LED, INSERT)
char CMDarg[128]; // argument komenty np "ON" --> LED[ON]
float nr = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void FrameRd()
{
	static FrameDetection detection = Waiting;

	// P1 - "10101010"
	// P2 - "123.234.123"
	// p3 - LICZBA DANYCH (7)
	// p4 - dANE ("LED[ON]")
	// P5 - CRC

	// cmd[] <-- łapać "LED", "INSERT"
	// cmdARG[] <-- łapie argumenty "ON", "OFF", "BLINK"
	// NumArr[] <-- łapie cyfry "LED[BLINK,5]"

	int16_t sign = USART_getchar(); // [
	if (sign < 0) return;

	switch (detection){
	case Waiting:
		if (sign == 'L' || sign =='I') detection = Command;
		//INSERT[Delay,5]
		// LED[ON]
	case Command:
		if (sign == '['){
			tempINDX = 0;
			detection = Argument;
		}
		else{
			cmd[tempINDX] = sign; //LED
			tempINDX++;
		}
		break;
	case Argument:
		if (sign == ']'){  /// LED[BLINK,5]
			detection = End;
		}
		else if (sign == ',') {
			tempINDX = 0;
			detection = number;
			break;
		}
		else{
			CMDarg[tempINDX] = sign; // ON, OFF, BLINK,5
			tempINDX++;
			break;
		}
	case number:
		if (sign == ']'){
			nr = atoi(NumArr);
			detection = End;
		}
		else{
			NumArr[tempINDX] = sign; //5
			tempINDX++;
		}
		break;
	case End:

		// cmd[] = LED
		// cmdARG[] = BLINK
		// NumaArr[] = 5

		if (strcmp(cmd, "LED") == 0){
			if (strcmp(CMDarg, "ON") == 0) ld2State = 1;
			else if (strcmp(CMDarg, "OFF") == 0) ld2State = 2;
			else if (strcmp(CMDarg, "BLINK") == 0) {slow = 1000 / nr; ld2State = 3;}
		}else if (strcmp(cmd, "INSERT") == 0) {
			if (strcmp(CMDarg, "Delay") == 0) {ld2State = 4; slow = nr;}
		}
		//USART_fsend("\r\r\n\n\r");
		USART_fsend("\r\r\n\n");
		tempINDX = 0;
		nr = 0;
		memset(cmd, '\0', sizeof(cmd));
		memset(CMDarg, '\0', sizeof(CMDarg));
		memset(NumArr, '\0', sizeof(NumArr));
		detection = Waiting;
		break;
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
  /* USER CODE BEGIN 2 */


  USART_fsend("START %03d\r\n",72);
  USART_fsend("START1 0x%04X\r\n",72);
  USART_fsend("START2 %03d\r\n",72);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Callback do odbioru bajtów - dodaje każdy odebrany bajt do `rxRingBuffer`






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
  while (1)
  {
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
