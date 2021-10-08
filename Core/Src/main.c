/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "fatfs_sd.h"
#include "string.h"
#include "inttypes.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

FATFS fs; // file system
FIL fil; // file
FILINFO filinfo;
FRESULT fresult; // to store the result
char buffer[1024]; // to store data

char data[7];
char dataStr[100] = "";

UINT br, bw; // file read/write count

/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

uint16_t value_adc[10];
uint32_t value_dac = 500;
float adcVal = 0;
float voltage = 0;
float resistance = 0;
float storedVal = 0;

volatile uint8_t DACout;

/* Button debounce */
uint32_t previous_millis = 0;
uint32_t current_millis = 0;
uint32_t counter = 0;
uint32_t measurement = 0;
uint8_t button_pressed = 0;
uint8_t button_state = 0;

/* custom character */
// byte is not defined here by default
typedef unsigned char byte;

// make some custom characters:
byte ohm[8] = {
  0x0E,
  0x11,
  0x11,
  0x11,
  0x0A,
  0x0A,
  0x1B,
  0x00
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* to send the data to the uart */
void send_uart(char *string)
{
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t *)string, len, 2000); // transmit in blocking mode
}

/* to find the size of data in the buffer */
int bufsize(char *buf)
{
	int i = 0;
	while (*buf++ != '\0') i++;
	return i;
}

/* clear buffer */
void bufclear(void)
{
	for(int i = 0; i < 1024; i++)
	{
		buffer[i] = '\0';
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
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_DAC_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	/* Mount SD Card */
	fresult = f_mount(&fs, "", 0);
	//if(fresult != FR_OK) send_uart("error in mounting SD CARD...\n");
	//else send_uart("SD CARD mounted successfully...\n");

	/************* Card Capacity details *************/

	/* check free space */
	/*f_getfree("", &fre_clust, &pfs);

	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Total Size: \t%lu\n", total);
	send_uart(buffer);
	bufclear();
	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Free Space: \t%lu\n", free_space);
	send_uart(buffer);
	 */

	/************* The following operation is using PUTS and GETS *************/

	/* Open file to write/create a file if it doesn't exist
	fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	 Writing text
	fresult = f_puts("This data is from the First FILE\n\n", &fil);
	 Close file
	fresult = f_close(&fil);
	send_uart("File1.txt created and the data is written \n");
	 Open file to read
	fresult = f_open(&fil, "file1.txt", FA_READ);
	 Read file info so we can get the file size
	fresult = f_stat("file1.txt", &filinfo);
	 Read string from the file
	f_gets(buffer, filinfo.fsize, &fil);
	send_uart(buffer);
	 Close file
	f_close(&fil);
	bufclear();*/


	/************* The following operation is using f_write and f_read *************/

	/* Create second file with read write access and open it */
	fresult = f_open(&fil, "Results.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	/* Writing text */
	strcpy(buffer, "Measurement, Voltage, Resistance \n");

	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);
	//bw is the pointer to the counter for the number of bytes written
	//send_uart("Results.txt created and data is written\n");
	/* Close file */
	f_close(&fil);
	// clearing buffer to show that result obtained is from the file
	bufclear();
	/* Open second file to read */
	fresult = f_open(&fil, "Results.txt", FA_READ);
	/* Read file info so we can get the file size */
	fresult = f_stat("Results.txt", &filinfo);
	f_read (&fil, buffer, filinfo.fsize, &br);
	//br is the pointer to the count variable for the number of bytes to read from the file
	//send_uart(buffer);
	/* Close file */
	f_close(&fil);
	bufclear();

	// initialize the library by associating any needed LCD interface pin
	LiquidCrystal(GPIOB, GPIO_PIN_12, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_3);

	// create a new character
	createChar(0, ohm);

	setCursor(0, 0);
	print("POWER ON");
	HAL_Delay(5000);
	setCursor(0, 0);
	print("PRESS TO");
	setCursor(0, 1);
	print(" START  ");

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&value_adc, 10);

	char str[10];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		while(counter >= 1)
		{
			//check timing van DAC op ossiloskoop vir al die button_states
			if(DACout == 1)
			{
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2482);
			}
			else
			{
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
			}
			HAL_ADC_Start(&hadc1);

			adcVal = value_adc[0];

			if(value_adc[0] <= 2000)
			{
				__NOP();
			}


			voltage = adcVal * 3.3 / 4095;
			resistance = (voltage / 0.00001) / 30.5; //gain van +-30.5

			if(button_state == 1)
			{
				// set the cursor to column 0, line 0
				setCursor(0, 0);
				sprintf(str, "%.0f     ", resistance);
				print(str);
				setCursor(0, 1);
				write((0));
				print("      ");
			}

			if(button_state == 2)
			{
				storedVal = resistance;
				setCursor(0, 0);
				sprintf(str, "%lu:", measurement);
				print(str);
				sprintf(str, "%.0f", storedVal);
				print(str);
				setCursor(0, 1);
				write((0));

				/************* Updating an existing file *************/

				dataStr[0] = 0;

				sprintf(data, "%lu", measurement);
				strcat(dataStr, data);
				strcat(dataStr, ", ");

				sprintf(data, "%0.3f", voltage);
				strcat(dataStr, data);
				strcat(dataStr, ", ");

				sprintf(data, "%0.3f", resistance);
				strcat(dataStr, data);
				strcat(dataStr, "\n");

				/* Open the file with write access */
				fresult = f_open(&fil, "Results.txt", FA_OPEN_ALWAYS | FA_WRITE);
				/* Read file info so we can get the file size */
				fresult = f_stat("Results.txt", &filinfo);
				/* Move to offset to the end of the file */
				fresult = f_lseek(&fil, filinfo.fsize);
				/* Write the string to the file */
				fresult = f_puts(dataStr, &fil);
				/* Close file */
				f_close(&fil);
				/* Open to read the file */
				fresult = f_open(&fil, "Results.txt", FA_READ);
				/* Read file info so we can get the file size */
				fresult = f_stat("Results.txt", &filinfo);
				/* Read string from the file */
				f_read(&fil, buffer, filinfo.fsize, &br);
				//send_uart(buffer);
				/* Close file */
				f_close(&fil);
				bufclear();

				button_state = 0;
			}
		}

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RNW_Pin|EN_Pin|RS_Pin|DB6_Pin
                          |DB5_Pin|DB4_Pin|DB7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RNW_Pin EN_Pin RS_Pin DB6_Pin
                           DB5_Pin DB4_Pin DB7_Pin */
  GPIO_InitStruct.Pin = RNW_Pin|EN_Pin|RS_Pin|DB6_Pin
                          |DB5_Pin|DB4_Pin|DB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
	__NOP();
}
//conversion complete callback in non-blocking mode
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	__NOP();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	current_millis = HAL_GetTick();
	if (GPIO_Pin == GPIO_PIN_13 && (current_millis - previous_millis > 10))
	{
		counter++;
		button_pressed = 1;
		previous_millis = current_millis;

		if(button_pressed == 1 && button_state == 0)
		{
			button_state = 1;
			button_pressed = 0;
		}

		if(button_pressed == 1 && button_state == 1)
		{
			measurement++;
			button_state = 2;
			button_pressed = 0;
		}

	}
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
