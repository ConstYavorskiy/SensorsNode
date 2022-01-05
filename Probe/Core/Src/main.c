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
#include "can.h"
#include "i2c.h"
// #include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TMP75/TMP75.h"
#include "Si7021/Si7021.h"
#include "MS5837/MS5837.h"
#include "Sensors/CANSensor.h"
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

static CAN_TxHeaderTypeDef TxHeader;
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t TxData[8] = {0,};
static uint8_t RxData[8] = {0,};
static uint32_t TxMailbox = 0;
static float* dTX = (float*)TxData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool has_TMP75 = false, has_Si1132 = false, has_Si7021 = false, has_MS5837 = false;
static float tmp = 0.0, si_humidity = 0.0, si_temperature = 0.0, ms_temperature = 0.0, ms_pressure = 0.0, ms_depth = 0.0;

static MS5837_t dataMS5837;

static void CAN_SendSensorData(uint16_t sid, float value)
{
  dTX[0] = value;
  TxHeader.StdId = sid;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  // MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  has_TMP75 = TMP75_Init(&hi2c1, 0b1001000);
  has_Si7021 = Si7021_Init(&hi2c1);
  has_MS5837 = MS5837_Init(&hi2c1, &dataMS5837);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  TxHeader.StdId = 0x0100;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA; // CAN_RTR_REMOTE
  TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = 0;

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

  while (1)	{
	HAL_GPIO_WritePin(LED_Port, LED_0, 0);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_Port, LED_0, 1);

	if (has_TMP75) {
	  tmp = TMP75_Read_Temp();
	  CAN_SendSensorData(CAN_TMP75, tmp);
	}

	if (has_Si7021) {
	  if (Si7021_ReadBoth(&si_humidity, &si_temperature))
	  {
		CAN_SendSensorData(CAN_Si7021_Temp, si_temperature);
		CAN_SendSensorData(CAN_Si7021_Humi, si_humidity);
	  }
	}

	if (has_MS5837) {
	  if (MS5837_30BA_Calc(&dataMS5837))
	  {
		ms_temperature = dataMS5837.temperature;
		ms_pressure = dataMS5837.pressure;
		ms_depth = MS5837_Depth(&dataMS5837);
		//ms_pressure = MS5837_Altitude(&dataMS5837);

		CAN_SendSensorData(CAN_MS5837_Temp, ms_temperature);
		CAN_SendSensorData(CAN_MS5837_Press, ms_pressure);
		CAN_SendSensorData(CAN_MS5837_Depth, ms_depth);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        HAL_GPIO_TogglePin(LED_Port, LED_1);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_WritePin(LED_Port, LED_2, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LED_Port, LED_2, 0);
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

