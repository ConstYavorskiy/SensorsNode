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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AT45DBxx/AT45DBxx.h"
#include "AT45DBxx/AT45DBxxConfig.h"

#include "ili9341/ili9341.h"
//#include "ili9341/ili9341_touch.h"
#include "ili9341/fonts.h"

#include "BME280/bme280.h"
#include "Si7021/Si7021.h"
#include "Si1132/Si1132.h"

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
BMP280_HandleTypedef bmp280;


static uint32_t counter = 0;
static uint16_t encoder = 0;
static uint16_t push = 0;
static bool initialized = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  AT45dbxx_Init();


  ILI9341_Unselect();
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_1;
	bmp280.i2c = &hi2c2;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_Delay(100);
	}

	rst_Si7021();
	Si1132_init(&hi2c2, SI1132_SLAVE_ADDRESS);


  // USB Connect
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	ILI9341_WriteString(3, 3, "00:00:00 00.00", Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
	ILI9341_WriteNumSigns(3, 3, sTime.Hours, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
	ILI9341_WriteNumSigns(51, 3, sTime.Minutes, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
	ILI9341_WriteNumSigns(99, 3, sTime.Seconds, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
	ILI9341_WriteNumSigns(147, 3, sDate.Date, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
	ILI9341_WriteNumSigns(195, 3, sDate.Month, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);

	ILI9341_WriteString(3, 63,  "T:00 H:00 P:000000", Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
	ILI9341_WriteString(3, 93, "T:00 H:00", Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
	ILI9341_WriteString(3, 123, "IR: 000000", Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
	ILI9341_WriteString(3, 153, "Vis:000000", Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
	ILI9341_WriteString(3, 183, "UV: 000000", Font_11x18, ILI9341_CYAN, ILI9341_BLACK);

	initialized = true;
	bool success = false;
	float si_humidity, si_temperature, bs_humidity, bs_temperature, bs_pressure;
	int bs_t = 0, si_t = 0, to = 0, bs_h = 0, si_h = 0, bs_p = 0, co2 = 0, tVOC = 0;
	uint32_t si_ir = 0, si_vis = 0, si_uv = 0;
  while (1)
  {
		uint16_t enc = TIM2->CNT;
		ILI9341_WriteNumSigns(3, 33, enc, 3, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);
		ILI9341_WriteNumSigns(60, 33, push, 3, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		uint8_t seconds = sTime.Seconds;
		ILI9341_WriteNumSigns(99, 3, seconds, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);

		if (seconds == 0) {
			ILI9341_WriteNumSigns(3, 3, sTime.Hours, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(51, 3, sTime.Minutes, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(147, 3, sDate.Date, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(195, 3, sDate.Month, 2, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
		}

		success = true;
		if (bmp280_read_float(&bmp280, &bs_temperature, &bs_pressure, &bs_humidity)) {
			bs_t = (int) bs_temperature;
			bs_p = (int) bs_pressure;
			bs_h = (int) bs_humidity;
		} else {
			success &= false;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_Delay(100);
		}

		if (r_both_Si7021(&si_humidity, &si_temperature) == 0) {
			si_t = (int) si_temperature;
			si_h = (int) si_humidity;
		} else {
			success &= false;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_Delay(100);
		}

		if (Si1132_read(&si_ir, &si_vis, &si_uv)) {

		} else {
			success &= false;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_Delay(100);
		}

		if (success) {
			ILI9341_WriteNumSigns(25, 63, bs_t, 2, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(80, 63, bs_h, 2, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(135, 63, bs_p, 6, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);

			ILI9341_WriteNumSigns(25, 93, si_t, 2, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(80, 93, si_h, 2, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);

			ILI9341_WriteNumSigns(47, 123, si_ir, 6, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(47, 153, si_vis, 6, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
			ILI9341_WriteNumSigns(47, 183, si_uv, 6, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
		}

		HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief This function handles RTC global interrupt.
 */
void RTC_IRQHandler(void) {
	HAL_RTCEx_RTCIRQHandler(&hrtc);
	if (!initialized) {
		return;
	}

	counter++;
	uint8_t state = counter % 3;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, state == 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, state == 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, state == 2);
}

void EXTI2_IRQHandler() {
	/* Clear Pending bit */
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
	EXTI->PR = (1uL << (EXTI_LINE_2 & EXTI_PIN_MASK));
	if (!initialized) {
		return;
	}

	//check pin state
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) {
		//push = 0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		push++;
	} else {

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
