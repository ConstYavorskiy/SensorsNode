/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "relays.h"

#include "AT45DBxx/AT45DBxx.h"
#include "AT45DBxx/AT45DBxxConfig.h"

#include "ili9341/ili9341.h"
//#include "ili9341/ili9341_touch.h"
#include "ili9341/fonts.h"
#include "ili9341/tsc.h"

#include "BME280/bme280.h"
#include "Si7021/Si7021.h"
#include "Si1132/Si1132.h"

#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>



static BMP280_HandleTypedef bmp280;
static TSC_ENVIRONMENT TSC_Env;
static TSC_STATE TSC_State;

static uint16_t counter = 0, fps = 0, iters = 0;
static bool initialized = false, has_BME280 = false, has_Si1132 = false, has_Si7021 = false, has_TSC = true, has_AT45db = false, has_USB_Connection = false;


static void SystemClock_Config(void);

static void InitializeDevices()
{
  // AT45db
 /* MX_SPI1_Init();
  has_AT45db = AT45dbxx_Init();
*/
  // ILI9341
  MX_SPI2_Init();
  ILI9341_Init();
  ILI9341_FillScreen(BLACK);

  MX_I2C2_Init();
  if (has_TSC) {
	TSC_init(&hi2c2);
  }

  // BME280
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_1;
  bmp280.i2c = &hi2c2;
  has_BME280 = bmp280_init(&bmp280, &bmp280.params);

  // Si1132
  has_Si1132 = Si1132_init(&hi2c2, SI1132_SLAVE_ADDRESS);

  // Si7021
  if (has_Si7021){
	  Si7021_Init(&hi2c2);
  }

  Relays_Init();
}

static void InitializeUI()
{
  ILI9341_WriteString(3, 3, "00:00:00 00.00", Font_16x26, GREEN, BLACK);
  ILI9341_WriteString(3, 63, "T        H     P" , Font_11x18, CYAN, BLACK);
  ILI9341_WriteString(3, 153, "IR: 00000", Font_11x18, CYAN, BLACK);
  ILI9341_WriteString(3, 183, "Vis:00000", Font_11x18, CYAN, BLACK);
  ILI9341_WriteString(3, 213, "UV: 00000", Font_11x18, CYAN, BLACK);

  ILI9341_WriteString(3, 33, "V", Font_11x18, YELLOW, BLACK);
  ILI9341_WriteString(12, 40, "IN", Font_7x10, YELLOW, BLACK);
  ILI9341_WriteString(80, 33, "V", Font_11x18, YELLOW, BLACK);
  ILI9341_WriteString(88, 40, "BAT", Font_7x10, YELLOW, BLACK);
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();
  MX_RTC_Init();
  MX_GPIO_Init();

  //MX_TIM2_Init();
  //MX_I2C1_Init();
  //MX_USB_DEVICE_Init();

  InitializeDevices();

  // USB Connect
  if (has_USB_Connection) {
    HAL_Delay(100);
    HAL_GPIO_WritePin(USB_Connect_Port, USB_Connect_Pin, GPIO_PIN_SET);
  }

  RTC_TimeTypeDef sTime = { 0 };
  RTC_DateTypeDef sDate = { 0 };

  bool success = false;
  float si_humidity, si_temperature, bs_humidity, bs_temperature, bs_pressure;
  int bs_t = 0, si_t = 0, to = 0, bs_h = 0, si_h = 0, bs_p = 0, co2 = 0, tVOC = 0;
  uint32_t si_ir = 0, si_vis = 0, si_uv = 0;

  char buffer_tx[64];
  uint8_t buffer_tx_len = 0;

  InitializeUI();

  while (1)
  {
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	uint8_t seconds = sTime.Seconds;
	ILI9341_WriteNumSigns(99, 3, seconds, 2, Font_16x26, GREEN, BLACK);
	ILI9341_WriteNumSigns(170, 33, fps, 2, Font_11x18, YELLOW, BLACK);

	if (!initialized || seconds % 10 == 0) {
		ILI9341_WriteNumSigns(3,   3, sTime.Hours, 2, Font_16x26, GREEN, BLACK);
		ILI9341_WriteNumSigns(51,  3, sTime.Minutes, 2, Font_16x26, GREEN, BLACK);
		ILI9341_WriteNumSigns(147, 3, sDate.Date, 2, Font_16x26, GREEN, BLACK);
		ILI9341_WriteNumSigns(195, 3, sDate.Month, 2, Font_16x26, GREEN, BLACK);

		if (!initialized) {
			initialized = true;
		}
	}

	Relays_Update(&sTime);

	if (has_TSC) {
		TSC_GetEnvironment(&TSC_Env);
		ILI9341_WriteNumSigns(30, 33, TSC_Env.Vin, 3, Font_11x18, YELLOW, BLACK);
		ILI9341_FillRectangle(38, 48, 2, 3, YELLOW);
		ILI9341_WriteNumSigns(112, 33, TSC_Env.Vbat, 3, Font_11x18, YELLOW, BLACK);
		ILI9341_FillRectangle(121, 48, 2, 3, YELLOW);

		ILI9341_WriteNumSigns(22, 63, TSC_Env.Temp0, 2, Font_11x18, YELLOW, BLACK);

		TSC_GetTouchState(&TSC_State);
		if (TSC_State.TouchDetected)
		{
			ILI9341_FillRectangle(TSC_State.X, TSC_State.Y, 3, 3, PINK);
		}
	}

	success = true;
	if (has_BME280) {
		if (bmp280_read_float(&bmp280, &bs_temperature, &bs_pressure, &bs_humidity)) {
			bs_t = (int) bs_temperature;
			bs_p = (int) bs_pressure;
			bs_h = (int) bs_humidity;

			ILI9341_WriteNumSigns(58, 63, bs_t, 2, Font_11x18, CYAN, BLACK);
			ILI9341_WriteNumSigns(120, 63, bs_h, 2, Font_11x18, CYAN, BLACK);
			ILI9341_WriteNumSigns(185, 63, bs_p, 4, Font_11x18, CYAN, BLACK);
		} else {
			success &= false;
			HAL_GPIO_WritePin(LED_Port, LED_2, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_Port, LED_2, GPIO_PIN_RESET);
			HAL_Delay(100);
		}
	}

	if (has_Si7021) {
		if (Si7021_ReadBoth(&si_humidity, &si_temperature)) {
			si_t = (int) si_temperature;
			si_h = (int) si_humidity;

			ILI9341_WriteNumSigns(25, 123, si_t, 2, Font_11x18, CYAN, BLACK);
			ILI9341_WriteNumSigns(80, 123, si_h, 2, Font_11x18, CYAN, BLACK);
		} else {
			success &= false;
			HAL_GPIO_WritePin(LED_Port, LED_2, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_Port, LED_2, GPIO_PIN_RESET);
			HAL_Delay(100);
		}
	}

	if (has_Si1132) {
		if (Si1132_read(&si_ir, &si_vis, &si_uv)) {
			ILI9341_WriteNumSigns(47, 153, si_ir, 5, Font_11x18, CYAN, BLACK);
			ILI9341_WriteNumSigns(47, 183, si_vis, 5, Font_11x18, CYAN, BLACK);
			ILI9341_WriteNumSigns(47, 213, si_uv, 5, Font_11x18, CYAN, BLACK);
		} else {
			success &= false;
			HAL_GPIO_WritePin(LED_Port, LED_2, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_Port, LED_2, GPIO_PIN_RESET);
			HAL_Delay(100);
		}
	}

    if (success) {


/*
			buffer_tx_len = sprintf(buffer_tx, "%02d:%02d:%02d %02d.%02d T:%02d H:%02d P:%06d \r\n",
					sTime.Hours, sTime.Minutes, sTime.Seconds, sDate.Date, sDate.Month, si_t, si_h, bs_p);

			CDC_Transmit_FS((uint8_t*)buffer_tx, buffer_tx_len);
*/
    }

    iters++;
    HAL_Delay(200);
  }
}

static void SystemClock_Config(void)
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

void RTC_IRQHandler(void) {
	HAL_RTCEx_RTCIRQHandler(&hrtc);
	if (!initialized) {
		return;
	}
	fps = iters;
	iters = 0;
	counter++;
	uint8_t state = counter % 3;
	HAL_GPIO_WritePin(LED_Port, LED_0, state == 0);
	HAL_GPIO_WritePin(LED_Port, LED_1, state == 1);
	HAL_GPIO_WritePin(LED_Port, LED_2, state == 2);
}

/*
void EXTI2_IRQHandler() {
	// Clear Pending bit
	//HAL_GPIO_EXTI_IRQHandler(Encoder_Button);
	EXTI->PR = (1uL << (EXTI_LINE_2 & EXTI_PIN_MASK));
	if (!initialized) {
		return;
	}

	//check pin state
	if (HAL_GPIO_ReadPin(Encoder_Port, Encoder_Button)) {
		HAL_GPIO_WritePin(LED_Port, LED_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_Port, LED_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_Port, LED_2, GPIO_PIN_SET);
		push++;
	} else {

	}
}
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

