#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"


void Error_Handler(void);


#define LED_0 GPIO_PIN_0
#define LED_1 GPIO_PIN_1
#define LED_2 GPIO_PIN_2
#define LED_Port GPIOB

#define USB_Connect_Pin GPIO_PIN_4
#define USB_Connect_Port GPIOB


#endif
