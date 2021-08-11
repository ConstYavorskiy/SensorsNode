#ifndef __RELAYS_H__
#define __RELAYS_H__

#include "stm32f1xx_hal.h"
#include "stdbool.h"

#define Relay1_Pin GPIO_PIN_2
#define Relay2_Pin GPIO_PIN_3
#define Relay_Port GPIOA

void Relays_Init();
void Relays_Update(RTC_TimeTypeDef *sTime);
void Relays_SetState(uint8_t relay, bool state);

#endif /* __RELAYS_H__ */
