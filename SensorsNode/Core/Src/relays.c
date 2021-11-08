#include "relays.h"

uint8_t _relays[4] = {Relay1_Pin, Relay2_Pin , Relay1_Pin, Relay2_Pin};

void Relays_Init()
{
    HAL_GPIO_WritePin(Relay_Port, Relay1_Pin|Relay2_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = Relay1_Pin|Relay2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Relay_Port, &GPIO_InitStruct);
}

void Relays_SetState(uint8_t relay, bool state)
{
	//HAL_GPIO_WritePin(Relay_Port, Relay1_Pin, state);
}

#define Relays_Pin Relay2_Pin // |Relay1_Pin
void Relays_Update(RTC_TimeTypeDef *sTime)
{
	if (sTime->Hours < 7 || sTime->Hours > 21)
	{
		HAL_GPIO_WritePin(Relay_Port, Relays_Pin, 0);
		return;
	}

	if (sTime->Hours < 8 || sTime->Hours > 20)
	{
		HAL_GPIO_WritePin(Relay_Port, Relays_Pin, 1);
		return;
	}

	if (sTime->Minutes >= 12 && sTime->Minutes <= 23
	||  sTime->Minutes >= 36 && sTime->Minutes <= 47)
	{
		HAL_GPIO_WritePin(Relay_Port, Relays_Pin, 1);
		return;
	}

	HAL_GPIO_WritePin(Relay_Port, Relays_Pin, 0);
}
