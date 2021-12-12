#ifndef TMP75_H_
#define TMP75_H_

#include "main.h"
#include "I2CSensor/I2CSensor.h"

/*=============================================*/
#define TMP75_ADDR	0x90

/*=============================================*/
void TMP75_Init(I2C_HandleTypeDef *hi2c, uint16_t addr);
void TMP75_OneShot_Temp(void);
float TMP75_Read_Temp();

#endif
