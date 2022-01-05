#ifndef TMP75_H_
#define TMP75_H_

#include "main.h"
#include "Sensors/I2CSensor.h"

/*=============================================*/
#define TMP75_ADDR	0x90

/*=============================================*/
bool TMP75_Init(I2C_HandleTypeDef *hi2c, uint16_t addr);
void TMP75_OneShot_Temp(void);
float TMP75_Read_Temp();

#endif
