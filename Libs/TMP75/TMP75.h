#ifndef TMP75_H_
#define TMP75_H_

#include "main.h"
#include "Sensors/I2CSensor.h"

/*=============================================*/
#define TMP75_ADDR	0x90
// #define TMP75_ADDR	0x92
// #define TMP75_ADDR	0x94
// #define TMP75_ADDR	0x96
// #define TMP75_ADDR	0x98
// #define TMP75_ADDR	0x9A
// #define TMP75_ADDR	0x9C
// #define TMP75_ADDR	0x9E

/*=============================================*/
bool TMP75_Init(I2C_HandleTypeDef *hi2c, uint16_t addr);
void TMP75_OneShot_Temp(void);
float TMP75_Read_Temp();

#endif
