#include "TMP75.h"

#define TMP75_Temperature	0b00000000
#define TMP75_Configuration	0b00000001

#define TMP75_TEMP_Resolution_12bit		0.0625
#define TMP75_TEMP_Resolution_11bit		0.125
#define TMP75_TEMP_Resolution_10bit		0.25
#define TMP75_TEMP_Resolution_9bit		0.5

static I2CSensor_HandleTypedef hTMP75;

bool TMP75_Init(I2C_HandleTypeDef *hi2c, uint16_t addr) {

	hTMP75.i2c = hi2c;
	hTMP75.addr = addr;
	hTMP75.addr_shifted = (addr << 1);

	/*[OS][Res][FQ][POL][TM][SD]
	    0   00  00   0    0   0  */

	uint8_t cfg = 0b01100000;
	return I2CSensor_write8(&hTMP75, TMP75_Configuration, cfg);
}

void TMP75_OneShot_Temp(void) {
	//One-Shot register = 0x04 and write any value to start a conversion
	uint8_t cfg = 0b11100001;
	I2CSensor_write8(&hTMP75, TMP75_Configuration, cfg);
}

float TMP75_Read_Temp() {
	uint8_t buffer[2];
	I2CSensor_read(&hTMP75, TMP75_Temperature, buffer, 2);
	int16_t temp = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]) >> 4;
	return (float)temp * TMP75_TEMP_Resolution_12bit;
}
