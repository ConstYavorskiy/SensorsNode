#ifndef MS5837_H_
#define MS5837_H_

#include "main.h"
#include "I2CSensor/I2CSensor.h"

//Definitions:
#define MS5837_ADDR               0x76

#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

//Models:
#define MS5837_30BA               0x00
#define MS5837_02BA				  0x01


// Default seawater    - 1029  kg/m3
// Default freshwater  - 998.2 kg/m3 at 20°C
#define FluidDensity 999.2;


#define MS5837_Delay()  HAL_Delay(20);

//Values read from MS5837
typedef struct {
	int32_t TEMP;
	int32_t P;
	uint16_t C[7];
	uint32_t D1;
	uint32_t D2;
} MS5837_values_t;

typedef struct  {
	//MS5837 model(Default MS5837_30BA)
	uint8_t model;

	//Fluid density (Default 1029)
	float fluidDensity;

	//Pressure unit (Default mBar)
	float temperature;

	float pressure;

	MS5837_values_t val;
} MS5837_t;

bool MS5837_Init(I2C_HandleTypeDef *hi2c, MS5837_t *MS5837);

bool MS5837_30BA_Calc(MS5837_t *MS5837);
bool MS5837_02BA_Calc(MS5837_t *MS5837);
float MS5837_Depth(MS5837_t *MS5837);
float MS5837_Altitude(MS5837_t *MS5837);

#endif /* MS5837_H_ */
