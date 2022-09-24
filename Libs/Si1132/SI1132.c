#include "Si1132.h"

static I2CSensor_HandleTypedef hSi1132;
static const bool is_indoor_usage = false;

/**
 @brief Search Device
 @retval true device connected
 @retval false device error
 */
static bool Si1132_search() {
	uint8_t device = 0x00;
	I2CSensor_read8(&hSi1132, SI1132_WHO_AM_I_REG, &device);

	if (device == SI1132_DEVICE) {
		return true;
	} else {
		return false;
	}
}

static bool Si1132_readParam(uint8_t p, uint8_t *v)
{
	I2CSensor_write8(&hSi1132, SI1132_COMMAND_REG, SI1132_NOP); // clear the register
	I2CSensor_write8(&hSi1132, SI1132_COMMAND_REG, p | SI1132_PARAM_QUERY);
	return I2CSensor_read8(&hSi1132, SI1132_PARAM_RD_REG, v);
}

static bool Si1132_writeParam(uint8_t p, uint8_t v)
{
/*
	For every write to the Command register, the following sequence is required:
	1. Write 0x00 to Command register to clear the Response register.
	2. Read Response register and verify contents are 0x00.
	3. Write Command value from Table 5 into Command register.
	4. Read the Response register and verify contents are now non-zero. If contents are still 0x00, repeat this step.
 */
	uint8_t response;
	I2CSensor_write8(&hSi1132, SI1132_COMMAND_REG, SI1132_NOP); // clear the register
	HAL_Delay(10);
	I2CSensor_read8(&hSi1132, SI1132_RESPONSE_REG, &response);
	if (response != 0x00)
	{
		return false;
	}

	I2CSensor_write8(&hSi1132, SI1132_PARAM_WR_REG, v);
	I2CSensor_write8(&hSi1132, SI1132_COMMAND_REG, p | SI1132_PARAM_SET);
	HAL_Delay(25);
	I2CSensor_read8(&hSi1132, SI1132_RESPONSE_REG, &response);
	if (response == 0x00)
	{
		return false;
	}

	return true;
}

/**
 @brief Set Config
 */
static void Si1132_configuration() {
	// Reset
	I2CSensor_write8(&hSi1132, 0x22, 0x01);
	HAL_Delay(25);

	uint8_t tx_buff[4] = { 0x7B, 0x6B, 0x01, 0x00 };
	I2CSensor_write(&hSi1132, SI1132_UCOEF0_REG, tx_buff, 4);
	HAL_Delay(25);
	// enable UV, AUX, IR and Visible light sensors
	Si1132_writeParam(SI1132_CHIPLIST_PARAM_OFFSET, SI1132_EN_UV | SI1132_EN_ALS_IR | SI1132_EN_ALS_VIS);

	// Select HW_KEY register
	I2CSensor_write8(&hSi1132, SI1132_HW_KEY_REG, SI1132_HW_KEY_DEFAULT);
	HAL_Delay(25);
	// Rate setting
	I2CSensor_write8(&hSi1132, SI1132_MEASRATE0_REG, 0xff);
	HAL_Delay(25);
	// SET PARAM_WR(ALS_ENCODING)
	Si1132_writeParam(SI1132_ALS_ENCODING_PARAM_OFFSET, SI1132_ALS_VIS_ALIGN | SI1132_ALS_IR_ALIGN);

	/* Visible */
	// SET ALS_VIS_ADC_COUNTER
	Si1132_writeParam(SI1132_ALS_VIS_ADC_COUNTER_PARAM_OFFSET, SI1132_511_ADC_CLOCK);

	// SET ALS_VIS_ADC_GAIN
	Si1132_writeParam(SI1132_ALS_VIS_ADC_GAIN_PARAM_OFFSET, SI1132_1_DIVIDED_ADC_CLOCK);

	// SET ALS_VIS_ADC_MISC
	Si1132_writeParam(SI1132_ALS_VIS_ADC_MISC_PARAM_OFFSET, is_indoor_usage ? SI1132_NORMAL_SIGNAL_RANGE : SI1132_HIGH_SIGNAL_RANGE);


	/* IR */
	// SET ALS_IR_ADC_COUNTER
	Si1132_writeParam(SI1132_ALS_IR_ADC_COUNTER_PARAM_OFFSET, SI1132_511_ADC_CLOCK);

	// SET ALS_IR_ADC_GAIN
	Si1132_writeParam(SI1132_ALS_IR_ADC_GAIN_PARAM_OFFSET, SI1132_1_DIVIDED_ADC_CLOCK);

	// SET ALS_IR_ADC_MISC
	Si1132_writeParam(SI1132_ALS_IR_ADC_MISC_PARAM_OFFSET, is_indoor_usage ? SI1132_NORMAL_SIGNAL_RANGE : SI1132_HIGH_SIGNAL_RANGE);

	// SET ALS_IR_ADCMUX
	Si1132_writeParam(SI1132_ALS_IR_ADCMUX_PARAM_OFFSET, SI1132_ALS_IR_ADCMUX_SMALLIR);


	// SET AUX_ADCMUX
	Si1132_writeParam(SI1132_AUX_ADCMUX_PARAM_OFFSET, SI1132_AUX_ADCMUX_TEMPERATURE);

/*
	The accuracy of UV readings can be improved by using calibration parameters that are programmed into the
	Si1132 at Silicon Labs' production facilities to adjust for normal part-to-part variation. The calibration parameters
	are recovered from the Si1132 by writing Command Register @ address 0x18 with the value 0x12.
	When the calibration parameters are recovered, they show up at I2C registers 0x22 to 0x2D. These are the same
	registers used to report the VIS, IR, and AUX measurements.

	Si1132_writeParam(0x18, 0x12);
*/

	// COMMAND
	I2CSensor_write8(&hSi1132, SI1132_COMMAND_REG, SI1132_COMMAND_ALS_AUTO);
	HAL_Delay(25);
}

/**
 @brief software reset Si1132
 */
static void Si1132_reset() {
	I2CSensor_write8(&hSi1132, SI1132_COMMAND_REG, SI1132_COMMAND_RESET);
	HAL_Delay(10);
}

/**
 @brief Begin Device
 @retval true normaly done
 @retval false device error
 */
bool Si1132_init(I2C_HandleTypeDef *hi2c, uint16_t addr) {
	hSi1132.i2c = hi2c;
	hSi1132.addr = addr;
	hSi1132.addr_shifted = (addr << 1);

	if (Si1132_search()) {
		Si1132_reset();
		Si1132_configuration();
		return true;
	} else {
		return false;
	}
}

bool Si1132_about(uint8_t *part_id, uint8_t *rev_id, uint8_t *seq_id)
{
	uint8_t rx_buff[3];
	I2CSensor_read(&hSi1132, SI1132_WHO_AM_I_REG, rx_buff, 3);
	*part_id = rx_buff[0];
	*rev_id = rx_buff[1];
	*seq_id = rx_buff[2];

	return true;
}

/**
 @brief Read UV
 @param [out] uv rawdata (rawdata/100 -> UV INDEX)
 */
uint16_t Si1132_readUV() {
	uint16_t value;
	I2CSensor_read16_LSBMSB(&hSi1132, SI1132_AUX_DATA_REG, &value);
/*
	AUX_DATA will contain a 16-bit value representing 100 times the sunlight UV Index. Host
	software must divide the results from AUX_DATA by 100.
	value /= 100;
*/
	return value;
}

/**
 @brief Read IR
 @param [out] IR data (lux)
 */
uint16_t Si1132_readIR() {
	uint16_t value;
	I2CSensor_read16_LSBMSB(&hSi1132, SI1132_IR_DATA_REG, &value);
	value = ((value - 250) / 2.44) * 14.5;
	return value;
}

/**
 @brief Read Visible
 @param [out] Visible data (lux)
 */
uint16_t Si1132_readVisible() {
	uint16_t value;
	I2CSensor_read16_LSBMSB(&hSi1132, SI1132_VISIBLE_DATA_REG, &value);
	value = ((value - 256) / 0.282) * 14.5;
	return value;
}

static float f_ir_average = 0, f_vis_average = 0, f_uv_average = 0;
bool Si1132_read(uint32_t *ir, uint32_t *vis, uint32_t *uv)
{
	uint8_t rx_buff[4];
	float f_ir = 0, f_vis = 0, f_uv = 0;
	I2CSensor_read(&hSi1132, SI1132_VISIBLE_DATA_REG, rx_buff, 4);
	f_vis = (float)toU16_LSBMSB(rx_buff);
	f_ir = (float)toU16_LSBMSB(rx_buff + 2);

	f_uv = Si1132_readUV();

/*
	When performing ALS-VIS measurements, the ADC can be programmed to operate in
	high sensitivity operation or high signal range.
	The high signal range is useful in operation under direct sunlight.
	0: Normal Signal Range
	1: High Signal Range (Gain divided by 14.5)


	Visible Photodiode Response (ALS_VIS_ADC_GAIN=0 VIS_RANGE=0)
	Sunlight						— 0.282 ADC counts / lux
	2500K incandescent bulb			— 0.319 ADC counts / lux
	“Cool white” fluorescent		— 0.146 ADC counts / lux
	Infrared LED (875 nm)			— 8.277 ADC counts. m2/W

	Small Infrared Photodiode Response (ALS_IR_ADC_GAIN=0 IR_RANGE=0)
	Sunlight						— 2.44 ADC counts / lux
	2500K incandescent bulb			— 8.46 ADC counts / lux
	“Cool white” fluorescent		— 0.71 ADC counts / lux
	Infrared LED (875 nm)			— 452.38 ADC counts. m2/W
*/

	f_vis = (f_vis > 256.0) ? (f_vis - 256.0) : 0;
	f_ir = (f_ir > 256.0) ? (f_ir - 256.0) : 0;

	if (!is_indoor_usage)
	{
		f_vis *= 14.5;
		f_ir *= 14.5;
	}

	f_vis /= 0.282;
	f_ir /= 2.44;

	if (f_ir_average == 0.0 && f_vis_average == 0)
	{
		*vis = f_vis_average = f_vis;
		*ir = f_ir_average = f_ir;
		*uv = f_uv_average = f_uv;
	}
	else
	{
		*vis = f_vis_average += (f_vis - f_vis_average) / 8;
		*ir = f_ir_average += (f_ir - f_ir_average) / 8;
		*uv = f_uv_average += (f_uv - f_uv_average) / 8;
	}

	return true;
}

