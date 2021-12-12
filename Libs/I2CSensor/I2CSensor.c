#include "I2CSensor.h"

static const uint16_t I2CSensor_Timeout = 1000;

uint16_t toU16_LSBMSB(uint8_t *buff) {
	return (uint16_t)(((uint16_t)buff[1] << 8) | buff[0]);
}

uint16_t toU16_MSBLSB(uint8_t *buff) {
	return (uint16_t) (((uint16_t)buff[0] << 8) | buff[1]);
}


bool I2CSensor_cmd(I2CSensor_HandleTypedef *sensor, uint8_t cmd) {
	if (HAL_I2C_Master_Transmit(sensor->i2c, sensor->addr_shifted, &cmd, 1, I2CSensor_Timeout) == HAL_OK) {
		return true;
	} else {
		return false;
	}
}

bool I2CSensor_write8(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t value) {
	if (HAL_I2C_Mem_Write(sensor->i2c, sensor->addr_shifted, reg, 1, &value, 1, I2CSensor_Timeout) == HAL_OK) {
		return true;
	} else {
		return false;
	}
}

bool I2CSensor_write(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value, uint8_t len) {
	if (HAL_I2C_Mem_Write(sensor->i2c, sensor->addr_shifted, reg, 1, value, len, I2CSensor_Timeout) == HAL_OK) {
		return true;
	} else {
		return false;
	}
}

bool I2CSensor_read8(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value) {
	if (HAL_I2C_Mem_Read(sensor->i2c, sensor->addr_shifted, reg, 1, value, 1, I2CSensor_Timeout) == HAL_OK) {
		return true;
	} else {
		return false;
	}
}

bool I2CSensor_read16_LSBMSB(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint16_t *value) {
	uint8_t rx_buff[2];
	if (HAL_I2C_Mem_Read(sensor->i2c, sensor->addr_shifted, reg, 1, rx_buff, 2, I2CSensor_Timeout) == HAL_OK) {
		*value = toU16_LSBMSB(rx_buff);
		return true;
	} else {
		return false;
	}
}

bool I2CSensor_read16_MSBLSB(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint16_t *value) {
	uint8_t rx_buff[2];
	if (HAL_I2C_Mem_Read(sensor->i2c, sensor->addr_shifted, reg, 1, rx_buff, 2, I2CSensor_Timeout) == HAL_OK) {
		*value = toU16_MSBLSB(rx_buff);
		return true;
	} else {
		return false;
	}
}

bool I2CSensor_read(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value, uint8_t len) {
	if (HAL_I2C_Mem_Read(sensor->i2c, sensor->addr_shifted, reg, 1, value, len, I2CSensor_Timeout) == HAL_OK) {
		return true;
	} else {
		return false;
	}
}
