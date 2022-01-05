#ifndef _I2C_SENSOR_H_
#define _I2C_SENSOR_H_

#include "main.h"
#include <stdbool.h>

typedef struct {
    uint8_t addr;
    uint8_t addr_shifted;
    I2C_HandleTypeDef* i2c;
} I2CSensor_HandleTypedef;

uint16_t toU16_LSBMSB(uint8_t *buff);
uint16_t toU16_MSBLSB(uint8_t *buff);

bool I2CSensor_cmd(I2CSensor_HandleTypedef *sensor, uint8_t cmd);
bool I2CSensor_write8(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t value);
bool I2CSensor_write (I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value, uint8_t len);

bool I2CSensor_read8 (I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value);
bool I2CSensor_read16_LSBMSB(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint16_t *value);
bool I2CSensor_read16_MSBLSB(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint16_t *value);
bool I2CSensor_read  (I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value, uint8_t len);

#endif
