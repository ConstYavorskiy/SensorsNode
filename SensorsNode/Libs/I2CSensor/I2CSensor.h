#ifndef I2CSENSOR_H_
#define I2CSENSOR_H_

#include "main.h"
#include <stdbool.h>

typedef struct {
    uint16_t addr;
    uint16_t addr_shifted;
    I2C_HandleTypeDef* i2c;
} I2CSensor_HandleTypedef;

uint16_t to_int16(uint8_t *buff);

bool I2CSensor_write8(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t value);
bool I2CSensor_write (I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value, uint8_t len);

bool I2CSensor_read8 (I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value);
bool I2CSensor_read16(I2CSensor_HandleTypedef *sensor, uint8_t reg, uint16_t *value);
bool I2CSensor_read  (I2CSensor_HandleTypedef *sensor, uint8_t reg, uint8_t *value, uint8_t len);

#endif
