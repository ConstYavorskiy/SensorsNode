
#ifndef __TSC_H
#define __TSC_H

#include "Sensors/I2CSensor.h"

// Touch Screen Information structure
typedef struct
{
	bool TouchDetected;
	uint16_t X;
	uint16_t Y;
	// uint16_t Z;
} TSC_STATE_TypeDef;

typedef struct
{
	float Temp0;
	float Temp1;
	float Vin;
	float Vbat;
} TSC_ENVIRONMENT_TypeDef;

// IO_Expander Error codes
typedef enum
{
	TSC_OK = 0,
	TSC_FAILURE,
	TSC_TIMEOUT,
	PARAM_ERROR,
	TSC1_NOT_OPERATIONAL,
	TSC2_NOT_OPERATIONAL
} TSC_Status_TypDef;

// TSC DMA Direction
typedef enum
{
	TSC_DMA_TX = 0,
	TSC_DMA_RX = 1
} TSC_DMADirection_TypeDef;




/* I2C clock speed configuration (in Hz)
  WARNING:
   Make sure that this define is not already declared in other files (ie.
  STM32F4_EVB.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED						100000
#endif /* I2C_SPEED */

/**
  * @brief  TSC DMA definitions
  */
#define TSC_DMA_CLK						RCC_AHB1Periph_DMA1
#define TSC_DMA_CHANNEL					DMA_Channel_1
#define TSC_DMA_TX_STREAM				DMA1_Stream6
#define TSC_DMA_RX_STREAM				DMA1_Stream0
#define TSC_DMA_TX_TCFLAG				DMA_FLAG_TCIF6
#define TSC_DMA_RX_TCFLAG				DMA_FLAG_TCIF0


bool TSC_init(I2C_HandleTypeDef *hi2c);

// Touch Screen controller functions
void TSC_GetTouchState(TSC_STATE_TypeDef* state);
void TSC_GetEnvironment(TSC_ENVIRONMENT_TypeDef* env);


/**
  * @brief  Timeout user callback function. This function is called when a timeout
  *         condition occurs during communication with IO Expander. Only protoype
  *         of this function is decalred in IO Expander driver. Its implementation
  *         may be done into user application. This function may typically stop
  *         current operations and reset the I2C peripheral and IO Expander.
  *         To enable this function use uncomment the define USE_TIMEOUT_USER_CALLBACK
  *         at the top of this file.
  */
#ifdef USE_TIMEOUT_USER_CALLBACK
 uint8_t TSC_TimeoutUserCallback(void);
#else
 #define TSC_TimeoutUserCallback()  TSC_TIMEOUT
#endif /* USE_TIMEOUT_USER_CALLBACK */

#endif
