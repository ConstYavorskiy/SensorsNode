#include "tsc.h"

#define TIMEOUT_MAX    0x3000 /*<! The value of the maximal timeout for I2C waiting loops */

#define TSC_DeviceAddress 0b1001000	// A1=A0=0
#define TSC_CMD_TEMP0     0b0000
#define TSC_CMD_VBAT1     0b0001
#define TSC_CMD_IN1       0b0010
#define TSC_CMD_TEMP1     0b0100
#define TSC_CMD_VBAT2     0b0101
#define TSC_CMD_IN2       0b0110

#define TSC_CMD_ActivateX  0b1000
#define TSC_CMD_ActivateY  0b1001
#define TSC_CMD_ActivateXY 0b1010

#define TSC_CMD_XPOS       0b1100
#define TSC_CMD_YPOS       0b1101
#define TSC_CMD_Z1POS      0b1110
#define TSC_CMD_Z2POS      0b1111

#define TSC_PD_IRQ 0b10
#define TSC_PD_ADC 0b11

#define TSC_Mode_8bit   0b1
#define TSC_Mode_12bit  0b0

#define TSC_cmd  0b1100


static I2CSensor_HandleTypedef hTSC;

static uint16_t TSC_EnableTouchPanelIRQ()
{
	// Enable TSC2003 /PENIRQ
	uint16_t response;
	I2CSensor_read16(&hTSC, 0xD0, &response);
	return response;
}

static uint16_t TSC_Read(uint8_t cmd)
{
	// command byte= [C3 C2 C1 C0 0 1 0 0]
	uint8_t pCommand = ((cmd << 4) | TSC_cmd);
	HAL_I2C_Master_Transmit(hTSC.i2c, hTSC.addr_shifted, &pCommand, 1, 1000);
	uint8_t rx_buff[2];
	HAL_I2C_Master_Receive(hTSC.i2c, hTSC.addr_shifted, &rx_buff, 2, 1000);


	uint16_t response;
	//I2CSensor_read(&hTSC, pCommand, &rx_buff, 2);


	// and save only 10 MSBs for reducing noise
	response = (((uint16_t)rx_buff[0] << 4) | ((uint16_t)rx_buff[1] >> 4));
	return response;
}

/**
 @brief Begin Device
 @retval true normaly done
 @retval false device error
 */
bool TSC_init(I2C_HandleTypeDef *hi2c) {
	hTSC.i2c = hi2c;
	hTSC.addr = TSC_DeviceAddress;
	hTSC.addr_shifted = (TSC_DeviceAddress << 1);


	return true;
}

//const float VBatScale = 1000 / 4096; // 100 * 2.5 * 4 / 4096;

void TSC_GetEnvironment(TSC_ENVIRONMENT* env)
{
	uint16_t t0 = TSC_Read(TSC_CMD_TEMP0);
	uint16_t t1 = TSC_Read(TSC_CMD_TEMP1);

	env->Temp0 = 3 * t0 / 100;
	//env->Temp1 = 2.573 * (t1 - t0) - 273;
	env->Vin = (uint32_t)TSC_Read(TSC_CMD_VBAT1) * 1000 / 4096;
	env->Vbat = (uint32_t)TSC_Read(TSC_CMD_VBAT2) * 1000 / 4096;

}

void TSC_GetTouchState(TSC_STATE* state)
{
	uint16_t x_res = 240, y_res = 240,  x_min = 330, x_max = 3800, y_min = 350, y_max = 3900;
	uint32_t x, y, z;

	z = TSC_Read(TSC_CMD_Z1POS);
	if (z < 10)
	{
		return;
	}

	y = TSC_Read(TSC_CMD_YPOS);
	x = TSC_Read(TSC_CMD_XPOS);
	if (x > x_max) x = x_max;
	if (x < x_min) x = x_min;
	if (y > y_max) y = y_max;
	if (y < y_min) y = y_min;

	state->X = (x - x_min) * x_res / (x_max - x_min);
	state->Y = (y - y_min) * y_res / (y_max - y_min);
}

/*
static void TSC_I2C_Config(void)
{
	I2C_InitTypeDef I2C_InitStructure;

//	if ((TSC_I2C->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
	{
		I2C_InitStructure.I2C_Mode					= I2C_Mode_I2C;
		I2C_InitStructure.I2C_DutyCycle				= I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1			= 0xA0;
		I2C_InitStructure.I2C_Ack					= I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress	= I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_ClockSpeed			= I2C_SPEED;
		I2C_Init(TSC_I2C, &I2C_InitStructure);
	}
}

static void TSC_DMA_Config(TSC_DMADirection_TypeDef Direction, uint8_t* buffer)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(TSC_DMA_CLK, ENABLE);
	DMA_InitStructure.DMA_Channel				= TSC_DMA_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr	= TSC_I2C_DR;
	DMA_InitStructure.DMA_Memory0BaseAddr		= (uint32_t)buffer;
	DMA_InitStructure.DMA_PeripheralInc			= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc				= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode					= DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority				= DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode				= DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold			= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst		= DMA_PeripheralBurst_Single;
	if (Direction == TSC_DMA_RX)
	{
		DMA_InitStructure.DMA_DIR				= DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize		= 2;
		DMA_DeInit(TSC_DMA_RX_STREAM);
		DMA_Init(TSC_DMA_RX_STREAM, &DMA_InitStructure);
	}
	else if (Direction == TSC_DMA_TX)
	{
		DMA_InitStructure.DMA_DIR				= DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize		= 1;
		DMA_DeInit(TSC_DMA_TX_STREAM);
		DMA_Init(TSC_DMA_TX_STREAM, &DMA_InitStructure);
	}
}

static void TSC_EXTI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// Enable GPIO clock
	RCC_AHB1PeriphClockCmd(TSC_IT_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Configure Button pin as input floating
	GPIO_InitStructure.GPIO_Pin		= TSC_IT_PIN;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_Init(TSC_IT_GPIO_PORT, &GPIO_InitStructure);

	// Connect Button EXTI Line to Button GPIO Pin
	SYSCFG_EXTILineConfig(TSC_IT_EXTI_PORT_SOURCE, TSC_IT_EXTI_PIN_SOURCE);

	// Configure Button EXTI line
	EXTI_InitStructure.EXTI_Line	= TSC_IT_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set Button EXTI Interrupt to the lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = TSC_IT_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
*/
