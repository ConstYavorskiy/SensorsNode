#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"


void Error_Handler(void);


#define LED_0 GPIO_PIN_0
#define LED_1 GPIO_PIN_1
#define LED_2 GPIO_PIN_2
#define LED_Port GPIOB

#define USB_Connect_Pin GPIO_PIN_4
#define USB_Connect_Port GPIOB



typedef struct
{
    uint8_t Y;
    uint8_t M;
    uint8_t D;
    uint8_t T;        // x6m
} Telemetry_DateTime_TypeDef;

typedef struct
{
    uint32_t Vis;
    uint32_t IR;
    uint32_t UV;
} Telemetry_Light_TypeDef;

typedef struct
{
    uint16_t Tmp;        // x10
    uint16_t Humidity;   // x10
} Telemetry_Device_TypeDef;

typedef struct
{
    uint16_t Tmp;        // x10
    uint16_t Pressure;   // x10
    uint16_t Depth;      // millimeters
} Telemetry_Water_TypeDef;

typedef struct
{
    uint16_t X;
    uint16_t Y;
    uint16_t Z;
} Telemetry_Vector_TypeDef;


typedef enum
{
   Undefined,
   Relay1_Enabled = 1,
   Relay2_Enabled = 2,
   Valve_Enabled = 4,
} Telemetry_PeripheryState;

typedef struct
{
	Telemetry_DateTime_TypeDef Time;

    uint16_t Vin;        // x100
    uint16_t Vbat;       // x100
    uint16_t Tmp;        // x10
    uint16_t Humidity;   // x10
    uint16_t Pressure;   // x10

    Telemetry_Light_TypeDef Light;
	Telemetry_Device_TypeDef Device1;
	Telemetry_Device_TypeDef Device2;

	Telemetry_Water_TypeDef WaterIn;
	Telemetry_Water_TypeDef WaterOut;

	Telemetry_PeripheryState Periphery;

} Telemetry_TypeDef;



#endif
