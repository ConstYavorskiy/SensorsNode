#ifndef _CAN_SENSOR_H_
#define _CAN_SENSOR_H_


typedef struct
{
    uint16_t Id;
    uint16_t Sender;
    float Value;
} CAN_Message_TypeDef;

#define CAN_MASTER         0x00
#define CAN_SENSOR_1       0x01
#define CAN_SENSOR_2       0x02
#define CAN_SENSOR_3       0x03

#define CAN_Temp           0x00
#define CAN_Vin            0x01
#define CAN_Vbat           0x02

#define CAN_BME_Temp       0x10
#define CAN_BME_Humi       0x11
#define CAN_BME_Press      0x12

#define CAN_Si7021_Temp    0x20
#define CAN_Si7021_Humi    0x21

#define CAN_Si1132_VIS     0x51
#define CAN_Si1132_IR      0x52
#define CAN_Si1132_UV      0x53

#define CAN_Acc_X          0x70
#define CAN_Acc_Y          0x71
#define CAN_Acc_Z          0x72
#define CAN_Gyro_X         0x73
#define CAN_Gyro_Y         0x74
#define CAN_Gyro_Z         0x75
#define CAN_Magn_X         0x76
#define CAN_Magn_Y         0x77
#define CAN_Magn_Z         0x78

#define CAN_MS5837_Temp    0x80
#define CAN_MS5837_Press   0x81
#define CAN_MS5837_Depth   0x82


#endif


/*
Clock
Temp
Vin
Vbat
BME
BMC
Si7021
MS5837
Relay1
Relay2
Valve0

 */
