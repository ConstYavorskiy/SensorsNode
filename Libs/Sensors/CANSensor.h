#ifndef _CAN_SENSOR_H_
#define _CAN_SENSOR_H_


typedef struct
{
    uint16_t Id;
    uint16_t Sender;
    float Value;
} CAN_Message_TypeDef;

#define CAN_MASTER         0x0000
#define CAN_SENSOR_1       0x0100
#define CAN_SENSOR_2       0x0200
#define CAN_SENSOR_3       0x0300

#define CAN_TMP75          0x0010

#define CAN_Si7021_Temp    0x0020
#define CAN_Si7021_Humi    0x0021

#define CAN_MS5837_Temp    0x0030
#define CAN_MS5837_Press   0x0031
#define CAN_MS5837_Depth   0x0032

#define CAN_Si1132_VIS     0x0041
#define CAN_Si1132_IR      0x0042
#define CAN_Si1132_UV      0x0043

#endif
