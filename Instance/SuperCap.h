#ifndef SUPERCAP_H
#define SUPERCAP_H

// #define SUPERCAP_CAN_ID 0x061 // 超级电容CAN ID

#include "stm32f4xx_hal.h"
#include "main.h"
#include "can.h"
#include "CanInst.h"
typedef struct
{
    uint8_t can_id;
    uint16_t txheader_id;
    uint16_t rxheader_id;
    CAN_HandleTypeDef *hcan;
} SuperCap_InitTypedef;

typedef struct __attribute__((packed))
{
    uint8_t errorCode;          // 报错代码
    float chassisPower;         // 底盘功率
    uint16_t chassisPowerLimit; // 底盘功率限制
    uint8_t capEnergy;          // 超级电容能量
} supercap_rx_data_t;

typedef struct __attribute__((packed))
{
    uint8_t enableDCDC : 1;               // 1位位域，使能DCDC
    uint8_t systemRestart : 1;            // 1位位域，系统重启
    uint8_t resv0 : 6;                    //  6 位的位域，保留位，未使用
    uint16_t feedbackRefereePowerLimit;   // 16 位无符号整数，用于表示反馈裁判的功率限制
    uint16_t feedbackRefereeEnergyBuffer; // 16 位无符号整数，用于表示反馈裁判的能量缓冲
    uint8_t resv1[3];                     // 3 个字节的保留位，未使用
} supercap_tx_data_t;

extern supercap_rx_data_t supercap_rx_data;
extern supercap_tx_data_t supercap_tx_data;
extern Can_InfoTypedef SuperCap_CanInfo;
void SuperCapCtrlData_Send(void);
void SuperCapRegist(SuperCap_InitTypedef st);

void SuperCapInit(void);

#endif // SUPERCAP_H
