/*
 * @file SuperCap.c
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-30
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "SuperCap.h"
#include "can.h"
#include "string.h"

supercap_rx_data_t supercap_rx_data; // 定义接收数据结构体
supercap_tx_data_t supercap_tx_data; // 定义发送数据结构体
Can_InfoTypedef SuperCap_CanInfo;

CAN_TxHeaderTypeDef supercap_txheader = {
    .ExtId = 0x0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE,
};

// static void SuperCapFillMsg(Can_InfoTypedef *st, uint8_t *txdata, uint8_t list_id);
// static void SuperCapCanCbk(Can_InfoTypedef *st, uint8_t *rxdata, uint8_t list_id);
// /*
//  * @brief 发送超级电容控制数据
//  *
//  */
// void SuperCapCtrlData_Send(void)
// {
//     HAL_CAN_AddTxMessage(&hcan1, &supercap_tx_header, (uint8_t *)&supercap_tx_data, (uint32_t *)CAN_TX_MAILBOX0);
// }
// /*
//  * @brief CAN接收中断回调函数
//  *
//  * @param hcan can句柄
//  */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     if (hcan == &hcan1)
//     {
//         CAN_RxHeaderTypeDef rx_header;
//         HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, (uint8_t *)&supercap_rx_data);
//     }
//     HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
// }

void SuperCapFillMsg(Can_InfoTypedef *st, uint8_t *txdata, uint8_t list_id)
{
    memcpy(txdata, &supercap_tx_data, sizeof(supercap_tx_data));
}

void SuperCapCanCbk(Can_InfoTypedef *st, uint8_t *rxdata, uint8_t list_id)
{
    memcpy(&supercap_rx_data, rxdata, sizeof(supercap_rx_data));
}

/*
 * @brief
 *
 */
void SuperCapInit(void)
{
    // supercap_tx_data.enableDCDC = 0;//初始化先不使能DCDC
    // supercap_tx_data.systemRestart = 1;//系统重启
    // supercap_tx_data.feedbackRefereePowerLimit = 11;
    // supercap_tx_data.feedbackRefereeEnergyBuffer = 22;
    // SuperCap_CanInfo.txheader.StdId = SUPERCAP_CAN_ID;
    SuperCap_InitTypedef init = {
        .can_id = 1,
        .hcan = &hcan1,       // 超级电容挂载在CAN1上
        .txheader_id = 0x061, // 发送ID
        .rxheader_id = 0x097, // 接收ID
    };
    SuperCapRegist(init);
}

void SuperCapRegist(SuperCap_InitTypedef st)
{
    supercap_txheader.StdId = st.txheader_id;
    SuperCap_CanInfo.txheader = supercap_txheader;
    SuperCap_CanInfo.rxheader.StdId = st.rxheader_id;
    SuperCap_CanInfo.rxheader.DLC = 8;
    SuperCap_CanInfo.CAN_id = st.can_id;
    SuperCap_CanInfo.FillTxMsg = SuperCapFillMsg;
    SuperCap_CanInfo.CbkHandel = SuperCapCanCbk;
    SuperCap_CanInfo.hcan_x = st.hcan;
}
