#include "CanInst.h"
#include "config.h"
#include "motor.h"
#include "SuperCap.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

Can_InfoTypedef *can_list[CAN_INST_NUM];
int8_t can_index[CAN_NUM][CAN_HEADER_NUM][9]; 

CAN_TxHeaderTypeDef *can_txheader[CAN_HEADER_NUM];

void CanSendMsg()
{
	static uint8_t can_txdata[8];
	for (int i = 0; i < CAN_HEADER_NUM; i++) // 遍历所有的header
	{
		memset(can_txdata, 0, sizeof(can_txdata));
		for (int j = 0; can_index[0][i][j] != (-1); j++) // 遍历header相同的实例，-1为结束标志
		{
			int8_t tmp_idx = can_index[0][i][j];
			// tmp_idx为调用can的实体在其列表中的索引
			can_list[tmp_idx]->FillTxMsg(can_list[tmp_idx], can_txdata, (tmp_idx >= MOTOR_NUM ? tmp_idx - MOTOR_NUM : tmp_idx));
		}
		HAL_CAN_AddTxMessage(&hcan1, can_txheader[i], can_txdata, (uint32_t *)CAN_TX_MAILBOX0);

//CAN2
		memset(can_txdata, 0, sizeof(can_txdata));
		for (int j = 0; can_index[1][i][j] != (-1); j++)
		{
			int8_t tmp_idx = can_index[1][i][j];
			can_list[tmp_idx]->FillTxMsg(can_list[tmp_idx], can_txdata, tmp_idx);
		}
		HAL_CAN_AddTxMessage(&hcan2, can_txheader[i], can_txdata, (uint32_t *)CAN_TX_MAILBOX0);

	}
}

void Can_Filter1Config(void)
{
	CAN_FilterTypeDef Filter;
	Filter.FilterActivation = ENABLE;
	Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	Filter.FilterScale = CAN_FILTERSCALE_16BIT;
	Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	Filter.FilterIdHigh = 0x0000;
	Filter.FilterIdLow = 0x0000;
	Filter.FilterMaskIdHigh = 0x0000;
	Filter.FilterMaskIdLow = 0x0000; // accept all
	Filter.FilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1, &Filter);
//CAN2
	HAL_CAN_ConfigFilter(&hcan2, &Filter);

	return;
}

void CanInit()
{
	// 将所有电机can实例的指针放入can_list中
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		can_list[i] = &(motor_list[i]->can);
	}
	// 将超级电容can实例的指针放入can_list中
	can_list[9] = &SuperCap_CanInfo;
	/* 手动配置帧头列表（取巧，1是0x1FF，2是0x200） start */
	can_txheader[0] = &motor_list[1]->can.txheader;
	can_txheader[1] = &motor_list[2]->can.txheader;
	can_txheader[2] = &(SuperCap_CanInfo.txheader);

	/* 手动配置帧头列表 end */

	memset(can_index, -1, sizeof(can_index));
	for (int i = 0; i < CAN_HEADER_NUM; i++)
	{
		uint8_t count_1 = 0;
		uint8_t count_2 = 0;

		for (int j = 0; j < CAN_INST_NUM; j++)
		{
			if (can_list[j]->txheader.StdId == can_txheader[i]->StdId)
			{
				if (can_list[j]->hcan_x == &hcan1)
					can_index[0][i][count_1++] = j;
//CAN2
				else if (can_list[j]->hcan_x == &hcan2)
					can_index[1][i][count_2++] = j;

			}
		}
	}
	Can_Filter1Config();
	return;
}

void CanCbkHandle(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef static header;
	uint8_t static data[8];
	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data);
		for (int i = 0; i < CAN_INST_NUM; i++)
		{
			if (can_list[i]->hcan_x->Instance == CAN1 && can_list[i]->rxheader.StdId == header.StdId)
				can_list[i]->CbkHandel(can_list[i], data, i);
		}
	}

// CAN2
	if (hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &header, data);
		for (int i = 0; i < CAN_INST_NUM; i++)
		{
			if (can_list[i]->hcan_x->Instance == CAN2 && can_list[i]->rxheader.StdId == header.StdId)
				can_list[i]->CbkHandel(can_list[i], data, i);
		}
	}

}
//回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanCbkHandle(hcan);
}
