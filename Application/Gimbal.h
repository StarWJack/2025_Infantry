#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "main.h"
#include "cmsis_os2.h"


typedef enum
{
	STOP = 0,
	AUTO = 2,//火控
	FULL = 1,//泼水
	TEST = 3,//测试 小拨盘上下发弹
} shooter_strategy_e;//开火策略

typedef enum
{
	SHOOTER_NORMAL = 0,  //正转
	SHOOTER_REVERSE = 1  //反转
} shooter_state_e;

typedef struct
{
	shooter_strategy_e shooter;
	shooter_state_e shooter_state;
	uint8_t friction_state;//摩擦轮状态:0-关闭，1-开启
	float pitch;
	float yaw;
	float v_yaw;          //YAW_角速度
	float v_pitch;
}__attribute__((__packed__)) Gimbal_CmdTypedef;


//extern osMessageQueueId_t gimbal_MQ_handel;
extern osThreadId_t gimbal_task_handel;
extern Gimbal_CmdTypedef gimbal_cmd;

extern void GimbalTask (void *argument);

void GimbalMotorPosi_Init (void);

void GimbalInit (void);

#endif
