#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "main.h"
#include "cmsis_os2.h"

typedef enum
{
	AUTO = 1,//云台跟随
	SHOOT = 3,//手动射击
	STOP = 2,//停止发射
	AUTO_SHOOT = 4,//自瞄+手打
	AUTO_FIRE = 5  //自瞄+火控
} shooter_strategy_e;//开火策略


typedef struct
{
	shooter_strategy_e shooter;
	uint8_t friction_state;//摩擦轮状态:0-关闭，1-开启
	uint8_t shoot_advice;
	uint8_t mouse_flag;//优化操作手界面超调现象
	uint8_t spin_flag;

	float pitch;
	float yaw;

	float v_yaw;          //YAW_角速度
	float v_pitch;
	float v_shoot;

}__attribute__((__packed__)) Gimbal_CmdTypedef;

typedef struct
{
	float yaw_diff;
	float pitch_diff;

	float yaw;
	float pitch;
}__attribute__((__packed__)) Gimbal_Auto;

extern osThreadId_t gimbal_task_handel;
extern Gimbal_CmdTypedef gimbal_cmd;
extern Gimbal_Auto gimbal_auto;

extern void GimbalTask (void *argument);

void GimbalMotorPosi_Init (void);

void GimbalInit (void);

#endif
