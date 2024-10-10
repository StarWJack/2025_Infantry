#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "cmsis_os2.h"

#if INFANTRY
#define SQRT_2 1.4142135f
#endif

#if HERO
//TODO(GUATAI)英雄车体数据待更新
//轴距（wheelbase） 即前后轮轴间距离
#define Length 0.38f
//轮距（track） 即左右驱动轮中心距离
#define Width 0.41f
//驱动轮直径（diameter）
#define Wheel_radius 0.075f
//底盘功率限制
#define Power_limit 80.0f
//驱动电机（driving motor） 驱动底盘移动。目前型号为 M3508，反馈转子转速单位为 rpm
#define Speed_rpm_Limit 450.0f
#endif

typedef struct
{
    float vx;
    float vy;
    float omega_z;   //spin clockwise from above when positive 
    uint8_t stop:1;
}__attribute__((__packed__))Chassis_CmdTypedef;

extern osMessageQueueId_t chassis_MQ_handel;
extern osMessageQueueId_t chassis_RMQ_handel;
extern osThreadId_t chassis_task_handel;;
void ChassisInit(void);
#endif
