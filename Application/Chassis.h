#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "cmsis_os2.h"
#include "pid.h"
#include "motor.h"

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

typedef enum
{
//    NOSPIN  ,//不动
    FOLLOW  = 3,//跟随
    SPIN    = 1 //小陀螺
}chassis_mode_e;


typedef struct
{
    float vx;
    float vy;
    chassis_mode_e mode;
    float omega_z;   //spin clockwise from above when positive

}__attribute__((__packed__))Chassis_CmdTypedef;

typedef struct
{
    float output;
    PID_TypeDef pid_outer;
    PID_TypeDef pid_inner;
}ChassisPID_InitTypedef;

extern osMessageQueueId_t chassis_MQ_handel;
extern osMessageQueueId_t chassis_RMQ_handel;
extern osThreadId_t chassis_task_handel;;
extern ChassisPID_InitTypedef chassis_pid;

void ChassisInit(void);
#endif
