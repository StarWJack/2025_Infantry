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
}__attribute__((__packed__))Chassis_CmdTypedef;

typedef struct
{
    float target;
    float output;
    float omega_z_last;
    float omega_z_now;
    float omega_z_tar;
    PID_TypeDef pid[2];    //outer(or angle)(0)、inner(or speed)(1) circle
}__attribute__((__packed__))Chassis_TypeDef;


extern osThreadId_t chassis_task_handel;;


extern Chassis_CmdTypedef chassis_cmd;

void ChassisInit(void);
float ChassisGetOmega(void);
#endif
