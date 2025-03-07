#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "cmsis_os2.h"
#include "pid.h"
#include "motor.h"

#if INFANTRY
#define SQRT_2 1.4142135f
#endif

typedef enum
{
    SPIN    = 1, //小陀螺
    FOLLOW  = 3,//跟随
    SYOP    = 2//不动
}chassis_mode_e;


typedef struct
{
    float vx;
    float vy;

    float omega_z;       //底盘小陀螺时的角速度(rad/s)
    float omega_fllow;   //底盘跟随时的角速度  (rad/s)
    chassis_mode_e mode;
}__attribute__((__packed__))Chassis_CmdTypedef;

typedef struct
{
    float target;
    float value;
    float error;
    float output;

    float omega_z_ref;
    PID_TypeDef pid[2];    //outer(or angle)(0)、inner(or speed)(1) circle
}__attribute__((__packed__))Chassis_TypeDef;


extern osThreadId_t chassis_task_handel;;

extern Chassis_TypeDef chassis ;
extern Chassis_CmdTypedef chassis_cmd;

void ChassisInit(void);
float ChassisGetOmega(void);
#endif
