#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "cmsis_os2.h"
#include "pid.h"
#include "motor.h"

#define SQRT_2 1.4142135f

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
