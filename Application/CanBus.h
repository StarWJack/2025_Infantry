#ifndef __CANBUS_H__
#define __CANBUS_H__

#include "main.h"
#include "cmsis_os2.h"

typedef enum
{
    BAN = 0,//禁用
    NORMAL = 1//正常
}RobotState_e;

extern RobotState_e robo_state;

extern void CANBusTask(void *argument);
void CANBusInit();
//extern osThreadId_t canbus_task_handel;
#endif
