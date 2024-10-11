#include "CanBus.h"
#include "cmsis_os2.h"
#include "config.h"
#include "CanInst.h"
#include "remote.h"
#include "motor.h"
#include "sin.h"
#include "usart.h"

osThreadId_t canbus_task_handel;
RobotState_e robo_state;

void CANBusInit()
{
	const osThreadAttr_t attr = {
	  .name = "CANBusTask",
	  .stack_size = 128 * 4,
	  .priority = (osPriority_t) osPriorityHigh,
	};
    canbus_task_handel = osThreadNew(CANBusTask, NULL, &attr);
    
}

uint32_t diff=0;
void CANBusTask(void *argument)
{
    uint32_t time = osKernelGetTickCount();
    robo_state = NORMAL;
//	for(int i=0;i<FREQTAB_LEN;i++)
//		Freq_Tab[i] *= 0.5;
    for(;;)
    {
        if(RC_Ctl.rc.sw1 == 2 || RC_Ctl.rc.sw1 == 0 )
        {
            MotorStop(NULL);
            robo_state = BAN;
        }
        else
        {
            if(robo_state == BAN)
            {
                 for(int i=0;i<MOTOR_NUM;i++)
                    motor_list[i]->info.enable = 1;
                 MotorSetTar(motor_list[MOTOR_YAW], MotorGetVal(motor_list[MOTOR_YAW],RAD), ABS);
                 MotorSetTar(motor_list[MOTOR_PITCH], MotorGetVal(motor_list[MOTOR_PITCH],RAD), ABS);
                 MotorSetTar(motor_list[MOTOR_AMMO], MotorGetVal(motor_list[MOTOR_AMMO],RAD),ABS);
            }

            MotorCalc();
            robo_state = NORMAL;
        }
    	CanSendMsg();
    	diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
    	osDelay(CANBUS_TASK_PERIOD);
    }


//        osDelayUntil(time + CANBUS_TASK_PERIOD);


}
