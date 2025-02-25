#include "CanBus.h"
#include "cmsis_os2.h"
#include "config.h"
#include "CanInst.h"
#include "remote.h"
#include "motor.h"
#include "usart.h"
#include "imu.h"
#include "vofa.h"
#include "Gimbal.h"
#include "SuperCap.h"
#include "ChassisPowerCtrl.h"

osThreadId_t canbus_task_handel;
RobotState_e robo_state;
uint32_t CanBus_diff = 0;

void CANBusInit ()
{
	const osThreadAttr_t attr = {
			.name = "CANBusTask",
			.stack_size = 128 * 16,
			.priority = (osPriority_t) osPriorityHigh,
	};
	canbus_task_handel = osThreadNew (CANBusTask, NULL, &attr);
	
}


void CANBusTask (void *argument)
{
	uint32_t time = osKernelGetTickCount ();
	robo_state = BAN;
	
	for (;;)
	{

		if (RC_Ctl . rc . sw1 != 1 && RC_Ctl . rc . sw1 != 3)
		{
			MotorStop (NULL);
			robo_state = BAN;
		} 
		else
		{
			if (robo_state == BAN)
			{
				for (int i = 0; i < MOTOR_NUM; i ++)
					motor_list[i] -> info . enable = 1;
				GimbalMotorPosi_Init();
			}
			my_MotorCalc ();
			robo_state = NORMAL;
		}



		CanSendMsg ();
		CanBus_diff = osKernelGetTickCount () - time;
		time = osKernelGetTickCount ();
		osDelayUntil (time + CANBUS_TASK_PERIOD);
	}
	
	
}
