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
uint16_t highWaterMark_canbus;
RobotState_e robo_state;
uint8_t CanBus_diff = 0;

void CANBusInit ()
{
	const osThreadAttr_t attr = {
			.name = "CANBusTask",
			.stack_size = 128 * 8,
			.priority = (osPriority_t) osPriorityHigh,
	};
	canbus_task_handel = osThreadNew (CANBusTask, NULL, &attr);
	
}


void CANBusTask (void *argument)
{
	uint32_t time = osKernelGetTickCount ();
	MotorStop (NULL);
	robo_state = BAN;
	
	for (;;)
	{

		if (RC_Ctl . rc . sw1 == 3)
		{
			MotorStop (NULL);
			robo_state = BAN;
		} 
		else if(RC_Ctl . rc . sw1 == 1)
		{
			if (robo_state == BAN)
			{
				for (int i = 0; i < MOTOR_NUM; i ++)
					motor_list[i] -> info . enable = 1;
				GimbalMotorPosi_Init();
			}
			robo_state = NORMAL;
		}


 		if(RC_Ctl . keyboard . C)
 		{
			for (int i = 0; i < MOTOR_NUM; i ++)
			motor_list[i] -> info . enable = 1;
			GimbalMotorPosi_Init();
 			robo_state = NORMAL;
 		}
 		else if(RC_Ctl . keyboard . B)
 		{
			MotorStop (NULL);
 			robo_state = BAN;
 		}

 		if(robo_state == NORMAL )
 			MotorCalc ();
 		else if(robo_state == BAN)
 			MotorStop (NULL);



		CanSendMsg ();
		CanBus_diff = osKernelGetTickCount () - time;

		highWaterMark_canbus = uxTaskGetStackHighWaterMark(NULL);
		time = osKernelGetTickCount ();
		osDelayUntil (time + CANBUS_TASK_PERIOD);
	}
	
	
}
