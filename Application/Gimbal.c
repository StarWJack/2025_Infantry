#include "Gimbal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "config.h"
#include "pid.h"
#include "ErrorHandel.h"
#include "math.h"
#include "remote.h"
#include "imu.h"

//#include "Serial.h"
//#include "SolveTrajectory.h"

osThreadId_t gimbal_task_handel;
Gimbal_CmdTypedef gimbal_cmd;


void GimbalCtrl (Gimbal_CmdTypedef *st)
{
	//目标值+增量
 	float tmp = motor_list[MOTOR_YAW] -> target + st -> v_yaw ;
// （0 < tmp < 2*PI) 过零点限制范围
 	while ( tmp > 2*PI )
 		tmp -= 2 * PI;
 	while ( tmp <0 )
 		tmp += 2 * PI;

// 	while ( tmp > 8192 )
// 			tmp -= 8192;
// 	while ( tmp <0 )
// 			tmp += 8192;
//	float tmp = st -> v_yaw ;
	MotorSetTar (motor_list[MOTOR_YAW], tmp, ABS);
	
	// tmp = motor_list[MOTOR_PITCH] -> target - st -> v_pitch;
	// if ( tmp < PI - (PITCH_LW_BOUND - imu_list[0] -> info . pitch_zero_point) / 8192 * 2 * PI )
	// tmp = PI - (PITCH_LW_BOUND - imu_list[0] -> info . pitch_zero_point) / 8192 * 2 * PI;
	// else if ( tmp > PI - (PITCH_UP_BOUND + imu_list[0] -> info . pitch_zero_point) / 8192 * 2 * PI )
	// 	tmp = PI - (PITCH_UP_BOUND + imu_list[0] -> info . pitch_zero_point) / 8192 * 2 * PI;
	//有待优化，上限角度限死
	tmp = motor_list[MOTOR_PITCH] -> target - st -> v_pitch;
	//pitch限制范围，绝对位置---待改
	if(tmp > PITCH_LH_BOUND)
		tmp = PITCH_LH_BOUND;
	else if(tmp < PITCH_LL_BOUND)
		tmp = PITCH_LL_BOUND;
	MotorSetTar (motor_list[MOTOR_PITCH], tmp, ABS);
}


void ShooterCtrl (Gimbal_CmdTypedef *st)
{

	if ( st -> friction_state )
	{//固定射速，待添加遥控器遥杆控制   
		MotorSetTar (motor_list[MOTOR_SHT_L], BULLET_V, ABS);
		MotorSetTar (motor_list[MOTOR_SHT_R], - BULLET_V, ABS);
	}
	else
	{
		gimbal_cmd . shooter_state = SHOOTER_STOP;
		MotorSetTar (motor_list[MOTOR_SHT_L], 0, ABS);
		MotorSetTar (motor_list[MOTOR_SHT_R], 0, ABS);
	}
	//sw2 == 2;
	if ( st -> shooter == STOP )
	{
		gimbal_cmd . shooter_state = SHOOTER_STOP;
		MotorSetTar (motor_list[MOTOR_AMMO], 0, ABS);
//		MotorSetTar (motor_list[MOTOR_AMMO], MotorGetVal (motor_list[MOTOR_AMMO], MOTORANGLE, RAD), ABS);
	}

//	else if ( st -> shooter == FULL )
//		gimbal_cmd . shooter_state = SHOOTER_NORMAL;
//		MotorSetTar (
//				motor_list[MOTOR_AMMO], (36 * shoot_freq * (float) (GIMBAL_TASK_PERIOD) / 1000) * (2 * PI / 8), INCR
//		            );//累加
	else if( st -> shooter == TEST )
	{
		float tar = st -> v_shoot * 2 * PI /36; // 实际弹频 = v_shoot/36（拨弹盘一圈）/10(任务周期) *8  k/s
		if(tar > 0)
		{
			tar = -tar;
		}
		MotorSetTar (motor_list[MOTOR_AMMO], tar, ABS);
	}
}

void GimbalMotorPosi_Init (void)
{
	MotorSetTar (motor_list[MOTOR_YAW], MotorGetVal (motor_list[MOTOR_YAW], IMU, RAD), ABS);
	MotorSetTar (motor_list[MOTOR_PITCH], MotorGetVal (motor_list[MOTOR_PITCH], IMU, RAD), ABS);
	MotorSetTar (motor_list[MOTOR_AMMO], MotorGetVal (motor_list[MOTOR_AMMO], MOTORANGLE, RAD), ABS);
}


void GimbalInit (void)
{
	osThreadAttr_t attr_task = {.name = "GimbalTask"};
	gimbal_task_handel = osThreadNew (GimbalTask, NULL, &attr_task);
	
}

void GimbalTask (void *argument)
{
	GimbalMotorPosi_Init ();
	uint32_t time = osKernelGetTickCount ();
	for ( ;; )
	{
		GimbalCtrl (&gimbal_cmd);
		ShooterCtrl (&gimbal_cmd);
		
		time = osKernelGetTickCount ();
		osDelayUntil (time + GIMBAL_TASK_PERIOD);
		
	}
}

