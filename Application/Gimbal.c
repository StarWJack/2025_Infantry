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
#include "Serial.h"
#include "control.h"
#include "referee.h"

osThreadId_t gimbal_task_handel;
Gimbal_CmdTypedef gimbal_cmd;
Gimbal_Auto gimbal_auto;

uint8_t gimbal_diff;//测量任务时间

void GimbalInit (void)
{
	osThreadAttr_t attr_task = {
			.name = "GimbalTask",
			.stack_size = 128 * 8,
			.priority = (osPriority_t) osPriorityAboveNormal2 ,};
	gimbal_task_handel = osThreadNew (GimbalTask, NULL, &attr_task);
}

void GimbalCtrl (Gimbal_CmdTypedef *st)
{
	float tar_yaw;
	float tar_pitch;
#ifdef USE_REMOTE
	if(st -> shooter == STOP)
	{
		//目标值+增量
		float tmp = motor_list[MOTOR_YAW] -> target + st -> v_yaw ;
		// （0 < tmp < 2*PI) 过零点限制范围
			 while ( tmp > 2*PI )
				 tmp -= 2 * PI;
			 while ( tmp <0 )
				 tmp += 2 * PI;


			MotorSetTar (motor_list[MOTOR_YAW], tmp, ABS);

			tmp = motor_list[MOTOR_PITCH] -> target - st -> v_pitch;
			//pitch限制范围，绝对位置---有待优化，上限角度限死
			if(tmp > PITCH_LH_BOUND)
				tmp = PITCH_LH_BOUND;
			else if(tmp < PITCH_LL_BOUND)
				tmp = PITCH_LL_BOUND;
			MotorSetTar (motor_list[MOTOR_PITCH], tmp, ABS);
	}
	else//自瞄
	{
		if(gimbal_auto.yaw != PI && gimbal_auto.yaw_diff != 0 )
		   tar_yaw = gimbal_auto.yaw ;
		else
		   tar_yaw = motor_list[MOTOR_YAW] -> target + st -> v_yaw;

		while ( tar_yaw > 2*PI )
			tar_yaw -= 2 * PI;
		while ( tar_yaw <0 )
			tar_yaw += 2 * PI;

		MotorSetTar (motor_list[MOTOR_YAW], tar_yaw, ABS);

		if(gimbal_auto.pitch != PI && gimbal_auto.pitch_diff != 0 )
			tar_pitch = gimbal_auto.pitch;
//			tar_pitch = (0.06 + PI);
		else
			tar_pitch = motor_list[MOTOR_PITCH] -> target - st -> v_pitch;

		if(tar_pitch > PITCH_LH_BOUND)
			tar_pitch = PITCH_LH_BOUND;
		else if(tar_pitch < PITCH_LL_BOUND)
			tar_pitch = PITCH_LL_BOUND;
		MotorSetTar (motor_list[MOTOR_PITCH], tar_pitch, ABS);
	}

#endif

#ifdef USE_KEYBOARD

		if(st -> shooter == AUTO || st -> shooter == AUTO_SHOOT || st -> shooter == AUTO_FIRE)//自瞄 -- 云台跟随和火控分开
		{
			if(gimbal_auto.yaw != PI && gimbal_auto.yaw_diff != 0)
						tar_yaw = gimbal_auto.yaw ;
			else
				tar_yaw = motor_list[MOTOR_YAW] -> target + st -> v_yaw;

			while ( tar_yaw > 2*PI )
				tar_yaw -= 2 * PI;
			while ( tar_yaw <0 )
				tar_yaw += 2 * PI;
			MotorSetTar (motor_list[MOTOR_YAW], tar_yaw, ABS);

			if(gimbal_auto.pitch != PI && gimbal_auto.pitch_diff != 0)
				tar_pitch = gimbal_auto.pitch;
			else
				tar_pitch = motor_list[MOTOR_PITCH] -> target - st -> v_pitch;

			if(tar_pitch > PITCH_LH_BOUND)
				tar_pitch = PITCH_LH_BOUND;
			else if(tar_pitch < PITCH_LL_BOUND)
				tar_pitch = PITCH_LL_BOUND;
			MotorSetTar (motor_list[MOTOR_PITCH], tar_pitch, ABS);

			st -> mouse_flag = 0;
			st -> spin_flag = 0;
		}
	    else
		{
			//目标值+增量
			float tmp = motor_list[MOTOR_YAW] -> target + st -> v_yaw ;
			// （0 < tmp < 2*PI) 过零点限制范围
				 while ( tmp > 2*PI )
					 tmp -= 2 * PI;
				 while ( tmp <0 )
					 tmp += 2 * PI;
//操作手云台补偿
			 if(st -> mouse_flag && st ->spin_flag != 0 )
			 {
			 	tmp = MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN);
			 }

			MotorSetTar (motor_list[MOTOR_YAW], tmp, ABS);

			tmp = motor_list[MOTOR_PITCH] -> target - st -> v_pitch;
			//pitch限制范围，绝对位置---有待优化，上限角度限死
			if(tmp > PITCH_LH_BOUND)
				tmp = PITCH_LH_BOUND;
			else if(tmp < PITCH_LL_BOUND)
				tmp = PITCH_LL_BOUND;
			MotorSetTar (motor_list[MOTOR_PITCH], tmp, ABS);
		}
#endif

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
		MotorSetTar (motor_list[MOTOR_SHT_L], 0, ABS);
		MotorSetTar (motor_list[MOTOR_SHT_R], 0, ABS);
	}
	//sw2 == 2; 摩擦轮启动但拨弹盘不旋转
	if ( st -> shooter == STOP )
	{
		MotorSetTar (motor_list[MOTOR_AMMO], 0, ABS);
	}

	else if( st -> shooter == SHOOT )
	{
		#ifdef USE_REMOTE
		float tar = st -> v_shoot * 4 * PI /36; // 实际弹频 = v_shoot/36（拨弹盘一圈）/10(任务周期) *8  k/s
		if(tar > 0)
		{
			tar = -tar;
		}
		#endif

		#ifdef USE_KEYBOARD
		float tar = 1870 * 2 * PI /36; // 实际弹频 = v_shoot/36（拨弹盘一圈）/10(任务周期) *8  k/s
		if(tar > 0)
		{
			tar = -tar;
		}
		#endif
		MotorSetTar (motor_list[MOTOR_AMMO], tar, ABS);
	}
//自瞄+火控
#ifdef USE_KEYBOARD
	else if (st -> shooter == AUTO_SHOOT)
#endif

#ifdef USE_REMOTE
	else if ( st -> shooter == AUTO && st -> shoot_advice)
#endif
	{
		float tar = 3960 * 2 * PI /36;  //固定值
		if(tar > 0)
		{
			tar = -tar;
		}
		MotorSetTar (motor_list[MOTOR_SHT_L], BULLET_V, ABS);
		MotorSetTar (motor_list[MOTOR_SHT_R], - BULLET_V, ABS);
		MotorSetTar (motor_list[MOTOR_AMMO], tar, ABS);
	}

	else if (st -> shoot_advice && st -> shooter == AUTO_FIRE)
	{
		float tar = 2310 * 2 * PI /36;  //固定值
		if(tar > 0)
		{
			tar = -tar;
		}

		MotorSetTar (motor_list[MOTOR_AMMO], tar, ABS);
	}
	else
	{
		MotorSetTar (motor_list[MOTOR_AMMO], 0, ABS);
	}

// 自瞄/操作手 两套PID
// 	if(gimbal_cmd.shooter == AUTO || gimbal_cmd.shooter == AUTO_SHOOT || gimbal_cmd.shooter == AUTO_FIRE)
// 				motor_list[MOTOR_YAW] -> pid[ANGLE].Kp = 10;
// 	else
// 		motor_list[MOTOR_YAW] -> pid[ANGLE].Kp = 8.5;
}


void GimbalMotorPosi_Init (void)
{
	MotorSetTar (motor_list[MOTOR_YAW], MotorGetVal (motor_list[MOTOR_YAW], IMU, RAD), ABS);
	MotorSetTar (motor_list[MOTOR_PITCH], MotorGetVal (motor_list[MOTOR_PITCH], IMU, RAD), ABS);
	MotorSetTar (motor_list[MOTOR_AMMO], 0, ABS);
}


void GimbalTask (void *argument)
{
	GimbalMotorPosi_Init ();
	uint32_t time = osKernelGetTickCount ();

	for ( ;; )
	{
//更新IMU数据
		aim_packet_to_nuc.roll = imu_list[0]->angle[1];
	    aim_packet_to_nuc.yaw = imu_list[0]->angle[0];
	    aim_packet_to_nuc.pitch = - imu_list[0]->angle[2];
//识别颜色
	    if(refree_info.Game_Robot_state.robot_id > 0 && refree_info.Game_Robot_state.robot_id < 12)
	    	aim_packet_to_nuc.detect_color = 1;//0 -- red , 1 -- blue
	    else if(refree_info.Game_Robot_state.robot_id > 100 && refree_info.Game_Robot_state.robot_id < 112)
	    	aim_packet_to_nuc.detect_color = 0;
//赋值
	    if( aim_packet_from_nuc.yaw < 1.5 && aim_packet_from_nuc.yaw > -1.5)
		{
	    	gimbal_auto.yaw = aim_packet_from_nuc.yaw + PI;//0 -- 2*PI
		}
	    if( aim_packet_from_nuc.pitch < 0.2 && aim_packet_from_nuc.pitch > -0.2)
	    {
	    	gimbal_auto.pitch = aim_packet_from_nuc.pitch + PI;//
	    }

		gimbal_auto.yaw_diff = aim_packet_from_nuc.yaw_diff;//0 -- 2*PI
		gimbal_auto.pitch_diff = aim_packet_from_nuc.pitch_diff;//
//火控标志位
	    if(aim_packet_from_nuc.fire_advice)
	    {
	    	gimbal_cmd.shoot_advice = 1;
	    }
	    else
	    {
	    	gimbal_cmd.shoot_advice = 0;
	    }
//云台，射击控制逻辑 （可以和遥控器区分两个函数）
		GimbalCtrl (&gimbal_cmd);
		ShooterCtrl (&gimbal_cmd);

		time = osKernelGetTickCount ();
		osDelayUntil (time + GIMBAL_TASK_PERIOD);
		gimbal_diff = osKernelGetTickCount () - time;
		
	}
}

