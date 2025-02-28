#include <stdlib.h>
#include "Control.h"
#include "cmsis_os2.h"
#include "remote.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "config.h"
#include "motor.h"
#include "imu.h"
#include "math.h"
#include "WatchDog.h"
#include "vofa.h"
#include "CanBus.h"
#include "usbd_cdc_if.h"
#include "Serial.h"
#include "referee.h"
#include "usart.h"
#include "SuperCap.h"

osThreadId_t control_task_handel;

uint8_t shoot_flag;

float Ramp_data(uint16_t val)
{
	float tar = 0;
	if(val)
	{
		tar += 0.2;
		if(tar >= 1)
		{
			tar = 1;
		}
	}
	else
	{
		tar -= 0.2;
		if(tar < 0)
		{
			tar = 0;
		}
	}
	return tar;
}


void ControlInit (void)
{
	osThreadAttr_t attr = {
		.name = "ControlTask",
		.stack_size = 128 * 4 ,
		.priority = (osPriority_t) osPriorityAboveNormal ,};
		
	control_task_handel = osThreadNew (ControlTask, NULL, &attr);

//    WatchDogInit();
}

/*
 * @brief  	底盘坐标系转云台坐标系
 * @param	底盘命令结构体指针
 * @param   底盘坐标系中，云台yaw与底盘x平行时yaw轴电机角度（编码值）
 * @param   yaw电机当前角度（编码值）
 * @retval 	无
 */
void ChassisCmdTrans (Chassis_CmdTypedef *cmd, float chs_zeropoint, float gim_ang)
{
	float bias_tmp = (gim_ang - chs_zeropoint + 8192.0f);
	if ( bias_tmp > 8192 )
		bias_tmp -= 8192;
	else if ( bias_tmp < 0 )
		bias_tmp += 8192;

	bias_tmp = bias_tmp / 8192 * 2 * PI;
	
	float vx_tmp = cmd -> vx * cosf (bias_tmp) + cmd -> vy * sinf (bias_tmp);
	float vy_tmp = - cmd -> vx * sinf (bias_tmp) + cmd -> vy * cosf (bias_tmp);
	cmd -> vx = vx_tmp;
	cmd -> vy = vy_tmp;
}


void KeyboardCtrl (Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)
{
	gim -> v_yaw   = -(float) RC_Ctl . keyboard . x * KEYBOARD_YAW_SEN; //x -- 鼠标左右

	gim -> v_pitch =  (float) RC_Ctl . keyboard . y * KEYBOARD_PITCH_SEN;//y -- 上下


	chs -> vx = ( Ramp_data(RC_Ctl . keyboard . W) * (-CHS_SPEED_LMT) ) +  ( Ramp_data(RC_Ctl . keyboard . S) * ( CHS_SPEED_LMT) ) ;
	chs -> vy = ( Ramp_data(RC_Ctl . keyboard . A) * ( CHS_SPEED_LMT) ) +  ( Ramp_data(RC_Ctl . keyboard . D) * (-CHS_SPEED_LMT) ) ;

	//开火
	if ( RC_Ctl . keyboard . R )
	{
		shoot_flag = 1;
	}
	else if( RC_Ctl . keyboard . E )
	{
		shoot_flag = 0;
	}

	if ( shoot_flag == 1)
	{
		gim -> friction_state = 1;
		if(RC_Ctl . keyboard . l )
			gim -> shooter = SHOOT;
		else if(RC_Ctl . keyboard . l == 0)
			gim -> shooter = STOP;

		else if(RC_Ctl . keyboard . r )
			gim -> shooter = AUTO;

	}
	else if(shoot_flag ==0)
	{
		gim -> friction_state = 0;
		gim -> shooter = STOP;
	}
	
	if ( RC_Ctl . keyboard . SHIFT )
		chs -> mode = SPIN;
	else
		chs -> mode = FOLLOW;

}

//canbus--RC_Ctl.rc.sw1 == 1 || RC_Ctl.rc.sw1 == 3 是开启状态
void RemoteCtrl (Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)
{

	chs -> vx = (float)- RC_Ctl . rc . ch2 * REMOTE_X_SEN;
	chs -> vy = (float)- RC_Ctl . rc . ch1 * REMOTE_Y_SEN;

	gim -> v_yaw = (float) - RC_Ctl . rc . ch3 * REMOTE_YAW_SEN;
	gim -> v_pitch = (float) - RC_Ctl . rc . ch4 * REMOTE_PITCH_SEN;
	gim -> v_shoot = (float) RC_Ctl . rc .wheel * 2;
	
	chs -> mode = RC_Ctl . rc . sw1;
	//sw2的控制模式，sw1在chassis
	if ( RC_Ctl . rc . sw2 == 2 || RC_Ctl . rc . sw2 == 0 )
	{
		gim -> shooter = STOP;
		gim -> friction_state = 0;
	}
	else
	{
		gim -> friction_state = 1;
		if ( RC_Ctl . rc . sw2 == 3 )
			gim -> shooter = SHOOT;//测试，小拨盘上下拨动发弹
		else if ( RC_Ctl . rc . sw2 == 1 )
			gim -> shooter = AUTO;//泼水
	}
}


void ControlTask (void *argument)
{
	RemoteInit ();

	Send_Packet_Init(&aim_packet_to_nuc);
	uint32_t time = osKernelGetTickCount ();

	for ( ;; )
	{
		//收
		UnPack_Data_ROS2(buf_receive_from_nuc, &aim_packet_from_nuc, sizeof (aim_packet_from_nuc));

#ifdef USE_REMOTE
		RemoteCtrl (&gimbal_cmd, &chassis_cmd);
#endif

#ifdef USE_KEYBOARD
		KeyboardCtrl(&gimbal_cmd, &chassis_cmd);
#endif
		//云台坐标系转底盘坐标系
		ChassisCmdTrans (&chassis_cmd, CHASSIS_ZEROPOINT, MotorGetVal (motor_list[MOTOR_YAW], MOTORANGLE, ORIGIN));
		
//		vofa_demo(MotorGetVal (motor_list[2], MOTORSPEED, RAD),
//				  MotorGetVal (motor_list[3], MOTORSPEED, RAD),
//				  MotorGetVal (motor_list[4], MOTORSPEED, RAD),
//				  MotorGetVal (motor_list[5], MOTORSPEED, RAD),
//				  chassis . omega_z_ref,
//				  moto_total);
//		vofa_demo((MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN)  + gimbal_auto.yaw_diff),
//				   MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN),
//				   (MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN)  + gimbal_auto.yaw_diff),
//				   MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN),1,1);

		vofa_demo(refree_info.Power_Heat_Data.buffer_energy,
				   refree_info.Game_Robot_state.chassis_power_limit,
				   supercap_rx_data.refreePower,
		           supercap_rx_data.capEnergy,
				   supercap_rx_data.chassisPower,60);

		//打包发送
		Pack_And_Send_Data_ROS2 (&aim_packet_to_nuc);

		time = osKernelGetTickCount ();
		osDelayUntil (time + CONTROL_TASK_PERIOD);
	}
}
