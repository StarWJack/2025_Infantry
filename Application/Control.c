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
#include "ui.h"
#include "ramp.h"
//谨慎使用浮点数的全局变量
osThreadId_t control_task_handel;
uint8_t Control_diff;

uint16_t highWaterMark;
uint8_t y_1;
uint8_t y_2;
uint8_t y_3;
uint8_t y_4;

uint8_t shoot_flag;
uint8_t shoot_test;

void ControlInit (void)
{
	osThreadAttr_t attr = {
		.name = "ControlTask",
		.stack_size = 128 * 8 ,
		.priority = (osPriority_t) osPriorityAboveNormal ,};
		
	control_task_handel = osThreadNew (ControlTask, NULL, &attr);

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
	float SPEED_LMT;

//速度缩放系数,结合斜坡的步长调整

//结合场地，调节最大速度

#ifdef GAME_THREE
	if(RC_Ctl . keyboard . X)
		SPEED_LMT = 0.010f;

	else if(refree_info.Game_Robot_state.robot_level > 0 && refree_info.Game_Robot_state.robot_level < 3)
	{
		SPEED_LMT = 0.003f;
	}
	else if(refree_info.Game_Robot_state.robot_level >= 3 && refree_info.Game_Robot_state.robot_level < 6)
	{
		SPEED_LMT = 0.004f;
	}
	else if(refree_info.Game_Robot_state.robot_level >= 6 && refree_info.Game_Robot_state.robot_level < 9)
	{
		SPEED_LMT = 0.005f;
	}
	else if(refree_info.Game_Robot_state.robot_level >= 9 && refree_info.Game_Robot_state.robot_level < 11)
	{
		SPEED_LMT = 0.006f;
	}
	else
	{
		SPEED_LMT = 0.006f;
	}
#endif

#ifdef GAME_ONE
	if(RC_Ctl . keyboard . X)
		SPEED_LMT = 0.010f;
	else
		SPEED_LMT = 0.008f;
#endif

	if(RC_Ctl . keyboard . Q)
    	supercap_tx_data.systemRestart = 1;
    else
		supercap_tx_data.systemRestart = 0;

	gim -> v_yaw   = -(float) RC_Ctl . keyboard . x * KEYBOARD_YAW_SEN; //x -- 鼠标左右
	gim -> v_pitch =  (float) RC_Ctl . keyboard . y * KEYBOARD_PITCH_SEN;//y -- 上下

	chs -> vx = ( Ramp_data1(RC_Ctl . keyboard . W) * (-SPEED_LMT) ) +  ( Ramp_data2(RC_Ctl . keyboard . S) * ( SPEED_LMT) ) ;
	chs -> vy = ( Ramp_data3(RC_Ctl . keyboard . A) * ( SPEED_LMT) ) +  ( Ramp_data4(RC_Ctl . keyboard . D) *  (-SPEED_LMT) ) ;
//云台超调补偿
	if(RC_Ctl . keyboard . x < 5 && RC_Ctl . keyboard . x > -5 && RC_Ctl . keyboard . SHIFT != 1)
		gim -> mouse_flag = 1;
	else
		gim -> mouse_flag = 0;

//开火标志位
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
		if(RC_Ctl . keyboard . l == 0 && RC_Ctl . keyboard . r == 0)
			gim -> shooter = STOP;//
		else if(RC_Ctl . keyboard . l ==1 &&  RC_Ctl . keyboard . r == 0)
			gim -> shooter = SHOOT;//仅射击
		else if(RC_Ctl . keyboard . l == 0 && RC_Ctl . keyboard . r == 1 && RC_Ctl . keyboard . V == 0)
			gim -> shooter = AUTO;//仅云台跟随
		else if(RC_Ctl . keyboard . l == 1 && RC_Ctl . keyboard . r == 1 && RC_Ctl . keyboard . V == 0)
			gim -> shooter = AUTO_SHOOT;//自瞄+手打
		else if(RC_Ctl . keyboard . l == 0 && RC_Ctl . keyboard . r == 1 && RC_Ctl . keyboard . V == 1)
			gim -> shooter = AUTO_FIRE;//自瞄+火控
	}
	else if(shoot_flag == 0)
	{
		gim -> friction_state = 0;
		gim -> shooter = STOP;
	}

	if ( RC_Ctl . keyboard . SHIFT )
	{
		chs -> mode = SPIN;//等级判断

#ifdef GAME_THREE           //3v3设置
		if(refree_info.Game_Robot_state.robot_level > 0 && refree_info.Game_Robot_state.robot_level < 3)
		{
			chassis_cmd.omega_z = 0.5;
		}
		else if(refree_info.Game_Robot_state.robot_level >= 3 && refree_info.Game_Robot_state.robot_level < 6)
		{
			chassis_cmd.omega_z = 0.7;
		}
		else if(refree_info.Game_Robot_state.robot_level >= 6 && refree_info.Game_Robot_state.robot_level < 9)
		{
			chassis_cmd.omega_z = 0.8;
		}
		else if(refree_info.Game_Robot_state.robot_level >= 9 && refree_info.Game_Robot_state.robot_level < 11)
		{
			chassis_cmd.omega_z = 1.0;
		}
		else
		{
			chassis_cmd.omega_z = 0.6;
		}
#endif

#ifdef GAME_ONE          //1v1设置
		chassis_cmd.omega_z = 1.1;
#endif
		gim -> spin_flag = 1;
	}
	else
	{
		chs -> mode = FOLLOW;
	}

	if(RC_Ctl . keyboard . Z)
	{
		ui_reinit();
	}

	if(supercap_rx_data.capEnergy < 60)
	{
		SPEED_LMT *= 0.8;
		chassis_cmd.omega_z *= 0.8;
	}
}

//canbus--RC_Ctl.rc.sw1 == 1 || RC_Ctl.rc.sw1 == 3 是开启状态
void RemoteCtrl (Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)
{

	chs -> vx = (float)- RC_Ctl . rc . ch2 * REMOTE_X_SEN;
	chs -> vy = (float)- RC_Ctl . rc . ch1 * REMOTE_Y_SEN;

	gim -> v_yaw = (float) - RC_Ctl . rc . ch3 * REMOTE_YAW_SEN;
	gim -> v_pitch = (float) - RC_Ctl . rc . ch4 * REMOTE_PITCH_SEN;
	gim -> v_shoot = (float) RC_Ctl . rc .wheel * 2;
	
	if(RC_Ctl . rc . sw1 == 1)
	{
		chs -> mode = FOLLOW;
	}
	else if(RC_Ctl . rc . sw1 == 3)
	{
		chs -> mode = SYOP;
	}

	//sw1
//	if( RC_Ctl . rc . sw1 == 0 )
//		supercap_tx_data.enableDCDC = 0;
//	else
//		supercap_tx_data.enableDCDC = 1;//超电使能
	
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
	Send_Packet_Init(&aim_packet_to_nuc);
	uint32_t time = osKernelGetTickCount ();

	for ( ;; )
	{
		//收
		UnPack_Data_ROS2(buf_receive_from_nuc, &aim_packet_from_nuc, sizeof (aim_packet_from_nuc));
//模式选择
		if(RC_Ctl.rc.sw1 ==0 || RC_Ctl.rc.sw1 == 2)
		{
			KeyboardCtrl(&gimbal_cmd, &chassis_cmd);
		}
		else
		{
			RemoteCtrl (&gimbal_cmd, &chassis_cmd);
		}
		//云台坐标系转底盘坐标系
		ChassisCmdTrans (&chassis_cmd, CHASSIS_ZEROPOINT, MotorGetVal (motor_list[MOTOR_YAW], MOTORANGLE, ORIGIN));
		
//		vofa_demo(motor_list[4]->target,
//				  motor_list[4]->output,
//				  motor_list[4]->data.current,
//				  motor_list[4]->data.temp,
//				  supercap_rx_data.errorCode,
//				  supercap_rx_data.capEnergy);

//		vofa_demo(refree_info.Power_Heat_Data.buffer_energy,
//		 		   refree_info.Game_Robot_state.chassis_power_limit,
//		 		   supercap_rx_data.refreePower,
//		            supercap_rx_data.capEnergy,
//		 		   supercap_rx_data.chassisPower,60);

//		vofa_demo(chassis . output,
//					chassis.pid[INNER].Output,
//					chassis.pid[INNER].i_out,
//					chassis.pid[INNER].p_out,
//					chassis.pid[OUTER].target,
//					chassis.pid[OUTER].measure);

	    // vofa_demo(motor_list[MOTOR_YAW] -> pid[ANGLE].target,
	    // 		motor_list[MOTOR_YAW] -> pid[ANGLE].measure,


	 	// 	  motor_list[MOTOR_PITCH] -> pid[ANGLE].target,
	 	// 	  motor_list[MOTOR_PITCH] -> pid[ANGLE].measure,

	 	// 	  0,
	 	// 	  motor_list[MOTOR_YAW] -> output);


//		vofa_demo((MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN)  + gimbal_auto.yaw_diff),
//				   MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN),
//				   gimbal_auto.yaw_diff,
//				   (MotorGetVal (motor_list[MOTOR_PITCH], IMU, ORIGIN)  + gimbal_auto.pitch_diff),
//				   MotorGetVal (motor_list[MOTOR_PITCH], IMU, ORIGIN),
//				   gimbal_auto.pitch_diff);

//		vofa_demo( (gimbal_auto.yaw ),
//				   MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN),
//				   gimbal_auto.yaw,
//				   (gimbal_auto.pitch),
//				   MotorGetVal (motor_list[MOTOR_PITCH], IMU, ORIGIN),
//				   gimbal_auto.pitch);

//		vofa_demo(motor_list[2]->target,
//				  motor_list[3]->target,
//				  motor_list[4]->target,
//				  motor_list[5]->target,
//				  chassis_cmd.vx,
//				  chassis_cmd.vy);

//		vofa_demo(
//				  motor_list[3]->pid[1].target,
//				  motor_list[3]->pid[1].measure,
//				  motor_list[4]->pid[1].target,
//				  motor_list[4]->pid[1].measure,
//				  motor_list[5]->pid[1].target,
//				  motor_list[5]->pid[1].measure);
		//打包发送
		Pack_And_Send_Data_ROS2 (&aim_packet_to_nuc);

		Control_diff = osKernelGetTickCount() - time;
		time = osKernelGetTickCount ();
		osDelayUntil (time + CONTROL_TASK_PERIOD);
	}
}
