#include "motor.h"
#include "pid.h"
#include "IMU.h"
#include "string.h"
#include "config.h"
#include "CanInst.h"
#include "can.h"
#include "ErrorHandel.h"
#include "malloc.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "ChassisPowerCtrl.h"
#include "SuperCap.h"
#include "referee.h"


Motor_TypeDef *motor_list[MOTOR_NUM];
uint32_t calu_auto;

/*
 * @brief  	计算PID并限幅
 * @param	电机结构体指针数组
 * @retval 	无，结果在结构体中
 * @note：运行频率为1000Hz
 */
//PID输出
void MotorCalc (void)
{
	static float tar = 0, ref = 0;
//IMU控制
	tar = motor_list[MOTOR_YAW] -> target;

	ref = MotorGetVal (motor_list[MOTOR_YAW], IMU, ORIGIN);//读IMU时已经做过处理，无需get-rad

	while(tar-ref > PI)
	 	ref += 2 * PI ;
	while(tar-ref < -PI)
	 	ref -= 2 * PI ;

	motor_list[MOTOR_YAW] -> output_Position = PID_Position (&(motor_list[MOTOR_YAW] -> pid[ANGLE]), ref, tar);
	motor_list[MOTOR_YAW] -> output = motor_list[MOTOR_YAW] -> output_Position;

	tar = motor_list[MOTOR_YAW] -> output_Position + (2*PI *motor_list[MOTOR_YAW] -> feed_fwd);//底盘跟随时的角速度（力补偿） rad/s

//	tar = motor_list[MOTOR_YAW] -> output_Position;

	ref = MotorGetVal (motor_list[MOTOR_YAW], MOTORSPEED, RAD);

	FeedForward_Input(&(motor_list[MOTOR_YAW] -> pid[SPEED]) , tar);//前馈

	motor_list[MOTOR_YAW] -> output = PID_Increment (&(motor_list[MOTOR_YAW] -> pid[SPEED]), ref, tar) ;
	motor_list[MOTOR_YAW] -> output += motor_list[MOTOR_YAW] -> pid[SPEED].Output_feed;

//单增量式
//	  tar = motor_list[MOTOR_YAW] -> target;
//	  ref = MotorGetVal (motor_list[MOTOR_YAW], MOTORSPEED, RAD); //motor->data.speed can线返回的速度
//	  FeedForward_Input(&(motor_list[MOTOR_YAW] -> pid[SPEED]) , tar);
//
//	  motor_list[MOTOR_YAW] -> output = PID_Increment (&(motor_list[MOTOR_YAW] -> pid[SPEED]), ref, tar);
//	  motor_list[MOTOR_YAW] -> output += motor_list[MOTOR_YAW] -> pid[SPEED].Output_feed;

	//#define MOTOR_PITCH 0  IMU控制
	tar = motor_list[MOTOR_PITCH] -> target;
	ref = MotorGetVal (motor_list[MOTOR_PITCH], IMU, RAD);
	motor_list[MOTOR_PITCH] -> output = PID_Position (&(motor_list[MOTOR_PITCH] -> pid[ANGLE]), ref, tar);
	motor_list[MOTOR_PITCH] -> output *= - 1.0f;//运动方向为负

	tar = motor_list[MOTOR_PITCH] -> output;
	ref = MotorGetVal (motor_list[MOTOR_PITCH], MOTORSPEED, RAD);
	motor_list[MOTOR_PITCH] -> output = PID_Increment (&(motor_list[MOTOR_PITCH] -> pid[SPEED]), ref, tar);
	
	//#define MOTOR_CHS_1 2
	//#define MOTOR_CHS_2 3
	//#define MOTOR_CHS_3 4
	//#define MOTOR_CHS_4 5
	for ( uint8_t i = MOTOR_CHS_1; i <= MOTOR_CHS_4; i ++ )
	{
		//tar = 0;
		tar = motor_list[i] -> target;
		ref = MotorGetVal (motor_list[i], MOTORSPEED, RAD);
		tar = tar * 3591 / 187;
		motor_list[i] -> output = PID_Increment (
				&(motor_list[i] -> pid[SPEED]), MotorGetVal (motor_list[i], MOTORSPEED, RAD), tar
		                                        );
	}
	
//	Chassis_PowerCtrl();

	//#define MOTOR_AMMO 6
	tar = motor_list[MOTOR_AMMO] -> target;
	ref = MotorGetVal (motor_list[MOTOR_AMMO], MOTORSPEED, RAD);//rad/s
	motor_list[MOTOR_AMMO] -> output = PID_Increment (&motor_list[MOTOR_AMMO] -> pid[SPEED], ref, tar);
	
	//#define MOTOR_SHT_L 7
	//#define MOTOR_SHT_R 8
	for ( uint8_t i = MOTOR_SHT_L; i <= MOTOR_SHT_R; i ++ )
	{
		tar = motor_list[i] -> target;
		ref = MotorGetVal (motor_list[i], MOTORSPEED, RAD);
		tar = tar * 3591 / 187;
		motor_list[i] -> output = PID_Increment (
				&(motor_list[i] -> pid[SPEED]), MotorGetVal (motor_list[i], MOTORSPEED, RAD), tar
		                                        );
	}
}

/*
 * @brief  	基于缓冲能量的功率控制,当超电失效时
 * @retval 
 */
void Chassis_PowerCtrl(void)
{
	float k_power = 1;
	float Power_limit = refree_info.Game_Robot_state.chassis_power_limit;
	if(supercap_rx_data.errorCode || supercap_tx_data.enableDCDC == 0)
	{
		if(supercap_rx_data.chassisPower > refree_info.Game_Robot_state.chassis_power_limit)
		{
			if(refree_info.Power_Heat_Data.buffer_energy <10)
				k_power = 0.10;
			else if(refree_info.Power_Heat_Data.buffer_energy < 25)
				k_power = 0.20;
			else if(refree_info.Power_Heat_Data.buffer_energy < 30)
				k_power = 0.35;
			else if(refree_info.Power_Heat_Data.buffer_energy < 35)
				k_power = 0.50;
			else if(refree_info.Power_Heat_Data.buffer_energy < 40)
				k_power = 0.65;
			else if(refree_info.Power_Heat_Data.buffer_energy < 45)
				k_power = 0.80;
			else if(refree_info.Power_Heat_Data.buffer_energy < 50)
				k_power = 0.90;

			for(uint8_t i = MOTOR_CHS_1; i <= MOTOR_CHS_4; i ++)
			{
				motor_list[i] -> output *= k_power*(Power_limit)/supercap_rx_data.chassisPower;

			}
		}
	}
}


/*
 * @brief  	获取电机当前值
 * @param	电机结构体地址
 * @param   style=ORIGIN, 读取原始数据；style=RAD, 弧度制
 * @retval 	电机当前值/是否过度封装/
 */

float MotorGetVal (Motor_TypeDef *motor, MOTOR_Feedback_e feedback, DataType_e type)
{
	float val;
	if ( feedback == MOTORANGLE )
		if ( type == ORIGIN )
			val = motor -> data . angle;
		else
			val = (((float) motor -> data . angle / 8192.0f) * 2 * PI);
	else if ( feedback == MOTORSPEED )
		if ( type == ORIGIN )
			val = motor -> data . speed;
		else
			val = motor -> data . speed * 2 * PI / 60.0f;//RPM->rad/s
	else
		val = motor -> value (motor);//调用 motor 结构体中的 value 函数，并将 motor 作为参数传递给该函数，然后将返回值赋给 val 变量
		                             //如果feedback不是MOTORANGLE或MOTORSPEED，那么就调用motor结构体中的value函数(当前代码在MotorRegist和MotorInit定义)
	return val;
}

float MotorGetIMU0_Yaw (Motor_TypeDef *motor)
{
	return imu_list[0] -> yaw (imu_list[0]); //已做过+PI处理
}

float MotorGetIMU0_Pitch (Motor_TypeDef *motor)
{
	return imu_list[0] -> pitch (imu_list[0]);
}


void MotorCanCbk (Can_InfoTypedef *st, uint8_t *rxdata, uint8_t list_id)
{
	motor_list[list_id] -> data . angle = (uint16_t) (rxdata[0] << 8) + rxdata[1];
	motor_list[list_id] -> data . speed = (int16_t) (rxdata[2] << 8) + rxdata[3];
	motor_list[list_id] -> data . current = (int16_t) (rxdata[4] << 8) + rxdata[5];
	motor_list[list_id] -> data . temp = rxdata[6];
}

/*
 * @brief  	电机发送报文
 * @param	can信息结构体
 * @param	发送的16位数据指针
 * @param   4个ID(参考电机手册发送格式)
 * @retval 	无
 */
void MotorFillMsg (Can_InfoTypedef *st, uint8_t *txdata, uint8_t list_id)
{
	uint8_t tmp_id = st -> CAN_id > 4 ? (st -> CAN_id) - 4 : st -> CAN_id;
	txdata[2 * (tmp_id - 1)] = (int16_t) motor_list[list_id] -> output >> 8;
	txdata[(2 * tmp_id) - 1] = (int16_t) motor_list[list_id] -> output;
}

/*
 * @brief  	设置电机目标值
 * @param	电机结构体指针
 * @param	目标值
 * @param   ABS->absolute target;
 *          INCR->add from perious target
 *          FEED->change feedforward
 * @retval 	无
 */
void MotorSetTar (Motor_TypeDef *motor, float val, ValSet_Type_e type)
{
	if ( type == ABS )
		motor -> target = val;
	else if ( type == INCR )
		motor -> target += val;
	else
		sErrorHandel (OUT_OF_ENUM);
}

/*
 * @brief  	停止指定或全部的电机输出
 * @param	电机结构体指针
 * @retval 	无
 * @note    传入空指针时，视为停止所有电机
 */
void MotorStop (Motor_TypeDef *motor)
{
	if ( motor == NULL)
	{
		for ( int i = 0; i < MOTOR_NUM; i ++ )
		{
			motor_list[i] -> output = 0;
			motor_list[i] -> info . enable = 0;
			PID_Clear (&motor_list[i] -> pid[INNER]);
			PID_Clear (&motor_list[i] -> pid[OUTER]);
		}
	}
	else
	{
		motor -> output = 0;
		motor -> info . enable = 0;
		PID_Clear (&motor -> pid[INNER]);
		PID_Clear (&motor -> pid[OUTER]);
	}
}

/*
 * @brief  	将电机注册至列表
 * @param	电机初始化结构体
 * @retval 	无
 */

CAN_TxHeaderTypeDef txheader = {
		.ExtId =   0x0,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
	
};

void MotorRegist (Motor_InitTypedef st)
{
	txheader . StdId = st . txheader_id;
	static Motor_TypeDef motor;
	motor . can . txheader = txheader;
	motor . can . rxheader . StdId = st . rxheader_id;
	motor . can . rxheader . DLC = 8;
	motor . can . CAN_id = st . can_id;
	motor . can . FillTxMsg = MotorFillMsg;
	motor . can . CbkHandel = MotorCanCbk;
	motor . can . hcan_x = st . hcan;
	

	Motor_InfoTypeDef info = {
			.enable = 0,
			.motor_type = st . motor_type,
			.output_max = st . output_max,//电机信息
			.pid_type = st . pid_type,
	};
	motor . info = info;
	
	motor . pid[OUTER] = st . pid_outer;
	motor . pid[INNER] = st . pid_inner;
	motor . feed_fwd = 0;
//    if(st.pid_type == DUAL_LOOP)
//  定义了一个函数指针value，存放一个函数
	motor . value = st . source;    //avoid null pointer
	
	motor_list[st . list_id] = malloc (sizeof (Motor_TypeDef));
	memcpy (motor_list[st . list_id], &motor, sizeof (Motor_TypeDef));
}

void MotorInit ()
{
//	自瞄
	PID_TypeDef pid_outer = {
//			.Kp = 12,  //大小应与遥控器输入量一同调节，p太大应引起目标点抖动
			.Kp = 6,
			.Ki = 0,
			.Kd = 0,
			.Kf = 0,
			.Output_Max = 7,
			.Err_sum_Max = 100,
	};

	PID_TypeDef pid_inner = {
//			pid_inner . Kp = 4050,  //2000-4000  ，3000-4000归中较慢
			pid_inner . Kp = 3800,
			pid_inner . Ki = 8,   //太小会回弹
			pid_inner . Kd = 0,
			pid_inner . Kf = 1,
			pid_inner . Output_Max = 16000, //5000-15000最合适
	};
//操作手
//	PID_TypeDef pid_outer = {
//			.Kp = 5,  //大小应与遥控器输入量一同调节，p太大应引起目标点抖动
//			.Ki = 0,
//			.Kd = 0,
//			.Kf = 0,
//			.Output_Max = 7,
//			.Err_sum_Max = 100,
//	};
//
//	PID_TypeDef pid_inner = {
////			pid_inner . Kp = 4050,  //2000-4000  ，3000-4000归中较慢
//			pid_inner . Kp = 3800,
//			pid_inner . Ki = 8,   //太小会回弹
//			pid_inner . Kd = 0,
//			pid_inner . Kf = 1,
//			pid_inner . Output_Max = 16000, //5000-15000最合适
//	};

	Motor_InitTypedef init = {
			.can_id = 1,
			.hcan = &hcan1,
			.list_id = MOTOR_YAW,
			.motor_type = GM6020,
			.pid_type = DUAL_LOOP,
			.source = MotorGetIMU0_Yaw,
			.txheader_id = 0x1FF,
			.rxheader_id = 0x205,
			.pid_outer = pid_outer,
			.pid_inner = pid_inner,
			.output_max = 30000
	};
	
	/* Yaw init */
	MotorRegist (init);
	
	/* Pitch init */
	init . can_id = 1;
	init . hcan = &hcan2;
	init . list_id = MOTOR_PITCH;
	init . pid_type = DUAL_LOOP;
	init . source = MotorGetIMU0_Pitch;
	
	pid_outer . Kp = 8;
	pid_outer . Ki = 0;
	pid_outer . Kd = 0;
	pid_outer . Kf = 0;
	pid_outer . Output_Max = 10;
	init . pid_outer = pid_outer;
	
	pid_inner . Kp = 2000;//太大过零点噪声极大
	pid_inner . Ki = 20;
	pid_inner . Kd = 0;
	pid_inner . Kf = 0;
	pid_inner . Output_Max = 15000;
//	pid_inner . Output_Max = 0;//改为0调试
	init . pid_inner = pid_inner;
	MotorRegist (init);
	
	/* Chassis init */
	pid_inner . Kp = 35;
	pid_inner . Ki = 0.15f;
	pid_inner . Kd = 1;
	pid_inner . Kf = 0;
	pid_inner . Err_sum_Max = 1000;
	pid_inner . Output_Max = 10000;
	init . pid_inner = pid_inner;
	init . can_id = 1;
	init . hcan = &hcan1;
	init . list_id = MOTOR_CHS_1;

	init . motor_type = M3508,
	init . pid_type = SPEED;
	init . output_max = 5000;
	init . txheader_id = 0x200;
	init . rxheader_id = 0x201;
	MotorRegist (init);
	
	init . can_id = 2;
	init . list_id = MOTOR_CHS_2;
	init . rxheader_id = 0x202;
	MotorRegist (init);
	
	init . can_id = 3;
	init . list_id = MOTOR_CHS_3;
	init . rxheader_id = 0x203;
	MotorRegist (init);
	
	init . can_id = 4;
	init . list_id = MOTOR_CHS_4;
	init . rxheader_id = 0x204;
	MotorRegist (init);
	
	//shooter init
	pid_inner . Kp = 400;
	pid_inner . Ki = 15;
	pid_inner . Kd = 300;
	pid_inner . Kf = 0;

//	pid_inner . Kp = 500;
//	pid_inner . Ki = 10;
//	pid_inner . Kd = 50;
//	pid_inner . Kf = 0;
	 pid_inner . Output_Max = 12000;
	init . pid_inner = pid_inner;
	init . hcan = &hcan2;
	init . can_id = 1;
	init . list_id = MOTOR_SHT_R;
	init . txheader_id = 0x200;
	init . rxheader_id = 0x201;
	MotorRegist (init);
	
	init . can_id = 2;
	init . list_id = MOTOR_SHT_L;
	init . rxheader_id = 0x202;
	MotorRegist (init);
	
	//ammofeeder init
	// pid_outer . Kp = 500;
	pid_outer . Kp = 0;
	pid_outer . Ki = 0;
	pid_outer . Kd = 0;
	pid_outer . Kf = 0;
	pid_outer . Output_Max = 15000;
	pid_outer . Err_sum_Max = 1000;
	pid_inner . Kp = 200;
	pid_inner . Ki = 25;
	pid_inner . Kd = 0;
	pid_inner . Kf = 0;
	 pid_inner . Output_Max = 10000;
//	pid_inner . Output_Max = 0;
	init . pid_inner = pid_inner;
	init . pid_outer = pid_outer;
	init . can_id = 7;
	init . motor_type = M2006;
	init . pid_type = AMMO;
	init . list_id = MOTOR_AMMO;
	init . txheader_id = 0x1FF;
	init . rxheader_id = 0x207;
	MotorRegist (init);
}

