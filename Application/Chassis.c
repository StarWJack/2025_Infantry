#include "Chassis.h"
#include "string.h"
#include "config.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "Control.h"


osThreadId_t chassis_task_handel;

Chassis_TypeDef chassis;
Chassis_CmdTypedef chassis_cmd;

void ChassisTask (void *arguement);

#if INFANTRY

/*
 * @brief  	全向轮解算
 * @param	底盘控制结构体指针
 * @retval 	float[4], 各轮角速度
 * @note:   仅考虑全向轮安装角为45°且轴长相等的情况
 */
void OnmiSolve (Chassis_CmdTypedef *cmd, float *ret)
{
	/*         vx
				^
				|
		   1//      \\2
		   // \    / \\
			   top      -->vy
		   \\ /    \ //
		   4\\      //3
					
	*/
	float omega_z = cmd -> omega_z;
	ret[0] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (- cmd -> vx - cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
	ret[1] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (cmd -> vx - cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
	ret[2] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (cmd -> vx + cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
	ret[3] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (- cmd -> vx + cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
}

/*
 * @brief  	全向轮逆解算(3508电机)
 * @param	底盘四电机结构体指针，顺序1-4
 * @retval 	底盘自旋速度，rps
 * @todo    按需增加对里程的解算
 */
float ChassisGetOmega (void)
{
	float ret = 0;
	for ( int i = MOTOR_CHS_1; i < 4; i ++ )
		ret += MotorGetVal (motor_list[i], MOTORSPEED, ORIGIN);
	ret = ret / 60;    //RPM to RPS
	ret = (ret * 187.0f / 3591.0f) / 4;    //转换到输出端转速
	ret = ret * (WHEEL_RADIUS / CHASSIS_RADIUS);
	return ret;
}

#endif

#if HERO
//TODO(GUATAI)底盘待优化
/*
 * @brief  	麦轮运动学逆解算(3508电机)
 * @param	底盘控制结构体指针
 * @retval 	float[4], 各轮角速度
 * @note:   仅考虑麦轮俯视打叉，仰视画圆且轴长不等的情况,以辊子与地面接触为准
 */
void MecanumSolve( Chassis_CmdTypedef *cmd, float *ret, float omega_z )
{
//		/*      vx				    0( a,-b)   1( a, b)
//		      ^						2(-a, b)   3(-a,-b)
//		      |
//		 ||       ||
//		0//       \\1		       (-1 -1 a+b)     (w0)
//		 || \   / ||		       ( 1 -1 a+b)(vx) (w1)
//		     top     -->vy      1/s(         )(vy)=(  )
//		 || /   \ ||		       ( 1  1 a+b)(wz) (w2)
//		3\\       //2		       (-1  1 a+b)     (w3)
//		 ||       ||
//		 */
	ret[0] = ( cmd->omega_z * PI * ( LENGTH + WIDTH ) - ( -cmd->vx - cmd->vy ) ) / WHEEL_RADIUS;
	ret[1] = ( cmd->omega_z * PI * ( LENGTH + WIDTH ) - (  cmd->vx - cmd->vy ) ) / WHEEL_RADIUS;
	ret[2] = ( cmd->omega_z * PI * ( LENGTH + WIDTH ) - (  cmd->vx + cmd->vy ) ) / WHEEL_RADIUS;
	ret[3] = ( cmd->omega_z * PI * ( LENGTH + WIDTH ) - ( -cmd->vx + cmd->vy ) ) / WHEEL_RADIUS;
}

/*
 * @brief  	麦轮运动学正解算(车体转速)
 * @param	底盘四电机结构体指针，顺序0-3
 * @retval 	底盘自旋速度，rps
 * @todo    按需增加对里程的解算
 */
float ChassisGetOmega( Motor_TypeDef *motor[4] )
{
//		/*         vx       vx=(-w0+w1+w2-w3)*s/4
//		      ^				vy=(-w0-w1+w2+w3)*s/4
//		      |				wz=(w0+w1+w2+w3)/a+b*s/4
//		 ||       ||
//		0//       \\1		 						     (w0)
//		 || \   / ||		    ( -1     1     1    -1  )(w1) (vx)
//	         top     -->vy   s/4( -1    -1     1     1  )(  )=(vy)
//		 || /   \ ||		    (1/a+b 1/a+b 1/a+b 1/a+b)(w2) (wz)
//		3\\       //2								     (w3)
//		 ||       ||
//		 */
	float ret = 0;
	for( int i = 0; i < 4; i++ )
	{
		ret += MotorGetVal( motor[i], ORIGIN );    //RPM to RPS
	}
	ret = ret / 60;
	ret = ( ret * 187.0f / 3591.0f ) / 4;    //转换到输出端转速
	ret = ret * 0.5f * ( LENGTH + WIDTH );
	return ret;
//	PID_Position(&PID_Motor_Angle[Chassis_Angle_ID], Motor[Motor_Yaw_ID].angle, 0);
//	//底盘角度环,以底盘与云台的相对角度为输入
//	omega = PID_Motor_Angle[Chassis_Angle_ID].Output;
}
#endif


void ChassisInit (void)
{
	osThreadAttr_t task_attr = {.name = "ChassisTask"};
	chassis_task_handel = osThreadNew (ChassisTask, NULL, &task_attr);
}

void ChassisTask (void *arguement)
{
	 static float solve[4];
//	底盘跟随测试
	 PID_TypeDef pid_outer = {
	 		.Kp = 0.001,
	 		.Ki = 0,
	 		.Kd = 0,
	 		.Kf = 0,
	 		.Output_Max = 1,
	 		.Err_sum_Max = 5000
	 };
	
	 PID_TypeDef pid_inner = {
	 		pid_inner . Kp = 15,
	 		pid_inner . Ki = 1,
	 		pid_inner . Kd = 0,
	 		.Kf = 0,
	 		.Output_Max = 0.5,//量程：RPS
	 		.Err_sum_Max = 10
	 };
	
	 chassis . pid[INNER] = pid_inner;
	 chassis . pid[OUTER] = pid_outer;
	uint32_t time = osKernelGetTickCount ();
	
	for ( ;; )
	{
		chassis . omega_z_ref = ChassisGetOmega ();
		motor_list[MOTOR_YAW] -> feed_fwd = chassis . omega_z_ref;

		if ( chassis_cmd . mode == SPIN )
		{
			chassis_cmd.omega_z = 0.8;//固定小陀螺转速
		}
		else if ( chassis_cmd . mode == FOLLOW )
		{
			chassis.target = MotorGetVal (motor_list[MOTOR_YAW], ANGLE, ORIGIN);
			chassis.value  = CHASSIS_ZEROPOINT;
			while(chassis.target - chassis.value > 4096)
					chassis.target -= 8192.0f;
			while(chassis.target - chassis.value < -4096)
					chassis.target += 8192.0f;
			//双环控制+500死区
			if( (chassis.target - chassis.value) < 500 && (chassis.target - chassis.value) > -500)
			{
				chassis.output = 0;
			}
			else
			{
				chassis.output = PID_Position (&chassis . pid[OUTER], chassis.value,chassis.target);
				chassis.target = chassis . output;
				chassis.value  = chassis . omega_z_ref;//角速度
				chassis.output = PID_Increment (&chassis . pid[INNER], chassis.value,chassis.target);
			}
			//如果不想跟随，注释下面底盘输出即可
			chassis_cmd.omega_z = - chassis . output;
		}
		else
		{
			chassis_cmd.omega_z = 0;
		}
#if INFANTRY
		OnmiSolve (&chassis_cmd, solve);
#elif HERO
		MecanumSolve(&chassis_cmd, solve,0);
#endif
		for ( int i = 0; i < 4; i ++ )
			MotorSetTar (motor_list[i + MOTOR_CHS_1], solve[i], ABS);
		
		time = osKernelGetTickCount ();
		osDelayUntil (time + CHASSIS_TASK_PERIOD);
	}
}
