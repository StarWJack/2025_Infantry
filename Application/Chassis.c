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

void ChassisInit (void)
{
	osThreadAttr_t task_attr = {
			.name = "ChassisTask",
			.stack_size = 128 * 4 ,
			.priority = (osPriority_t) osPriorityAboveNormal2 ,
	};
	chassis_task_handel = osThreadNew (ChassisTask, NULL, &task_attr);
}

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
	float omega_z = cmd -> omega_z + cmd -> omega_fllow;
	ret[0] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (- cmd -> vx - cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
	ret[1] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (cmd -> vx - cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
	ret[2] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (cmd -> vx + cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
	ret[3] = (omega_z * (2 * PI * CHASSIS_RADIUS) - (- cmd -> vx + cmd -> vy) * SQRT_2) / WHEEL_RADIUS;
}

/*
 * @brief  	全向轮逆解算(3508电机)
 * @param	底盘四电机结构体指针ID，顺序2-5
 * @retval 	底盘自旋速度，rad/s
 * @todo    按需增加对里程的解算
 */
float ChassisGetOmega (void)
{
	uint8_t i = 0;
	float v_total = 0;//四个3508转子的角速度之和
	for(i = MOTOR_CHS_1; i <= MOTOR_CHS_4; i++)
		v_total += MotorGetVal (motor_list[i], MOTORSPEED, ORIGIN);
	v_total = v_total/4;                                        //平均一个3508 ， 单位rpm
	v_total = v_total * (187/3591.0f) /60;                      // 转子 --> 转轴   rpm-->rps
	v_total = v_total * WHEEL_RADIUS / CHASSIS_RADIUS; // rps （轮子的线速度 = 底盘的线速度） 单位变换时要调内环PID
	return v_total;    //滤波
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
			chassis_cmd.omega_z = 0;//固定小陀螺转速
			chassis_cmd.omega_fllow = 0;
		}
		else if ( chassis_cmd . mode == FOLLOW )
		{
			//小陀螺转速清零
			chassis_cmd.omega_z = 0;

			chassis.target = MotorGetVal (motor_list[MOTOR_YAW], ANGLE, ORIGIN);
			chassis.value  = CHASSIS_ZEROPOINT;
			while(chassis.target - chassis.value > 4096)
					chassis.target -= 8192.0f;
			while(chassis.target - chassis.value < -4096)
					chassis.target += 8192.0f;
			//双环控制+500死区
			if( (chassis.target - chassis.value) < 300 && (chassis.target - chassis.value) > -300)
			{
				chassis.output = 0;
			}
			else
			{
//				//进行PID计算前转换为弧度制，方便计算
//				chassis.value = chassis.value *2*PI /8192;
//				chassis.target = chassis.target *2*PI /8192;

				chassis.output = PID_Position (&chassis . pid[OUTER], chassis.value,chassis.target);
				chassis.target = chassis . output;
				chassis.value  = chassis . omega_z_ref;//角速度
				chassis.output = PID_Increment (&chassis . pid[INNER], chassis.value,chassis.target);
			}
			//如果不想跟随，注释下面底盘输出即可
			chassis_cmd.omega_fllow = - chassis . output;
		}
		else
		{
			chassis_cmd.omega_z = 0;
			chassis_cmd.omega_fllow = 0;
		}
		OnmiSolve (&chassis_cmd, solve);
		
		for ( int i = 0; i < 4; i ++ )
			MotorSetTar (motor_list[i + MOTOR_CHS_1], solve[i], ABS);
		
		time = osKernelGetTickCount ();
		osDelayUntil (time + CHASSIS_TASK_PERIOD);
	}
}
