#include "Chassis.h"
#include "string.h"
#include "config.h"
#include "cmsis_os2.h"
#include "motor.h"

osMessageQueueId_t chassis_MQ_handel;
osMessageQueueId_t chassis_RMQ_handel;
osThreadId_t chassis_task_handel;
ChassisPID_InitTypedef chassis_pid;
void ChassisTask(void *arguement);

#if INFANTRY
/*
 * @brief  	全向轮解算
 * @param	底盘控制结构体指针
 * @retval 	float[4], 各轮角速度
 * @note:   仅考虑全向轮安装角为45°且轴长相等的情况
 */
void OnmiSolve(Chassis_CmdTypedef *cmd, float *ret,float omega_z)
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
    ret[0] = (omega_z * (2*PI*CHASSIS_RADIUS) - (-cmd->vx - cmd->vy) * SQRT_2) / WHEEL_RADIUS;
    ret[1] = (omega_z * (2*PI*CHASSIS_RADIUS) - (cmd->vx - cmd->vy) * SQRT_2) / WHEEL_RADIUS;
    ret[2] = (omega_z * (2*PI*CHASSIS_RADIUS) - (cmd->vx + cmd->vy) * SQRT_2) / WHEEL_RADIUS;
    ret[3] = (omega_z * (2*PI*CHASSIS_RADIUS) - (-cmd->vx + cmd->vy) * SQRT_2) / WHEEL_RADIUS;
}
/*
 * @brief  	全向轮逆解算(3508电机)
 * @param	底盘四电机结构体指针，顺序1-4
 * @retval 	底盘自旋速度，rps
 * @todo    按需增加对里程的解算
 */
float ChassisGetOmega(Motor_TypeDef *motor[4])
{
    float ret=0;
    for(int i=0;i<4;i++)
        ret += MotorGetVal(motor[i], ORIGIN);
    ret = ret /60 ;    //RPM to RPS
    ret = (ret*187.0f/3591.0f)/4;	//转换到输出端转速
    ret = ret*(WHEEL_RADIUS/CHASSIS_RADIUS);
    return ret;
}
#endif

#if HERO
//TODO(GUATAI)后续修改底盘解算方向与步兵一致
/*
 * @brief  	麦轮运动学逆解算(3508电机)
 * @param	底盘控制结构体指针
 * @retval 	float[4], 各轮角速度
 * @note:   仅考虑麦轮俯视打叉，仰视画圆且轴长不等的情况
 */
void Mecanum_Solve(Chassis_CmdTypedef *cmd, float *ret)
{
//		/*         vx
//		      ^
//		      |
//		 ||       ||
//		0//       \\1		  (-1  1 a+b)     (w0)
//		 || \   / ||		  ( 1  1 a+b)(vx) (w1)
//	vy<--    top           1/s(         )(vy)=(  )
//		 || /   \ ||		  ( 1 -1 a+b)(wz) (w2)
//		3\\       //2		  (-1 -1 a+b)     (w3)
//		 ||       ||
//		 */
	ret[0] = -(cmd->vx - cmd->vy + cmd->omega_z * 0.5 * (Length + Width)) * 60
			/ (2 * Wheel_radius * PI) * 3591 / 187;
	ret[1] = (cmd->vx + cmd->vy - cmd->omega_z * 0.5 * (Length + Width)) * 60
			/ (2 * Wheel_radius * PI) * 3591 / 187;
	ret[2] = (cmd->vx - cmd->vy - cmd->omega_z * 0.5 * (Length + Width)) * 60
			/ (2 * Wheel_radius * PI) * 3591 / 187;
	ret[3] = -(cmd->vx + cmd->vy + cmd->omega_z * 0.5 * (Length + Width)) * 60
			/ (2 * Wheel_radius * PI) * 3591 / 187;
}

/*
 * @brief  	麦轮运动学正解算
 * @param	底盘四电机结构体指针，顺序1-4
 * @retval 	底盘自旋速度，rps
 * @todo    按需增加对里程的解算
 */
float Chassis_Get_Mecanum(motor_t *motor[4])
{
//		/*         vx       vx=(-w0+w1+w2-w3)*s/4
//		      ^				vy=(w0+w1-w2-w3)*s/4
//		      |				wz=(w0+w1+w2+w3)/a+b*s/4
//		 ||       ||
//		0//       \\1		 						   (w0)
//		 || \   / ||		  ( -1      1    1    -1  )(w1) (vx)
//	vy<--             top  s/4(  1      1   -1    -1  )(  )=(vy)
//		 || /   \ ||		  (1/a+b 1/a+b 1/a+b 1/a+b)(w2) (wz)
//		3\\       //2								   (w3)
//		 ||       ||
//		 */
	float ret = 0;
	for (int i = 0; i < 4; i++)
		ret += Motor_Get_Val(motor[i], ORIGIN) / 60;	//RPM to RPS
	ret = (ret * 187.0 / 3591.0) / 4;	//转换到输出端转速
	ret = ret * 0.5 * (Length + Width);
	return ret;
//	PID_Origin(&PID_Motor_Angle[Chassis_Angle_ID], Motor[Motor_Yaw_ID].angle, 0);
//	//底盘角度环,以底盘与云台的相对角度为输入
//	omega = PID_Motor_Angle[Chassis_Angle_ID].Output;
}
#endif


void ChassisInit(void)
{
    osMessageQueueAttr_t const cmd_attr = {.name = "chassis_cmd"};
    chassis_MQ_handel = osMessageQueueNew(3, sizeof(Chassis_CmdTypedef), &cmd_attr);
    osMessageQueueAttr_t const msg_attr = {.name = "chassis_msg"};
    chassis_RMQ_handel = osMessageQueueNew(3, sizeof(float), &msg_attr);
    osThreadAttr_t task_attr = {.name = "ChassisTask"};
    chassis_task_handel = osThreadNew(ChassisTask, NULL, &task_attr);
//    MotorStatusChange(motor_list[MOTOR_CHS_1], SPEED, NULL);//?
}

void ChassisTask(void *arguement)
{
    Chassis_CmdTypedef static cmd;
    float static solve[4];
    float static ret;
    Motor_TypeDef *chs_list[4] = {motor_list[MOTOR_CHS_1], motor_list[MOTOR_CHS_2], motor_list[MOTOR_CHS_3], motor_list[MOTOR_CHS_4]};
    for(;;)
    {
        osMessageQueueGet(chassis_MQ_handel, &cmd, NULL, 0);

        ret = ChassisGetOmega(chs_list);
        if(cmd.mode == SPIN)
        {
            chassis_pid.output = PID_Incr(&chassis_pid.pid_inner, CHS_SPIN_LMT, ret);
        }
        else if(cmd.mode == FOLLOW)
        {
            chassis_pid.output = PID_Origin(&chassis_pid.pid_outer, MotorGetVal(motor_list[MOTOR_YAW],ORIGIN), CHASSIS_ZEROPOINT);
            chassis_pid.output = PID_Incr(&chassis_pid.pid_inner, ret, chassis_pid.output);
        }
        OnmiSolve(&cmd, solve,chassis_pid.output);
        for(int i=0;i<4;i++)
            MotorSetTar(chs_list[i], solve[i], ABS);

#if INFANTRY
		ret = ChassisGetOmega(chs_list);
#endif
#if HERO
		ret = Chassis_Get_Mecanum(chs_list);
#endif
        osMessageQueuePut(chassis_RMQ_handel, &ret, NULL, 0);
        osDelay(CHASSIS_TASK_PERIOD);
    }
}
