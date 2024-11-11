#include "Chassis.h"
#include "string.h"
#include "config.h"
#include "cmsis_os2.h"
#include "motor.h"

//osMessageQueueId_t chassis_MQ_handel;
//osMessageQueueId_t chassis_RMQ_handel;
osThreadId_t chassis_task_handel;
Chassis_TypeDef chassis;

Chassis_CmdTypedef chassis_cmd;



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
float ChassisGetOmega(void)
{
    float ret=0;
    for(int i=MOTOR_CHS_1;i<4;i++)
        ret += MotorGetVal(motor_list[i], MOTORSPEED,ORIGIN);
    ret = ret /60 ;    //RPM to RPS
    ret = (ret*187.0f/3591.0f)/4;	//转换到输出端转速
    ret = ret*(WHEEL_RADIUS/CHASSIS_RADIUS);
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


void ChassisInit(void)
{
//    osMessageQueueAttr_t const cmd_attr = {.name = "chassis_cmd"};
//    chassis_MQ_handel = osMessageQueueNew(3, sizeof(Chassis_CmdTypedef), &cmd_attr);
//    osMessageQueueAttr_t const msg_attr = {.name = "chassis_msg"};
//    chassis_RMQ_handel = osMessageQueueNew(3, sizeof(float), &msg_attr);
    osThreadAttr_t task_attr = {.name = "ChassisTask"};
    chassis_task_handel = osThreadNew(ChassisTask, NULL, &task_attr);
//    MotorStatusChange(motor_list[MOTOR_CHS_1], SPEED, NULL);//?
}

void ChassisTask(void *arguement)
{
    uint32_t time = osKernelGetTickCount();
    float static solve[4];
    for(;;)
    {
        chassis_cmd.omega_z_last = ChassisGetOmega();
        if(chassis_cmd.mode == SPIN)
        {
            chassis.output = PID_Increment(&chassis.pid[INNER], chassis_cmd.omega_z_last,CHS_SPIN_LMT);
        }
        else if(chassis_cmd.mode == FOLLOW)
        {
            chassis.output = PID_Position(&chassis.pid[OUTER], MotorGetVal(motor_list[MOTOR_YAW],MOTORANGLE,ORIGIN), CHASSIS_ZEROPOINT);
            chassis.output = PID_Increment(&chassis.pid[INNER], chassis_cmd.omega_z_last, chassis.output);
        }
#if INFANTRY
        OnmiSolve(&chassis_cmd, solve,0);
#endif
#if HERO
		MecanumSolve(&chassis_cmd, solve,0);
#endif
        for(int i=0;i<4;i++)
            MotorSetTar(motor_list[i+MOTOR_CHS_1], solve[i], ABS);

//#if INFANTRY
//		ret = ChassisGetOmega(chs_list);
//#endif
//#if HERO
//		ret = ChassisGetOmega(chs_list);
//#endif
//osMessageQueuePut(chassis_RMQ_handel, &ret, NULL, 0);


        time = osKernelGetTickCount();
//    	osDelay(CANBUS_TASK_PERIOD);
        osDelayUntil(time + CHASSIS_TASK_PERIOD);
    }
}
