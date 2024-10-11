#include "Gimbal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "config.h"
#include "pid.h"
#include "ErrorHandel.h"
#include "math.h"
#include "remote.h"

osMessageQueueId_t gimbal_MQ_handel;
osThreadId_t gimbal_task_handel;
// const osMessageQueueAttr_t gimbal_cmd_attr = {
//     .name = "gimbal_cmd_queue",
//     .mq_size = 3*sizeof(gimbal_cmd),
//     .mq_mem = sizeof(gimbal_cmd)
// };
//static float pitch_tar=0;
//TODO: updata feedforward for motor_yaw
void GimbalCtrl(Gimbal_CmdTypedef *st)
{

	//弧度转6020编码值
    MotorSetTar(motor_list[MOTOR_YAW], st->v_yaw, INCR);
//    MotorSetTar(motor_list[MOTOR_YAW], st->omega_z*60*CHASSIS_RADIUS*187/(3591*WHEEL_RADIUS), FEED);
//    float pitch_tar =motor_list[MOTOR_PITCH]->target +st->v_pitch ;
//    if(pitch_tar > PITCH_LW_BOUND && pitch_tar < PITCH_UP_BOUND)
//    {
//        float diff1 = fabs(pitch_tar - PITCH_LW_BOUND);
//        float diff2 = fabs(pitch_tar - PITCH_UP_BOUND);
//        pitch_tar = diff1<diff2?PITCH_LW_BOUND:PITCH_UP_BOUND;
//    }
    MotorSetTar(motor_list[MOTOR_PITCH], 0, ABS);

}


void ShooterCtrl(Gimbal_CmdTypedef *st)
{
    static uint8_t shooter_pause = 0;
//    float speed = CALIBRATE*10/(2*PI*FRICTION_RADIUS);
//    speed = speed*3591/187;		//摩擦轮rps
    if(st->friction_state)
    {
        MotorSetTar(motor_list[MOTOR_SHT_L], 20, ABS);
        MotorSetTar(motor_list[MOTOR_SHT_R], -20, ABS);
    }
    else
    {
        MotorSetTar(motor_list[MOTOR_SHT_L], 0, ABS);
        MotorSetTar(motor_list[MOTOR_SHT_R], 0, ABS);
    }

    if(st->shooter == STOP)
////        MotorSetTar(motor_list[MOTOR_AMMO], 36*(2*PI/8), INCR);
        MotorSetTar(motor_list[MOTOR_AMMO], 0, INCR);
//    else if(st->shooter == FULL)
//        MotorSetTar(motor_list[MOTOR_AMMO], (36*SHOOT_FREQ*(float)(GIMBAL_TASK_PERIOD)/1000)*(2*PI/8), INCR);
//        MotorSetTar(motor_list[MOTOR_AMMO], 36*(2*PI/8), INCR);
    else if(st->shooter == TEST)
    {
        if(RC_Ctl.rc.wheel>200)
        {
            if(shooter_pause++>50)
            {
#ifdef AMMO_DIR_INV
                MotorSetTar(motor_list[MOTOR_AMMO], -36*(2*PI/8), INCR);
#elif
                MotorSetTar(motor_list[MOTOR_AMMO], (36*SHOOT_FREQ*(float)(GIMBAL_TASK_PERIOD)/1000)*(2*PI/8), INCR);
#endif
                shooter_pause = 0;
            }

        }
        else if(RC_Ctl.rc.wheel<-200)
        {
#ifdef AMMO_DIR_INV
                MotorSetTar(motor_list[MOTOR_AMMO], -(36*SHOOT_FREQ*(float)(GIMBAL_TASK_PERIOD)/1000)*(2*PI/8), INCR);
#elif
                MotorSetTar(motor_list[MOTOR_AMMO], (36*SHOOT_FREQ*(float)(GIMBAL_TASK_PERIOD)/1000)*(2*PI/8), INCR);
#endif
        }
        else
        {
            MotorSetTar(motor_list[MOTOR_AMMO], 0, INCR);
        }
    }
//        MotorSetTar(motor_list[MOTOR_AMMO], 36*(2*PI/8), INCR);
//    else if(st->shooter == AUTO)
//    {}//TODO:火控
}


void GimbalInit(void)
{   
    //gimbal command message queue
	osMessageQueueAttr_t const attr = {.name = "gimbal_cmd"};
    gimbal_MQ_handel = osMessageQueueNew(3, sizeof(Gimbal_CmdTypedef), &attr);
    osThreadAttr_t attr_task = {.name = "GimbalTask"};
    gimbal_task_handel = osThreadNew(GimbalTask, NULL, &attr_task);
    
//    MotorSetZeroPoint(motor_list[MOTOR_YAW]);
//    MotorSetTar(motor_list[MOTOR_YAW], (float)motor_list[MOTOR_YAW]->info.zero_point, ABS);
//    MotorSetZeroPoint(motor_list[MOTOR_PITCH]);
//    MotorSetTar(motor_list[MOTOR_PITCH], 0.5*(PITCH_LW_BOUND+PITCH_UP_BOUND)*2*PI/8192, ABS);
    MotorSetTar(motor_list[MOTOR_YAW], MotorGetVal(motor_list[MOTOR_YAW],RAD), ABS);
    MotorSetTar(motor_list[MOTOR_PITCH], PITCH_MID, ABS);
    MotorSetTar(motor_list[MOTOR_AMMO], MotorGetVal(motor_list[AMMO],RAD),ABS);
}

void GimbalTask(void *argument)
{

    Gimbal_CmdTypedef gimbal_cmd;
//    uint32_t time = osKernelSysTick();
    for(;;)
    {
        osMessageQueueGet(gimbal_MQ_handel, &gimbal_cmd, NULL, 0);   //non blocking
        GimbalCtrl(&gimbal_cmd);
        ShooterCtrl(&gimbal_cmd);
        
//        osDelayUntil(time+GIMBAL_TASK_PERIOD);
//        time = osKernelSysTick();
        osDelay(GIMBAL_TASK_PERIOD);
    }
}
















