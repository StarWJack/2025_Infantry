#include "Gimbal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "config.h"
#include "pid.h"
#include "ErrorHandel.h"
#include "math.h"
#include "remote.h"

//osMessageQueueId_t gimbal_MQ_handel;
osThreadId_t gimbal_task_handel;
Gimbal_CmdTypedef gimbal_cmd;

// const osMessageQueueAttr_t gimbal_cmd_attr = {
//     .name = "gimbal_cmd_queue",
//     .mq_size = 3*sizeof(gimbal_cmd),
//     .mq_mem = sizeof(gimbal_cmd)
// };
//static float pitch_tar=0;
//TODO: updata feedforward for motor_yaw
void GimbalCtrl(Gimbal_CmdTypedef *st)
{
     float tmp = motor_list[MOTOR_YAW]->target + st->v_yaw;
     while (tmp >  PI)
         tmp -= 2 * PI;
     while (tmp < -PI)
         tmp += 2 * PI;
     MotorSetTar(motor_list[MOTOR_YAW], tmp, ABS);
    // MotorSetTar(motor_list[MOTOR_YAW], st->omega_z * 60 * CHASSIS_RADIUS * 187 / (3591 * WHEEL_RADIUS), FEED);
    tmp = motor_list[MOTOR_PITCH]->target - st->v_pitch;
    if(tmp < PITCH_LW_BOUND)
        tmp = PITCH_LW_BOUND;
    else if(tmp > PITCH_UP_BOUND)
        tmp = PITCH_UP_BOUND;
    MotorSetTar(motor_list[MOTOR_PITCH], tmp, ABS);
}


void ShooterCtrl(Gimbal_CmdTypedef *st)
{
    static uint8_t shooter_pause = 0;
    if(st->friction_state)
    {
        MotorSetTar(motor_list[MOTOR_SHT_L], BULLET_V, ABS);
        MotorSetTar(motor_list[MOTOR_SHT_R], -BULLET_V, ABS);
    }
    else
    {
    MotorSetTar(motor_list[MOTOR_SHT_L], 0, ABS);
    MotorSetTar(motor_list[MOTOR_SHT_R], 0, ABS);
    }

    if(st -> shooter == STOP)
////        MotorSetTar(motor_list[MOTOR_AMMO], 36*(2*PI/8), INCR);
        MotorSetTar(motor_list[MOTOR_AMMO], 0, INCR);
//    else if(st->shooter == FULL)
//        MotorSetTar(motor_list[MOTOR_AMMO], (36*SHOOT_FREQ*(float)(GIMBAL_TASK_PERIOD)/1000)*(2*PI/8), INCR);
//        MotorSetTar(motor_list[MOTOR_AMMO], 36*(2*PI/8), INCR);
    else if(st -> shooter == TEST)
    {
        if(RC_Ctl . rc . wheel > 200)
        {
            if(shooter_pause ++ > 50)
            {
#ifdef AMMO_DIR_INV
                MotorSetTar(motor_list[MOTOR_AMMO], - 36 * (2 * PI / 8), INCR);
#elif
                MotorSetTar(motor_list[MOTOR_AMMO], (36*SHOOT_FREQ*(float)(GIMBAL_TASK_PERIOD)/1000)*(2*PI/8), INCR);
#endif
                shooter_pause = 0;
            }

        }
        else if(RC_Ctl . rc . wheel < - 200)
        {
#ifdef AMMO_DIR_INV
            MotorSetTar(motor_list[MOTOR_AMMO],
                          - (36 * SHOOT_FREQ * (float) (GIMBAL_TASK_PERIOD) / 1000) * (2 * PI / 8), INCR);
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

void GimbalPosi_Init(void)
{
    MotorSetTar(motor_list[MOTOR_YAW], MotorGetVal(motor_list[MOTOR_YAW], IMU, RAD), ABS);
    MotorSetTar(motor_list[MOTOR_PITCH], MotorGetVal(motor_list[MOTOR_PITCH], IMU, RAD), ABS);
    MotorSetTar(motor_list[MOTOR_AMMO], MotorGetVal(motor_list[MOTOR_AMMO], MOTORANGLE, RAD), ABS);
}


void GimbalInit(void)
{
    osThreadAttr_t attr_task = {.name = "GimbalTask"};
    gimbal_task_handel = osThreadNew(GimbalTask, NULL, &attr_task);
    GimbalPosi_Init();
}

void GimbalTask(void *argument)
{
    uint32_t time = osKernelGetTickCount();
    for(;;)
    {
        GimbalCtrl(&gimbal_cmd);
        ShooterCtrl(&gimbal_cmd);

        time = osKernelGetTickCount();
        osDelayUntil(time + GIMBAL_TASK_PERIOD);

    }
}
















