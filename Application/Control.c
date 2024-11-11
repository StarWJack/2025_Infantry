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
#include "imu.h"
#include "vofa.h"
#include "CanBus.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"
#include "Serial.h"


osThreadId_t control_task_handel;
void ControlInit(void)
{
	osThreadAttr_t attr = {.name = "ControlTask"};
    control_task_handel = osThreadNew(ControlTask, NULL, &attr);

//    WatchDogInit();
}

/*
 * @brief  	底盘坐标系转云台坐标系
 * @param	底盘命令结构体指针
 * @param   底盘坐标系中，云台yaw与底盘x平行时yaw轴电机角度（编码值）
 * @param   yaw电机当前角度（编码值）
 * @retval 	无
 */
void ChassisCmdTrans(Chassis_CmdTypedef *cmd, float chs_zeropoint, float gim_ang)
{
    float bias_tmp = (gim_ang - chs_zeropoint +8192.0f);
    if(bias_tmp>8192)
        bias_tmp -= 8192;
    else if(bias_tmp<0)
        bias_tmp += 8192;
    bias_tmp = bias_tmp/8192*2*PI;

//    bias = ((gim_ang/8192.0f)*2*PI) - ((chs_zeropoint/8192.0f)*2*PI);
    float vx_tmp = cmd->vx*cosf(bias_tmp) + cmd->vy*sinf(bias_tmp);
    float vy_tmp = -cmd->vx*sinf(bias_tmp) + cmd->vy*cosf(bias_tmp);
    cmd->vx = vx_tmp;
    cmd->vy = vy_tmp;

}

void KeyboardCtrl(Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)
{
    gim->v_yaw -= (RC_Ctl.keyboard.x >> 4)*KEYBOARD_YAW_SEN;
    gim->v_pitch += (RC_Ctl.keyboard.y >> 4)*KEYBOARD_PITCH_SEN;
    gim->shooter = RC_Ctl.keyboard.l>=1 ? AUTO : STOP;

    chs->vx = (RC_Ctl.keyboard.W * (-CHS_SPEED_LMT)) + (RC_Ctl.keyboard.S * (CHS_SPEED_LMT));
    chs->vy = (RC_Ctl.keyboard.A * (-CHS_SPEED_LMT)) + (RC_Ctl.keyboard.D * (CHS_SPEED_LMT));
//    chs->omega_z = RC_Ctl.keyboard.SHIFT * CHS_SPIN_LMT;
    if(RC_Ctl.keyboard.SHIFT)
        chs->mode = SPIN;
    else
        chs->mode = FOLLOW;
}

void RemoteCtrl(Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)
{
    chs->vx = RC_Ctl.rc.ch2 * REMOTE_X_SEN;
    chs->vy = RC_Ctl.rc.ch1 * REMOTE_Y_SEN;
    gim->v_yaw = -RC_Ctl.rc.ch3 * REMOTE_YAW_SEN*CONTROL_TASK_PERIOD;
    gim->v_pitch = -RC_Ctl.rc.ch4 * REMOTE_PITCH_SEN*CONTROL_TASK_PERIOD;


    // chassis_cmd.omega_z = RC_Ctl.rc.sw2==1 ? 2*PI : 0;
//    gim->shooter = RC_Ctl.rc.sw2==1 ? AUTO : 0;
    chs->mode = RC_Ctl.rc.sw1;
    if(RC_Ctl.rc.sw2 == 2 || RC_Ctl.rc.sw2 == 0)
    {
        gim->shooter = STOP;
        gim->friction_state = 0;
    }
    else
    {
//        gim->shooter = STOP;
        gim->friction_state = 1;
        if(RC_Ctl.rc.sw2==3)
            gim->shooter = TEST;//测试，小拨盘上下拨动发弹
        else if(RC_Ctl.rc.sw2==1)
            gim->shooter = FULL;//泼水
    }


}
//TODO:添加底盘跟随和小陀螺时云台自稳
void ControlTask(void *argument)
{
    uint32_t time = osKernelGetTickCount();
    for(;;)
    {
        UnPack_Data_ROS2(buf_receive_from_nuc,&aim_packet_from_nuc,sizeof (aim_packet_from_nuc));
        RemoteCtrl(&gimbal_cmd, &chassis_cmd);
#ifdef USE_KEYBOARD
        KeyboardCtrl(&gimbal_cmd, &chassis_cmd);
#endif
        //云台坐标系转底盘坐标系
        ChassisCmdTrans(&chassis_cmd, CHASSIS_ZEROPOINT, MotorGetVal(motor_list[MOTOR_YAW],MOTORANGLE,ORIGIN));

//        vofa_demo(motor_list[MOTOR_CHS_1]->pid[SPEED].Err_now,motor_list[MOTOR_CHS_1]->pid[ANGLE].Err_sum,
//                  motor_list[MOTOR_CHS_1]->output,motor_list[MOTOR_CHS_1]->target,
//                  MotorGetVal(motor_list[MOTOR_PITCH],IMU,RAD),robo_state
//                  );
        vofa_demo(motor_list[MOTOR_CHS_1]->pid[SPEED].Err_now,motor_list[MOTOR_CHS_1]->pid[SPEED].Err_last,
                  motor_list[MOTOR_CHS_1]->pid[SPEED].Err_pre,motor_list[MOTOR_CHS_1]->target,
                  MotorGetVal(motor_list[MOTOR_CHS_1],MOTORSPEED,RAD),robo_state);

        Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);

        time = osKernelGetTickCount();
        osDelayUntil(time + CONTROL_TASK_PERIOD);
    }
}
