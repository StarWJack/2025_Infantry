#ifndef __GIMBAL_H__
#define __GIMBAL_H__
#include "main.h"
#include "cmsis_os2.h"


typedef enum
{
    STOP = 0,
    AUTO = 1,//火控
    FULL = 2,//泼水
    TEST = 3//测试 小拨盘上下发弹
}shooter_strategy_e;//开火策略


typedef struct
{
    shooter_strategy_e shooter;
    uint8_t friction_state;//摩擦轮状态:0-关闭，1-开启
    float pitch;
    float yaw;
    float v_yaw;
    float v_pitch;
    float omega_z;   //spin clockwise from above when positive正值时从上方顺时针旋转
}__attribute__((__packed__))Gimbal_CmdTypedef;


extern osMessageQueueId_t gimbal_MQ_handel;
extern void GimbalTask(void *argument);
void GimbalInit(void);

#endif
