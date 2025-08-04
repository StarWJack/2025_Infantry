#ifndef __PID_H__
#define __PID_H__
#include "main.h"

//取绝对值
#ifndef ABS
#define ABS(x) ((x) > 0 ? (x) : -(x) )
#endif

//定义前馈各阶参数
//v1.2
//#define FEED_PARAM0 794.4f
//#define FEED_PARAM1 2466.0f
//#define FEED_PARAM2 -1124
//#define FEED_PARAM3 273
//#define FEED_PARAM4 -25.97f
//#define FEED_PARAM5 0.7919f
//v1.3,最优版(无带载)
#define FEED_PARAM0 298.2f
#define FEED_PARAM1 1214
#define FEED_PARAM2 -26.32

typedef struct __PID_TypeDef
{
	float Kp;
	float Ki;
	float Kd;
	float Kf;//前馈系数

	float i_term;
	float p_out;
	float i_out;
	float d_out;

	float target;//目标值
	float last_target;
	float measure;//观测值
	float last_measure;
	float dead_band;

	float Err_pre;
	float Err_last;
	float Err_now;
	float Err_sum;
	float Output_Max; //PID最大输出值
	float Err_sum_Max;

	float Output;
	float Output_feed;
	float Output_last;
	float integral_limit;     // 积分限幅
	float count;     //PID死区标志
	float f_err;//前馈误差输出值
} PID_TypeDef;

typedef enum
{
    ANGLE = 0,          //速度环，使用增量式
    SPEED = 1,         //角度环，使用位置式
    DUAL_LOOP,   //双环控制，外环角度,内环速度
	AMMO
} PID_Type_e;

extern float PID_Position( PID_TypeDef *hpid, float val_now, float target_now );

extern float PID_Increment( PID_TypeDef *hpid, float val_now, float target_now );

extern void PID_Clear( PID_TypeDef *hpid );

extern void FeedForward_Input(PID_TypeDef *hpid,float tar);

#endif
