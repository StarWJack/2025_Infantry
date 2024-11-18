#ifndef __PID_H__
#define __PID_H__

#include "main.h"

//取绝对值
#ifndef abs
#define abs( x ) ((x > 0) ? x : -x)
#endif

//0为关闭
//TODO(GUATAI)其它优化PID函数以及前馈待测试
#define TEST_FEED 0
//TODO(GUATAI)输入处理待测试
#define TEST_INPUT 0
/*
 * 先暂时T:采样周期0.01s(10ms)
 * (根据输出大小调整合适单位)
 * 假设角速度/力矩：G(s)=1/(s+1)
 * 则前馈环节：Gf(s)=s+1
 *
 * ①输出：f_out=target'+(input-last_input)/T+input
 * input=(fliter(error))(t)即f_out=(input-last_input)/T+input
 * 从而f_out+pid_out=out,对时间积分(out)=output,输出output
 * 综上似乎拖慢了计算过程，待简化
 * &&
 * ②输出：f_out=kf*Err_now
 */

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum
{
	PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
	PID_INTEGRAL_LIMIT = 0b00000001,              // 0000 0001
	PID_DERIVATIVE_ON_MEASUREMENT = 0b00000010,   // 0000 0010
	PID_TRAPEZOID_INTEGRAL = 0b00000100,         // 0000 0100
	PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
	PID_OUTPUT_FILTER = 0b00010000,                // 0001 0000
	PID_CHANGING_INTEGRATION_RATE = 0b00100000,     // 0010 0000
	PID_DERIVATIVE_FILTER = 0b01000000,            // 0100 0000
	PID_ErrorHandle = 0b10000000,                 // 1000 0000 检测错误
} PID_improvement_e;

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

	float target;
	float last_target;
	float measure;
	float last_measure;
	float dead_band;
#if TEST_FEED
	PID_improvement_e improve;//PID开启优化选项
	float ci_coefA;             // 变速积分 For Changing Integral
	float ci_coefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
	float dt;
	uint32_t DWT_cnt;
	float d_out_last;
	float output_LPF_RC;     // 输出滤波器 RC = 1/omegac
	float derivative_LPF_RC; // 微分滤波器系数
#else
#endif
#if TEST_INPUT
	float *ref_p; //设定值
	float *fab_p; //反馈值
	float target_change_max;
	float input_LPF_RC;//输入滤波系数
#else
#endif
	float Err_pre;
	float Err_last;
	float Err_now;
	float Err_sum;
	float Output_Max;
	float Err_sum_Max;

	float Output;
	float Output_last;
	float integral_limit;     // 积分限幅
} PID_TypeDef;

typedef enum
{
    ANGLE = 0,          //速度环，使用增量式
    SPEED = 1,         //角度环，使用位置式
    DUAL_LOOP,   //双环控制，外环角度,内环速度
	AMMO
} PID_Type_e;

extern float fit_data[5] ;

extern float PID_Position( PID_TypeDef *hpid, float val_now, float target_now );

extern float PID_Increment( PID_TypeDef *hpid, float val_now, float target_now );

extern void PID_Clear( PID_TypeDef *hpid );

extern float FeedForward_Input(float tar,float *data);

#endif
