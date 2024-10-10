#ifndef __PID_H__
#define __PID_H__

#include "main.h"

//TODO(GUATAI)死区处理操作待测试
#define TEST_D 0
//TODO(GUATAI)前馈待测试
#define TEST_F 0
//TODO(GUATAI)其它优化PID函数待测试
#define TEST_I 0
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

typedef struct __PID_TypeDef {
	float Kp;
	float Ki;
	float Kd;
	float Kf;//前馈系数
#if TEST_I
	float dt;
	uint32_t DWT_cnt;
#else
	float T;//采样时间（任务运行周期）
#endif
	float Err_former;
	float Err_last;
	float Err_now;
	float Err_sum;
	float Err_diff;
	float Output_Max;
	float Err_sum_Max;
#if TEST_I
	float Output;
	float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
#else
#endif
	float Output_last;
#if TEST_D
	float Dead_Band;
#else
#endif
} PID_TypeDef;

typedef enum {
	ANGLE = 0,          //速度环，使用增量式
	SPEED = 1,         //角度环，使用原始位置式
	DUAL_LOOP,   //双环控制，外环角度内环速度
	IMU,
	AMMO
} PID_Type_e;

extern float PID_Origin(PID_TypeDef *hpid, float val_now, float target_now);

extern float PID_Incr(PID_TypeDef *hpid, float val_now, float target_val_now);

extern void PID_Clear(PID_TypeDef *hpid);

#endif
