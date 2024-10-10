#include "pid.h"

#if TEST_I
//TODO(GUATAI)需要进一步联动看门狗
float DWT_GetDeltaT(uint32_t *cnt_last)
{
	float dt= 0.0f;
	return dt;
}
// 输出滤波
static void f_Output_Filter(PID_TypeDef *hpid)
{
	hpid->dt=DWT_GetDeltaT(&hpid->DWT_cnt);
	hpid->Output = hpid->Output * hpid->dt / (hpid->Output_LPF_RC + hpid->dt) +
				  hpid->Output_last * hpid->Output_LPF_RC / (hpid->Output_LPF_RC + hpid->dt);
}
#else
#endif
/*
 * @brief  	位置式pid控制器
 * @param
 * 		hpid			pid结构体
 * 		val_now			获取到的当前值
 * 		target_now		目标值
 * @retval 	控制量
 *
 */
float PID_Origin(PID_TypeDef *hpid, float val_now, float target_now)
{
	hpid->Err_now = target_now - val_now;
	float Output;
#if TEST_D
    if(hpid->Err_now<hpid->Dead_Band)
    {
#else
#endif
        hpid->Err_sum += hpid->Err_now;
        if (hpid->Err_sum < -hpid->Err_sum_Max) hpid->Err_sum = -hpid->Err_sum_Max;
        if (hpid->Err_sum > hpid->Err_sum_Max) hpid->Err_sum = hpid->Err_sum_Max;

        Output = hpid->Kp * hpid->Err_now + hpid->Ki * hpid->Err_sum
                 + (hpid->Kd) * (hpid->Err_now - hpid->Err_last);

        Output += hpid->Kf * (Output - hpid->Output_last);//前馈

        if (Output > hpid->Output_Max)
            Output = hpid->Output_Max;
        if (Output < -hpid->Output_Max)
            Output = -hpid->Output_Max;
#if TEST_D
    }
    else
	{
		hpid->Err_sum=0;
        Output=0;
	}
#else
#endif
        hpid->Err_former = hpid->Err_last;
        hpid->Err_last = hpid->Err_now;
        hpid->Output_last = Output;

    return Output;
}
/*
 * @brief  	增量式pid控制器
 * @param
 * 		hpid			pid结构体
 * 		val_now			当前值
 * 		target_now		目标值
 * @retval 	控制量的增量
 *
 */
float PID_Incr(PID_TypeDef *hpid, float val_now, float target_val_now)
{
	hpid->Err_now = target_val_now - val_now;
	float Output;
#if TEST_D
    if(hpid->Err_now<hpid->Dead_Band)
    {
#else
#endif
        Output = hpid->Kp * (hpid->Err_now - hpid->Err_last) + hpid->Ki * (hpid->Err_now) +
                 hpid->Kd * (hpid->Err_now - hpid->Err_last - hpid->Err_diff);
        if (Output > hpid->Output_Max) Output = hpid->Output_Max;
        if (Output < -hpid->Output_Max) Output = -hpid->Output_Max;
        hpid->Err_diff = hpid->Err_now - hpid->Err_last;
        hpid->Err_last = hpid->Err_now;
#if TEST_D
    }
    else
	{
		hpid->Err_sum=0;
        Output=0;
	}
#else
#endif

    return Output;
}


/*
 * @brief  	清除PID积分值和输出
 * @param	pid结构体
 * @retval 	无
 *
 */
void PID_Clear(PID_TypeDef *hpid)
{
//	hpid->Output_Max = 0;
//	hpid->Err_sum_Max = 0;
    hpid->Err_diff = 0;
    hpid->Err_former = 0;
    hpid->Err_last = 0;
    hpid->Err_now = 0;
    hpid->Err_sum = 0;
#if TEST_D
    hpid->Dead_Band=0.0f;
#else
#endif
}
