#include "pid.h"

#if TEST_I
/* ----------------------------下面是pid优化环节的实现---------------------------- */
//TODO(GUATAI)dt获取需要进一步联动DWT内核定时器
// 梯形积分
static void F_Trapezoid_Intergral( PID_TypeDef *hpid )
{
	// 计算梯形的面积,(上底+下底)*高/2
	hpid->i_term = hpid->Ki * ( ( hpid->Err_now + hpid->Err_last ) / 2 ) * hpid->dt;
}

// 变速积分(误差小时积分作用更强)
//完整变速积分 基于积分分离优化的思想，根据系统偏差大小改变积分的速度，偏差越大，积分越慢，反之则越快，
//形成连续的变化过程。为此，设置变速积分系数alpha
//当error增大时，f减小，反之增大，其值在0~1变化。
//static float err_absmax,err_absmin,alpha = 0.0;
//
//	if (abs(error) <= err_absmin)
//	{
//		alpha = 1.0;
//	}
//	else if (abs(error) > err_absmax)
//	{
//		alpha = 0.0;
//	}
//	else
//	{
//		alpha = (err_absmax - abs(error)) / (err_absmax - err_absmin);
//	}
//	pid->i_term=pid->i_term*alpha
//err_absmax=A+B,err_absmin=B 人为设定两个值即可
static void F_Changing_Integration_Rate( PID_TypeDef *hpid )
{
	if ( hpid->Err_now * hpid->i_out > 0 )
	{
		// 积分呈累积趋势(误差小时直接返回即Full integral)
		if ( abs( hpid->Err_now ) <= hpid->ci_coefB )
		{
			return;
		}
		// 本次误差处于一定范围内，积分变速即*（0~1）的值
		if ( abs( hpid->Err_now ) <= ( hpid->ci_coefA + hpid->ci_coefB ) )
		{
			hpid->i_term *= ( hpid->ci_coefA - abs( hpid->Err_now ) + hpid->ci_coefB ) / hpid->ci_coefA;
		}
			//误差大时直接清零本次积分
		else
		{ // 最大阈值,不使用积分
			hpid->i_term = 0;
		}
	}
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
//完整微分先行static float d1, d2, d3, temp,gama微分先行系数;
//				temp = pid->gama * pid->kd + pid->kp;
//				d3 = pid->kd / temp;
//				d2 = (pid->kd + pid->kp) / temp;
//				d1 = pid->gama * d3;
//				pid->delta_current_value = pid->current_value
//						- pid->last_current_value;
//				pid->d_out = d1 * pid->d_out_last
//						+ d2 * pid->delta_current_value
//						+ d3 * pid->last_delta_current_value;
static void F_Derivative_On_Measurement( PID_TypeDef *hpid )
{
	hpid->d_out = hpid->Kd * ( hpid->last_measure - hpid->measure ) / hpid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void F_Derivative_Filter( PID_TypeDef *hpid )
{
	hpid->d_out = hpid->d_out * hpid->dt / ( hpid->derivative_LPF_RC + hpid->dt ) +
				  hpid->d_out_last * hpid->derivative_LPF_RC / ( hpid->derivative_LPF_RC + hpid->dt );
}

float DWT_GetDeltaT( uint32_t *cnt_last )
{
	float dt = 0.0f;
	return dt;
}

// 输出滤波
static void F_Output_Filter( PID_TypeDef *hpid )
{
	hpid->dt = DWT_GetDeltaT( &hpid->DWT_cnt );
	hpid->Output = hpid->Output * hpid->dt / ( hpid->output_LPF_RC + hpid->dt ) +
				   hpid->Output_last * hpid->output_LPF_RC / ( hpid->output_LPF_RC + hpid->dt );
}

#if TEST_R

//TODO(GUATAI)两种输入滤波初步预想，有待优化
static void F_Smooth_Target( PID_TypeDef *hpid )
{
	static float step = 0.0; //0~100的斜率
	static float k = 0.0;
	step = ( hpid->Output_Max - ( -hpid->Output_Max ) ) / 10;
	if ( abs( hpid->target - *hpid->ref_p ) <= step )
	{
		hpid->target = *hpid->ref_p;
	}
	else
	{
		if ( hpid->target - *hpid->ref_p > 0 )
		{
			k = -1.0;
		}
		else if ( hpid->target - *hpid->ref_p < 0 )
		{
			k = 1.0;
		}
		else
		{
			k = 0.0;
		}
		hpid->target = hpid->target + k * step;
	}
}

static void F_Ramping_Target( PID_TypeDef *hpid )
{
	if ( ( *hpid->ref_p - hpid->target ) <= hpid->target_change_max )
	{
		hpid->target = *hpid->ref_p;
	}
	else
	{
		hpid->target = hpid->input_LPF_RC * *hpid->ref_p
					   + ( 1 - hpid->input_LPF_RC ) * hpid->target;
	}
}

#else
#endif
#else
#endif

// 输出限幅
static void F_Output_Limit( PID_TypeDef *hpid )
{
	if ( hpid->Output > hpid->Output_Max )
	{
		hpid->Output = hpid->Output_Max;
	}
	if ( hpid->Output < -( hpid->Output_Max ) )
	{
		hpid->Output = -( hpid->Output_Max );
	}
}

//积分限幅
static void F_Integral_Limit( PID_TypeDef *hpid )
{
	static float temp_output, temp_iout;
	temp_iout = hpid->i_out + hpid->i_term;
	temp_output = hpid->p_out + hpid->i_out + hpid->d_out;
	//输出已经大于输出上下限限幅，清零本次积分
	if ( abs( temp_output ) > hpid->Output_Max )
	{
		if ( hpid->Err_now * hpid->i_out > 0 ) // 积分却还在累积
		{
			hpid->i_term = 0; // 当前积分项置零
		}
	}
	//积分输出超过上限，等于最大积分输出
	if ( temp_iout > hpid->integral_limit )
	{
		hpid->i_term = 0;
		hpid->i_out = hpid->integral_limit;
	}
	//积分输出超过下限，等于最小积分输出
	if ( temp_iout < -hpid->integral_limit )
	{
		hpid->i_term = 0;
		hpid->i_out = -hpid->integral_limit;
	}
}

/*
 * @brief  	位置式pid控制器
 * @param
 * 		hpid			pid结构体
 * 		val_now			获取到的当前值
 * 		target_now		目标值
 * @retval 	控制量
 *
 */
float PID_Origin( PID_TypeDef *hpid, float val_now, float target_now )
{
	hpid->Err_now = target_now - val_now;
#if TEST_I
	hpid->dt = DWT_GetDeltaT( &hpid->DWT_cnt );

	hpid->measure = hpid->last_measure;
	hpid->last_target = hpid->last_target;
	hpid->measure = val_now;
	hpid->target = target_now;
#if TEST_D
	if ( hpid->Err_now < hpid->Dead_Band )
	{
#else
#endif
#else
#endif
#if TEST_I
		hpid->p_out = hpid->Kp * hpid->Err_now;
		hpid->i_term = hpid->Ki * hpid->Err_now * hpid->dt;
		hpid->d_out = hpid->Kd * ( hpid->Err_now - hpid->Err_last ) / hpid->dt;

		//TODO(GUATAI)标志位识别待测试
		// 梯形积分
		if ( hpid->improve & PID_TRAPEZOID_INTEGRAL )
		{
			F_Trapezoid_Intergral( hpid );
		}
		// 变速积分
		if ( hpid->improve & PID_CHANGING_INTEGRATION_RATE )
		{
			F_Changing_Integration_Rate( hpid );
		}
		// 微分先行
		if ( hpid->improve & PID_DERIVATIVE_ON_MEASUREMENT )
		{
			F_Derivative_On_Measurement( hpid );
		}
		// 微分滤波器
		if ( hpid->improve & PID_DERIVATIVE_FILTER )
		{
			F_Derivative_Filter( hpid );
		}

		// 积分限幅
		if ( hpid->improve & PID_INTEGRAL_LIMIT )
		{
			F_Integral_Limit( hpid );
		}

		hpid->i_out += hpid->i_term;
		hpid->Output = hpid->p_out + hpid->i_out + hpid->d_out;

		// 输出滤波
		if ( hpid->improve & PID_OUTPUT_FILTER )
		{
			F_Output_Filter( hpid );
		}

		F_Output_Limit( hpid );
#if TEST_D
	}
	else
	{
		hpid->Output = 0;
		hpid->i_term = 0;
	}
#else
#endif
#else

	hpid->Err_sum += hpid->Err_now; 
	// hpid->Err_sum += hpid->Err_now;
	if ( hpid->Err_sum < -hpid->Err_sum_Max )
	{ hpid->Err_sum = -hpid->Err_sum_Max; }
	if ( hpid->Err_sum > hpid->Err_sum_Max )
	{ hpid->Err_sum = hpid->Err_sum_Max; }

	hpid->p_out = hpid->Kp * hpid->Err_now;
	hpid->i_out = hpid->Ki * hpid->Err_sum;
	hpid->d_out = hpid->Kd * ( hpid->Err_now - hpid->Err_last );

	hpid->Output = hpid->p_out + hpid->i_out + hpid->d_out;

	hpid->Output += hpid->Kf * ( hpid->Output - hpid->Output_last );//前馈

	F_Output_Limit( hpid );

	hpid->Err_former = hpid->Err_last;
	hpid->Err_last = hpid->Err_now;
	hpid->Output_last = hpid->Output;
#endif

	return hpid->Output;
}

/*
 * @brief  	增量式pid控制器
 * @param
 * 		hpid			pid结构体
 * 		val_now			当前值
 * 		target_now		目标值
 * @retval 	控制量
 *
 */
float PID_Incr( PID_TypeDef *hpid, float val_now, float target_now )
{
	hpid->Err_now = target_now - val_now;

	hpid->p_out = hpid->Kp * ( hpid->Err_now - hpid->Err_last );
	hpid->i_out = hpid->Ki * ( hpid->Err_now );
	hpid->d_out = hpid->Kd * ( hpid->Err_now - hpid->Err_last - hpid->Err_diff );

	hpid->Output += hpid->p_out + hpid->i_out + hpid->d_out;

	F_Output_Limit( hpid );

	hpid->Err_diff = hpid->Err_now - hpid->Err_last;
	hpid->Err_last = hpid->Err_now;

	return hpid->Output;
}


/*
 * @brief  	清除PID积分值和输出
 * @param	pid结构体
 * @retval 	无
 *
 */
void PID_Clear( PID_TypeDef *hpid )
{

    hpid->Err_diff = 0;
    hpid->Err_former = 0;
    hpid->Err_last = 0;
    hpid->Err_now = 0;
    hpid->Err_sum = 0;

#if TEST_I
	hpid->target = 0;
	hpid->last_target = 0;
	hpid->measure = 0;
	hpid->last_measure = 0;
	hpid->d_out_last = 0;
	hpid->i_term = 0;
#if TEST_D
	hpid->Dead_Band = 0.0f;
#else
#endif
#else
#endif
}
