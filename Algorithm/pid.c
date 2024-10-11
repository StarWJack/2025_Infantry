#include "pid.h"

#if TEST_I
/* ----------------------------下面是pid优化环节的实现---------------------------- */
//TODO(GUATAI)dt获取需要进一步联动DWT内核定时器
// 梯形积分
static void Trapezoid_Intergral( PID_TypeDef *hpid )
{
	// 计算梯形的面积,(上底+下底)*高/2
	hpid->i_error = hpid->Ki * ( ( hpid->Err_now + hpid->Err_last ) / 2 ) * hpid->dt;
}

// 变速积分(误差小时积分作用更强)
static void Changing_Integration_Rate( PID_TypeDef *hpid )
{
	if ( hpid->Err_now * hpid->i_out > 0 )
	{
		// 积分呈累积趋势
		if ( abs( hpid->Err_now ) <= hpid->ci_coefB )
		{
			return;
		} // Full integral
		if ( abs( hpid->Err_now ) <= ( hpid->ci_coefA + hpid->ci_coefB ) )
		{
			hpid->i_term *= ( hpid->ci_coefA - abs( hpid->Err_now ) + hpid->ci_coefB ) / hpid->ci_coefA;
		}
		else
		{ // 最大阈值,不使用积分
			hpid->i_error = 0;
		}
	}
}

//积分限幅
static void Integral_Limit( PID_TypeDef *hpid )
{
	static float temp_Output, temp_Iout;
	temp_Iout = hpid->i_out + hpid->i_term;
	temp_Output = hpid->p_out + hpid->i_out + hpid->d_out;
	if ( abs( temp_Output ) > hpid->Output_Max )
	{
		if ( hpid->Err_now * hpid->i_out > 0 ) // 积分却还在累积
		{
			hpid->i_term = 0; // 当前积分项置零
		}
	}

	if ( temp_Iout > hpid->integral_limit )
	{
		hpid->i_term = 0;
		hpid->i_out = hpid->integral_limit;
	}
	if ( temp_Iout < -hpid->integral_limit )
	{
		hpid->i_term = 0;
		hpid->i_out = -hpid->integral_limit;
	}
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void Derivative_On_Measurement( PID_TypeDef *hpid )
{
	hpid->d_out = hpid->Kd * ( hpid->last_measure - hpid->measure ) / hpid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void Derivative_Filter( PID_TypeDef *hpid )
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
static void Output_Filter( PID_TypeDef *hpid )
{
	hpid->dt = DWT_GetDeltaT( &hpid->DWT_cnt );
	hpid->Output = hpid->Output * hpid->dt / ( hpid->output_LPF_RC + hpid->dt ) +
				   hpid->Output_last * hpid->output_LPF_RC / ( hpid->output_LPF_RC + hpid->dt );
}

#else
#endif

// 输出限幅
static void Output_Limit( PID_TypeDef *hpid )
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
	hpid->dt= DWT_GetDeltaT(&hpid->DWT_cnt);

	hpid->measure=hpid->last_measure;
	hpid->last_target=hpid->last_target;
	hpid->measure=val_now;
	hpid->target=target_now;

#else
#endif
#if TEST_D
	if(hpid->Err_now<hpid->Dead_Band)
	{
#else
#endif
#if TEST_I
	hpid->p_out = hpid->Kp * hpid->Err_now;
	hpid->i_term = hpid->Ki * hpid->Err_now * hpid->dt;
	hpid->d_out = hpid->Kd * ( hpid->Err_now - hpid->Err_last ) / hpid->dt;

	//TODO(GUATAI)0为暂时的占位提示，后续添加标志位识别
	// 梯形积分
	if (hpid->improve & 0)
		Trapezoid_Intergral(hpid);
	// 变速积分
	if (hpid->improve & 0)
		Changing_Integration_Rate(hpid);
	// 微分先行
	if (hpid->improve & 0)
		Derivative_On_Measurement(hpid);
	// 微分滤波器
	if (hpid->improve & 0)
		Derivative_Filter(hpid);
	// 积分限幅
	if (hpid->improve & 0)
		Integral_Limit(hpid);

	hpid->i_out += hpid->i_term;
	hpid->Output = hpid->p_out + hpid->i_out + hpid->d_out;

	// 输出滤波
	if (hpid->improve & 0)
		Output_Filter(hpid);

	Output_Limit(hpid);
#else
	hpid->Err_sum += hpid->Err_now;
	if ( hpid->Err_sum < -hpid->Err_sum_Max )
	{ hpid->Err_sum = -hpid->Err_sum_Max; }
	if ( hpid->Err_sum > hpid->Err_sum_Max )
	{ hpid->Err_sum = hpid->Err_sum_Max; }

	hpid->Output = hpid->Kp * hpid->Err_now + hpid->Ki * hpid->Err_sum
			 + ( hpid->Kd ) * ( hpid->Err_now - hpid->Err_last );

	hpid->Output += hpid->Kf * ( hpid->Output - hpid->Output_last );//前馈

	Output_Limit(hpid);
#endif
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
	hpid->Output_last = hpid->Output;

	return hpid->Output;
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
float PID_Incr( PID_TypeDef *hpid, float val_now, float target_val_now )
{
	hpid->Err_now = target_val_now - val_now;
	float Output;
#if TEST_D
	if(hpid->Err_now<hpid->Dead_Band)
	{
#else
#endif
	Output = hpid->Kp * ( hpid->Err_now - hpid->Err_last ) + hpid->Ki * ( hpid->Err_now ) +
			 hpid->Kd * ( hpid->Err_now - hpid->Err_last - hpid->Err_diff );
	if ( Output > hpid->Output_Max )
	{ Output = hpid->Output_Max; }
	if ( Output < -hpid->Output_Max )
	{ Output = -hpid->Output_Max; }
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
void PID_Clear( PID_TypeDef *hpid )
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
