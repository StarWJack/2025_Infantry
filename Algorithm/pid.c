#include "pid.h"


// 输出限幅
static void F_Output_Limit (PID_TypeDef *hpid)
{
	if ( hpid -> Output > hpid -> Output_Max )
	{
		hpid -> Output = hpid -> Output_Max;
	}
	if ( hpid -> Output < - (hpid -> Output_Max))
	{
		hpid -> Output = - (hpid -> Output_Max);
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
float PID_Position (PID_TypeDef *hpid, float val_now, float target_now)
{

	//观测
	hpid->measure = val_now;
	hpid->target = target_now;

	hpid -> Err_now = target_now - val_now;

	hpid -> Err_sum += hpid -> Err_now;

	if ( hpid -> Err_sum < - hpid -> Err_sum_Max )
	{
		hpid -> Err_sum = - hpid -> Err_sum_Max;
	}
	if ( hpid -> Err_sum > hpid -> Err_sum_Max )
	{
		hpid -> Err_sum = hpid -> Err_sum_Max;
	}

	hpid -> p_out = hpid -> Kp * hpid -> Err_now;
	hpid -> i_out = hpid -> Ki * hpid -> Err_sum;
	hpid -> d_out = hpid -> Kd * (hpid -> Err_now - hpid -> Err_last);

	hpid -> Output = hpid -> p_out + hpid -> i_out + hpid -> d_out;

	F_Output_Limit (hpid);
	
	hpid -> Err_pre = hpid -> Err_last;
	hpid -> Err_last = hpid -> Err_now;
	hpid -> Output_last = hpid -> Output;
	
	return hpid -> Output;
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
float PID_Increment (PID_TypeDef *hpid, float val_now, float target_now)
{

	hpid->Err_now = target_now - val_now;

	hpid->p_out = hpid->Kp * ( hpid->Err_now - hpid->Err_last );
	hpid->i_out = hpid->Ki * ( hpid->Err_now );
	hpid->d_out = hpid->Kd * ( hpid->Err_now - hpid->Err_last - hpid->Err_pre );

	hpid->Output += hpid->p_out + hpid->i_out + hpid->d_out;
	
	F_Output_Limit( hpid );
	
	hpid->Err_pre = hpid->Err_now - hpid->Err_last;
	hpid->Err_last = hpid->Err_now;
//观测
	hpid->measure = val_now;
	hpid->target = target_now;

	hpid -> Output_last = hpid -> Output;
	return hpid->Output;
}

/*
 * @brief  	清除PID积分值和输出
 * @param	pid结构体
 * @retval 	无
 *
 */
void PID_Clear (PID_TypeDef *hpid)
{
	
	hpid -> Err_pre = 0;
	hpid -> Err_last = 0;
	hpid -> Err_now = 0;
	hpid -> Err_sum = 0;

}


//增量式前馈
void FeedForward_Input(PID_TypeDef *hpid, float tar)
{

	hpid -> Output_feed =0;
	if(ABS(tar) > 0.5f)
	{
		float tar_tmp = ABS(tar);
		hpid -> Output_feed =  FEED_PARAM0 + FEED_PARAM1 * tar_tmp + FEED_PARAM2 * tar_tmp* tar_tmp;
		hpid -> Output_feed = tar > 0 ? hpid -> Output_feed : -hpid -> Output_feed;
	}
}
