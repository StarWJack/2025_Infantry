/*
 * @file		WatchDog.c/h
 * @brief	一个简易版本的看门狗程序
 * @history
 * 版本			作者			编写日期
 * v1.0.0		郑煜壅		2023/4/1
 *
 */

#include "main.h"
#include "WatchDog.h"
#include "config.h"
#include "FreeRTOS.h"
#include "ErrorHandel.h"

WatchDog_t remote_WatchDog;
WatchDog_t referee_WatchDog;
WatchDog_t motor_WatchDog[MOTOR_NUM];
/*
 * @brief	喂狗程序代码，放在需要进行检测的程序段中
 * @param	对应的看门狗指针（由用户自定义）
 * @retval	无
 */
void feedDog(WatchDog_t* hdog)
{
	hdog->status = 1;
	hdog->last_time = uwTick;
}
/*
 * @brief	看门狗状态更新，放在任意能够稳定运行的程序段中
 * @param	对应的看门狗指针（由用户自定义）
 * @retval	无
 */
uint8_t Dog_Status_update(WatchDog_t* hdog)
{
	if(uwTick - hdog->last_time > LOSTTIME_MAX)
		hdog->status = 0;
	return hdog->status;
}
/*
 * @brief	看门狗状态初始化
 * @param	无
 * @retval	无
 */
void WatchDogInit(void)
{
    feedDog(&remote_WatchDog);
    feedDog(&referee_WatchDog);
    for(int i=0;i <MOTOR_NUM -1;i++)
        feedDog(&motor_WatchDog[i]);
}
/*
 * @brief	看门狗检测掉线
 * @param	无
 * @retval	任一看门狗掉线，调用错误处理程序
 */
void WatchDogTask(void)
{
    for(int i=0;i <MOTOR_NUM -1;i++)
        if(!Dog_Status_update(&motor_WatchDog[i]))
            sErrorHandel(i);
    if(!(Dog_Status_update(&remote_WatchDog) && Dog_Status_update(&referee_WatchDog)))
        sErrorHandel(SERIALLOST);
}
