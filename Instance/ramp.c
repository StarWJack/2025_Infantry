/*
 * @brief	  粗略版斜坡函数(使用整形运算来优化运行，防止大量浮点数运算溢出)
 * @history
 * 	版本			作者		  编写日期			内容
 * 	v1.0		  FJJ		  2025/3/10		 适配键盘控制模式
 */
#include <stdint.h>
#include <stdio.h>
#include "ramp.h"
#include "Control.h"

/*
 * @brief  	专门应用与按键的斜坡函数
 * @param	x -- 输入值(0到1)
 * @param   y -- 输出值(0到100)
 * @retval 	y -- 输出值(0到100)
 */
uint8_t Ramp_data1(uint16_t x)
{
    if(x)
      y_1 = (y_1 + RAMP_STEP)> 100 ? 100 :(y_1 + RAMP_STEP);
    else
      y_1 = (y_1 - RAMP_STEP)< 0 ? 0 :(y_1 - RAMP_STEP);
    return y_1;
}

uint8_t Ramp_data2(uint16_t x)
{
    if(x)
      y_2 = (y_2 + RAMP_STEP)> 100 ? 100 :(y_2 + RAMP_STEP);
    else
      y_2 = (y_2 - RAMP_STEP)< 0 ? 0 :(y_2 - RAMP_STEP);
    return y_2;
}

uint8_t Ramp_data3(uint16_t x)
{
    if(x)
      y_3 = (y_3 + RAMP_STEP)> 100 ? 100 :(y_3 + RAMP_STEP);
    else
      y_3 = (y_3 - RAMP_STEP)< 0 ? 0 :(y_3 - RAMP_STEP);
    return y_3;
}

uint8_t Ramp_data4(uint16_t x)
{
    if(x)
      y_4 = (y_4 + RAMP_STEP)> 100 ? 100 :(y_4 + RAMP_STEP);
    else
      y_4 = (y_4 - RAMP_STEP)< 0 ? 0 :(y_4 - RAMP_STEP);
    return y_4;
}
