#include "bomb_detect.h"
#include "motor.h"
#include "math.h"

Bomb_StatusTypedef bomb_status;

/*
 * @brief	检测弹丸是否发射成功，不存在卡弹现象
 * @param	无
 * @retval	通过发射弹丸时的电机转速大大下降现象，判断是否发射成功
 */
void bomb_Status_update(void)
{
    if( (BULLET_V - fabs(motor_list[MOTOR_SHT_L]->data.speed) > MOTORSPEED_DIFF) && (BULLET_V - fabs(motor_list[MOTOR_SHT_R]->data.speed) > MOTORSPEED_DIFF))
        bomb_status.status = 1;
}

void bomb_detect_init(void)
{
    bomb_status.status = 1;
}