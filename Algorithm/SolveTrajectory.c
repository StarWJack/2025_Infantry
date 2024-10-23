// 弹道结算
// 只考虑水平方向的空气阻力

#include <math.h>
#include <stdio.h>
#include "SolveTrajectory.h"
#include "Serial.h"
#include "referee.h"

struct SolveTrajectoryParams st =
        {
                .s_bias = 0.05,
                .z_bias = 0.0
        };
struct tar_pos tar_position[4];
int idx = 0;
float t = 0.0f; // 飞行时间
float aim[3];

/*
@brief 初始化
@param pitch:rad
@param yaw:rad
@param v:m/s
@param k:弹道系数
*/
void GimbalControlInit(float pitch, float yaw, float tar_yaw, float v_yaw,
                        float r1, float r2, float dz, uint8_t armor_num,
                        float v, float k)
{
    st . current_pitch = pitch;
    st . current_yaw = yaw;
    st . current_v = v;
    st . k = k;
    st . tar_yaw = tar_yaw;
    st . v_yaw = v_yaw;
    st . r1 = r1;
    st . r2 = r2;
    st . dz = dz;
    st . armor_num = armor_num;
}

/*
@brief 弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return y:m
*/
float GimbalControlBulletModel(float s, float v, float angle)
{
    //t为给定v与angle时的飞行时间
    t = (float) ((expf(st . k * s) - 1) / (st . k * v * cosf(angle)));
    if(t < 0 || t > 5)
    {
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    float z = (float) (v * sinf(angle) * t - GRAVITY * t * t / 2);
    return z;
}

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float GimbalControlGetPitch(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float tmp_angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for(i = 0; i < 20; i ++)
    {
        tmp_angle_pitch = atan2f(z_temp, s); // rad
        z_actual = GimbalControlBulletModel(s, v, tmp_angle_pitch);
        if(! z_actual)
        {
            tmp_angle_pitch = 0;
            break;
        }
        dz = 0.3 * (z - z_actual);
        z_temp = z_temp + dz;
        if(fabsf(dz) < vision_gap)
        {
            break;
        }
    }
    return tmp_angle_pitch;
}

/*
@brief 世界坐标系转换到云台坐标系
@param xw:ROS坐标系下的x
@param yw:ROS坐标系下的y
@param zw:ROS坐标系下的z
@param vxw:ROS坐标系下的vx
@param vyw:ROS坐标系下的vy
@param vzw:ROS坐标系下的vz
@param timestamp_start:开始时间戳
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
*/
void GimbalControlTransformYaw(float xw, float yw,
                               float vxw, float vyw,
                               int timestamp_start, float *yaw)
{
    int timestamp_now = timestamp_start + 70; // 假设当前时间戳=开始时间戳+50ms
    //旧步兵70，新步兵--
    // TODO：获取当前时间戳
    // 线性预测
    // 计算通信及解算时间戳延时+子弹飞行时间  考虑了200ms的通信延时
    float timeDelay = (float) ((timestamp_now - timestamp_start) / 1000.0) + t;

    xw += vxw * timeDelay;
    yw += vyw * timeDelay;
    *yaw = (float) (atan2f(yw, xw));
}

void GimbalControlTransform(float xw, float yw, float zw,
                            float vxw, float vyw, float vzw,
                            int timestamp_start, float *pitch, float *yaw,
                            float *aim_x, float *aim_y, float *aim_z)
{
    idx = 0;
    int timestamp_now = timestamp_start + 150; // 假设当前时间戳=开始时间戳+50ms
    // TODO：获取当前时间戳
    st . xw = xw;
    st . yw = yw;
    st . zw = zw;
    st . vxw = vxw;
    st . vyw = vyw;
    st . vzw = vzw;
    // 线性预测
    // 计算通信及解算时间戳延时+子弹飞行时间  考虑了200ms的通信延时
    float timeDelay = (float) ((timestamp_now - timestamp_start) / 1000.0) + t;
    st . tar_yaw += st . v_yaw * timeDelay;
    //计算四块装甲板的位置
    int use_1 = - 1;
    int i = 0;
    if(st . armor_num == ARMOR_NUM_OUTPOST)
    {  //前哨站
        for(i = 0; i < 3; i ++)
        {
            float tmp_yaw = st . tar_yaw + i * 2.0 * PI / 3.0;  // 2/3PI
            float r = (st . r1 + st . r2) / 2;   //理论上r1=r2 这里取个平均值
            tar_position[i] . x = st . xw - r * cosf(tmp_yaw);
            tar_position[i] . y = st . yw - r * sinf(tmp_yaw);
            tar_position[i] . z = st . zw;
            tar_position[i] . yaw = tmp_yaw;
        }
        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用
    }
    else
    {
        for(i = 0; i < 4; i ++)
        {
            float tmp_yaw = st . tar_yaw + i * PI / 2.0;
            float r = use_1 ? st . r1 : st . r2;
            tar_position[i] . x = st . xw - r * cosf(tmp_yaw);
            tar_position[i] . y = st . yw - r * sinf(tmp_yaw);
            tar_position[i] . z = use_1 ? st . zw : st . zw + st . dz;
            tar_position[i] . yaw = tmp_yaw;
            use_1 = ! use_1;
        }
        //2种常见决策方案：
        //1.计算枪管到目标装甲板yaw最小的那个装甲板
        //2.计算距离最近的装甲板

        //计算枪管到目标装甲板yaw最小的那个装甲板
//	        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//	        for (i = 1; i<4; i++) {
//	            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//	            if (temp_yaw_diff < yaw_diff_min)
//	            {
//	                yaw_diff_min = temp_yaw_diff;
//	                idx = i;
//	            }
//	        }

        //计算距离最近的装甲板
        float dis_diff_min =
                sqrt(tar_position[0] . x * tar_position[0] . x + tar_position[0] . y * tar_position[0] . y);
        idx = 0;
        for(i = 1; i < 4; i ++)
        {
            float temp_dis_diff =
                    sqrt(tar_position[i] . x * tar_position[0] . x + tar_position[i] . y * tar_position[0] . y);
            if(temp_dis_diff < dis_diff_min)
            {
                dis_diff_min = temp_dis_diff;
                idx = i;
            }
        }

    }

    *aim_z = tar_position[idx] . z + st . vzw * timeDelay;
    *aim_x = tar_position[idx] . x + st . vxw * timeDelay;
    *aim_y = tar_position[idx] . y + st . vyw * timeDelay;
    //这里符号错了
    if(*aim_y || *aim_x)
        *yaw = (float) (atan2f(*aim_y, *aim_x));
    float temp_pitch =
            - GimbalControlGetPitch(sqrtf(((*aim_y) * (*aim_y)) + ((*aim_x) * (*aim_x))), *aim_z, st . current_v);
    if(temp_pitch)
        *pitch = temp_pitch;
}
// 从坐标轴正向看向原点，逆时针方向为正

