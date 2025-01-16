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



/*
//zy版本
// system output -> {est_x, est_y, est_z}
// 选板+预测开火时目标装甲板位置（转头+拨弹+发射存在时间延迟）
#include "SolveTrajectory.h"
#include "Serial.h"

//装甲板宽度
#define LARGE_ARMOR_WIDTH 0.23
#define SMALL_ARMOR_WIDTH 0.135

#define YAW_MOTOR_RES_SPEED 1.6f

uint8_t armor_choose = 1;
uint8_t suggest_fire = 1;

extern uint32_t now_timestamp;//当前时间戳

//  反斜杠 \ 是一个行连接符，它的作用是将多行代码连接成一行

#define VISION_DATA_ASSIGNMENT                         \
 uint8_t target = vision_ctrl_data.target;            \
 uint8_t id_num = vision_ctrl_data.id_num;            \
 float yaw = vision_ctrl_data.yaw;                    \
 float r1 = vision_ctrl_data.r1;                      \
 float r2 = vision_ctrl_data.r2;                      \
 float xc = vision_ctrl_data.x;                       \
 float yc = vision_ctrl_data.y;                       \
 float zc = vision_ctrl_data.z;                       \
 float vx = vision_ctrl_data.vx;                      \
 float vy = vision_ctrl_data.vy;                      \
 float vz = vision_ctrl_data.vz;                      \
 float vyaw = vision_ctrl_data.v_yaw;                 \
 float dz = vision_ctrl_data.dz;                      \
 uint8_t armor_num = vision_ctrl_data.armor_num;      \
 uint32_t r_timestamp = vision_ctrl_data.r_timestamp; \
 uint16_t first_phase_1 = vision_ctrl_data.first_phase;


void expected_preview_calc(void) {
 float armor_x = 0.f, armor_y = 0.f, armor_z = 0.f;//存放结算后的装甲板坐标

 VISION_DATA_ASSIGNMENT

 float allow_fire_ang_max = 0.f, allow_fire_ang_min = 0.f;//允许开火的最大最小角度

 // aim_status judge
 if (target == 1) {
   // Prediction
   // predict_time --> shoot delay
   // (now_timestamp - r_timestamp) --> vision calculate using time
   // 计算最终预测时间，包括当前时间戳与接收时间戳的差值和预测时间
   float predict_time_final =
       (float)(now_timestamp - r_timestamp) / 1000 + predict_time;

   //根据预测时间和速度更新 目标装甲板的坐标（以自己云台为中心）
   xc = xc + predict_time_final * vx;
   yc = yc + predict_time_final * vy;
   zc = zc + predict_time_final * vz;

   // 根据预测时间和偏航速度更新 目标装甲板的偏航角
   float predict_yaw = yaw + predict_time_final * vyaw;
   // 根据预测时间和速度更新 目标装甲板的坐标
   float center_theta = atan2(yc, xc);

   // 选择敌方装甲板摆放方式；1--四个等高 ； 
   uint8_t use_1 = 1;
   // 计算装甲板之间的角度差
   float diff_angle = 2 * PI / armor_num;

   //
   for (size_t i = 0; i < armor_num; i++) {

     float armor_yaw = predict_yaw + i * diff_angle;
     // 计算装甲板偏航角与中心角度的差值
     float yaw_diff = get_delta_ang_pi(armor_yaw, center_theta);//计算装甲板与偏航角与中心？？？

     // 如果偏航角差值小于角度差的一半，选择该装甲板
     if (fabsf(yaw_diff) < diff_angle / 2) {
       armor_choose = 1;

       // Only 4 armors has 2 radius and height 
       float r = r1;
       armor_z = zc;
       if (armor_num == 4) {
         r = use_1 ? r1 : r2;//use_1==1    r=r1
         armor_z = use_1 ? zc : (zc + dz);
       }

       // Robot state to armor
       armor_x = xc - r * cos(armor_yaw);
       armor_y = yc - r * sin(armor_yaw);

       // Calculate angle of advance

       float armor_z_next = zc;
       if (armor_num == 4) {
         r = !use_1 ? r1 : r2;
         armor_z_next = !use_1 ? zc : (zc + dz);
       }
        //计算下一个装甲板
       float next_armor_yaw =
           armor_yaw -
           sign(vyaw) * diff_angle;  // TODO: use future span to calculate
                                     // target delta angle
       float armor_x_next = xc - r * cos(next_armor_yaw);
       float armor_y_next = yc - r * sin(next_armor_yaw);
        //计算前进角度
       float yaw_motor_delta =
           get_delta_ang_pi(atan2(armor_y, armor_x),
                            atan2(armor_y_next, armor_x_next));
       float angle_of_advance =
           fabsf(yaw_motor_delta) / YAW_MOTOR_RES_SPEED * fabsf(vyaw) / 2;
           
        // 根据前进角度和偏航角差值选择最终的装甲板坐标和偏航角
       float est_yaw;
       if (sign(vyaw) * yaw_diff < diff_angle / 2 - angle_of_advance ||
           angle_of_advance > diff_angle / 4) {
         est_x = armor_x;
         est_y = armor_y;
         est_z = armor_z;
         est_yaw = armor_yaw;
       } else {
         est_x = armor_x_next;
         est_y = armor_y_next;
         est_z = armor_z_next;
         est_yaw = next_armor_yaw;
       }

       // Calculate fire control params
       float armor_w;
       if (armor_num == 2 || id_num == 1)
         armor_w = LARGE_ARMOR_WIDTH;
       else
         armor_w = SMALL_ARMOR_WIDTH;
       float ax = est_x - 0.5f * armor_w * sin(est_yaw);
       float ay = est_y + 0.5f * armor_w * cos(est_yaw);
       float bx = est_x + 0.5f * armor_w * sin(est_yaw);
       float by = est_y - 0.5f * armor_w * cos(est_yaw);
       float angle_a = atan2(ay, ax);
       float angle_b = atan2(by, bx);
       float angle_c = atan2(est_y, est_x);
       allow_fire_ang_max = angle_c - angle_b;
       allow_fire_ang_min = angle_c - angle_a;

       break;
     } else {
       armor_choose = 0;
     }
     use_1 = !use_1;
   }
 }
 fire_ctrl(allow_fire_ang_max, allow_fire_ang_min, target);
}

void fire_ctrl(float allow_fire_ang_max, float allow_fire_ang_min,
              uint8_t target) {
 if (target == 0 || armor_choose == 0) {
   suggest_fire = 0;
 } else if (target == 1) {
   // yaw_ang_ref
   float control_delta_angle =
       get_delta_ang_pi(yaw_ang_ref / 4096, _imu_eular.yaw / 4096);
   suggest_fire = (control_delta_angle < allow_fire_ang_max &&
                   control_delta_angle > allow_fire_ang_min);
 } else {
   // ERROR
   suggest_fire = 0;
 }
}
*/
