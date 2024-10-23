#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__

#include "main.h"

#define GRAVITY 9.78
#define vision_gap  0.0001


enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};


//设置参数
struct SolveTrajectoryParams
{
    float k;             //弹道系数

    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v;      //当前弹速
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw

    //目标参数
    float xw;             //ROS坐标系下的x
    float yw;             //ROS坐标系下的y
    float zw;             //ROS坐标系下的z
    float vxw;            //ROS坐标系下的vx
    float vyw;            //ROS坐标系下的vy
    float vzw;            //ROS坐标系下的vz
    float tar_yaw;        //目标yaw
    float v_yaw;          //目标yaw速度
    float r1;             //目标中心到前后装甲板的距离
    float r2;             //目标中心到左右装甲板的距离
    float dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;        //偏置时间
    float s_bias;         //枪口前推的距离
    float z_bias;         //yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵
    enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
};

//用于存储目标装甲板的信息
struct tar_pos
{
    float x;           //装甲板在世界坐标系下的x
    float y;           //装甲板在世界坐标系下的y
    float z;           //装甲板在世界坐标系下的z
    float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
};

extern void GimbalControlInit(float pitch, float yaw, float tar_yaw , float v_yaw, float r1, float r2, float z2, uint8_t armor_type, float v, float k);
extern float GimbalControlBulletModel(float x, float v, float angle);
extern float GimbalControlGetPitch(float x, float y, float v);

extern void GimbalControlTransform(float xw, float yw, float zw,
                                float vxw, float vyw, float vzw,
                                int timestamp_start, float *pitch, float *yaw,
                                float *aim_x, float *aim_y, float *aim_z);
extern void GimbalControlTransformYaw(float xw, float yw,
        float vxw, float vyw,
        int timestamp_start,float *yaw);
extern float angle_yaw,angle_pitch,pitch,yaw,aim[3];

extern int idx;
extern struct tar_pos tar_position[4];

#endif /*__SOLVETRAJECTORY_H__*/
