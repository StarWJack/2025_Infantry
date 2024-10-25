#ifndef __CONFIG_H__
#define __CONFIG_H__

//普通兵种代码框架，根据车体选择，同时为1应会报错
//默认为步兵
#define HERO 0
#define INFANTRY 1
//电机
#if INFANTRY
#define MOTOR_NUM 9     //电机总数
#define MOTOR_YAW 1     //电机列表索引测试用
#define MOTOR_PITCH 0
#define MOTOR_CHS_1 2   //chassis motor
#define MOTOR_CHS_2 3
#define MOTOR_CHS_3 4
#define MOTOR_CHS_4 5
#define MOTOR_AMMO 6    //ammofeeder motor  ID=7
#define MOTOR_SHT_L 7   //left shooter motor ID=2,1
#define MOTOR_SHT_R 8


//CAN
#define CAN_HEADER_NUM 2    //CAN标识符数量
#define CAN_INST_NUM 9      //CAN设备总数
#define CAN_NUM 2           //CAN控制器数量
#define USE_CAN2 0        //若使用CAN2，取消注释
#define TOPIC_NUM 0
#endif

#if HERO
//TODO(GUATAI)英雄实例数据
#define MOTOR_NUM 10     //电机总数
#define MOTOR_YAW 1     //电机列表索引测试用
#define MOTOR_PITCH 0
#define MOTOR_CHS_1 2   //chassis motor
#define MOTOR_CHS_2 3
#define MOTOR_CHS_3 4
#define MOTOR_CHS_4 5
#define MOTOR_AMMO 6    //ammofeeder motor  ID=7
#define MOTOR_SHT_L 7   //left shooter motor ID=2,1
#define MOTOR_SHT_R 8   //right shooter motor ID
#define MOTOR_SHT_U 9   //up shooter motor ID

//CAN
#define CAN_HEADER_NUM 2    //CAN标识符数量
#define CAN_INST_NUM 10      //CAN设备总数
#define CAN_NUM 2           //CAN控制器数量
#define USE_CAN2 0        //若使用CAN2，取消注释
#define TOPIC_NUM 0
#endif

//IMU
#define IMU_NUM 1
#define IMU_YAW 0

//WatchDog
#define LOSTTIME_MAX 1000 //此处修改最小判断离线时间，单位ms

//freeRTOS任务周期
#define CONTROL_TASK_PERIOD 10   //freeRTOS执行周期, ms
#define GIMBAL_TASK_PERIOD 10
#define CHASSIS_TASK_PERIOD 10
#define CANBUS_TASK_PERIOD 1

//遥控器灵敏度设置
#define REMOTE_YAW_SEN 0.0000008f
#define REMOTE_PITCH_SEN 0.0000006125f
#define REMOTE_X_SEN    0.002f
#define REMOTE_Y_SEN    0.002f

//键盘灵敏度设置
#define  KEYBOARD_YAW_SEN 0.001f
#define  KEYBOARD_PITCH_SEN 0.001f

//控制器
//#define USE_REMOTE_KEYBORAD     //若使用遥控器传输键鼠数据，取消注释
//#define USE_KEYBOARD              //若使用键盘传输数据，取消注释

//车体数据
#if INFANTRY
#define WHEEL_RADIUS 0.075f  //底盘车轮半径, 米
#define CHASSIS_RADIUS 0.214f
#define FRICTION_RADIUS 0.03    //摩擦轮半径
#endif

/*步兵1*/

// #define PITCH_UP_BOUND	2*PI*7708/8192.0f
// #define PITCH_LW_BOUND  2*PI*400/8192.0f	//pitch轴可动范围上下界
// #define PITCH_MID       2*PI*34/8192.0f  //pitch轴中间位置
#define PITCH_UP_BOUND	3.58f
#define PITCH_LW_BOUND  PI	//pitch轴可动范围上下界
#define PITCH_MID       (0 +PI)  //pitch轴中间位置
#define CHASSIS_ZEROPOINT   7851.0f	//在底盘坐标系中，云台yaw与底盘x平行时yaw轴电机角度
#define AMMO_DIR_INV //拨弹盘为顺时针旋转


#if HERO
//轴距（wheelbase） 即前后轮轴间距离
#define LENGTH 0.36f
//轮距（track） 即左右驱动轮中心距离
#define WIDTH 0.33f
//驱动轮直径（diameter）
#define WHEEL_RADIUS 0.154f

#define PITCH_UP_BOUND	3.58f
#define PITCH_LW_BOUND  PI	//pitch轴可动范围上下界
#define PITCH_MID       (0 +PI)  //pitch轴中间位置
#define CHASSIS_ZEROPOINT   7851.0f	//在底盘坐标系中，云台yaw与底盘x平行时yaw轴电机角度
#define AMMO_DIR_INV //拨弹盘为顺时针旋转
#endif


//性能
#define SHOOT_FREQ 15    //射频, hz
#define BULLET_V 25     //弹速, m/s
#define MOTORSPEED_DIFF 2.0f //弹速差异，m/s
#define CALIBRATE 1     //弹速校准比率
#define CHS_SPEED_LMT   2.0f
#define CHS_SPIN_LMT    0.5*PI //底盘自旋速度

//other
//#define ABS 0
//#define INCR 1

typedef enum
{
    ABS=0,
    INCR=1
}ValSet_Type_e;



//#define OUTER 0
//#define INNER 1
//#define FEED 2

typedef enum
{
    OUTER=0,
    INNER=1,
}PID_LOOP_e;




#endif


//TODO:1、火控
//TODO:2、功率控制
//TODO:3、nuc串口通信
//TODO:4、裁判系统通信
//TODO:5、看门狗
//TODO:6、遥控器启动不接收
//TODO:7、拨弹盘反转（卡弹时）
