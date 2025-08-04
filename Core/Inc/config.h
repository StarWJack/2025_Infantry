#ifndef __CONFIG_H__
#define __CONFIG_H__

// 普通兵种代码框架，根据车体选择，同时为1应会报错
// 默认为步兵
#define INFANTRY 1

//-------------------- 控制器 -------------------------
//#define USE_REMOTE    			//若使用遥控器传输数据，取消注释
#define USE_KEYBOARD                //若使用键盘传输数据(图传链路)，取消注释
//#define USE_REMOTE_KEYBORAD		//使用遥控器传输键盘(遥控器链路)

//--------------------  电机  -------------------------
#if INFANTRY
#define MOTOR_NUM 9 // 电机总数
#define MOTOR_YAW 1 // 电机列表索引测试用
#define MOTOR_PITCH 0
#define MOTOR_CHS_1 2 // chassis_info motor
#define MOTOR_CHS_2 3
#define MOTOR_CHS_3 4
#define MOTOR_CHS_4 5
#define MOTOR_AMMO 6  // ammofeeder motor  ID=7
#define MOTOR_SHT_L 7 // left shooter motor ID=2,1 , 从正面到背面的方向去看
#define MOTOR_SHT_R 8

// CAN
#define CAN_HEADER_NUM 3 // CAN标识符数量
#define CAN_INST_NUM 10	 // CAN设备总数
#define CAN_NUM 2		 // CAN控制器数量
#endif

// IMU
#define IMU_NUM 1
#define IMU_YAW 0

// WatchDog
#define LOSTTIME_MAX 1000 // 此处修改最小判断离线时间，单位ms

// freeRTOS任务周期
#define CONTROL_TASK_PERIOD 10 // freeRTOS执行周期, ms  1000HZ?
#define CHASSIS_TASK_PERIOD 10
#define GIMBAL_TASK_PERIOD 4
#define CANBUS_TASK_PERIOD 1
#define IMU_TASK_PERIOD 1
#define UI_TASK_PERIOD 40


// 遥控器灵敏度设置
// #define REMOTE_YAW_SEN 0.03f  //

#define REMOTE_YAW_SEN 0.00003f //
//#define REMOTE_YAW_SEN 0.00001f //

// #define REMOTE_YAW_SEN 0.005  //
// #define REMOTE_PITCH_SEN 0.0000006125f
#define REMOTE_PITCH_SEN 0.000006125f
#define REMOTE_X_SEN 0.002f
#define REMOTE_Y_SEN 0.002f
// -----------------------------------------------
// 键盘灵敏度设置
#define KEYBOARD_YAW_SEN 0.00005f
#define KEYBOARD_PITCH_SEN 0.00005f


// 车体数据
#if INFANTRY
#define WHEEL_RADIUS 0.075f // 底盘车轮半径, 米
#define CHASSIS_RADIUS 0.214f
#define FRICTION_RADIUS 0.03 // 摩擦轮半径
#endif


/*步兵1*/

#define PITCH_UP_BOUND (8192.0f - 7708.0f)
// #define PITCH_BOUND       (300.0f - 7850.0f)   pitch电机 编码值范围
// #define PITCH_BOUND       (PI - * PI)    pitch电机 陀螺仪的范围

#define PITCH_LL_BOUND (PI - 0.18)													// pitch轴下界极限， +0.025确保安全，极限实际为PI
#define PITCH_LH_BOUND ((8192.0f - 7850.0f) + 300) * 2 * PI / 8192 + PITCH_LL_BOUND // pitch轴上界极限， 由于走线限制
// #define PITCH_LW_BOUND  400.0f    //pitch轴可动范围上下界
#define PITCH_MID 34.0f // pitch轴中间位置

// #define PITCH_UP_BOUND	3.58f
// #define PITCH_LW_BOUND  PI	//pitch轴可动范围上下界
// #define PITCH_MID       (0 +PI)  //pitch轴中间位置
#define CHASSIS_ZEROPOINT    5155.0f
//#define CHASSIS_ZEROPOINT    5121.0f	 // 在底盘坐标系中，云台yaw正方向与底盘x轴平齐时yaw轴电机角度 --- 底盘坐标系x轴要结合各电机ID确定
#define CHASSIS_ZEROPOINT_FOLLOW 5766.0f // 在底盘跟随时的零点
#define AMMO_DIR_INV					 // 拨弹盘为顺时针旋转

// 性能
//  #define SHOOT_FREQ 15    //射频, hz
#define SHOOT_FREQ 20 // 射频, hz
// #define BULLET_V 25     //弹速, m/smotor_list[7]->pid[1].target
#define BULLET_V 35		 // 弹速, m/s     34.0f---24
#define MOTORSPEED_DIFF 2.0f // 弹速差异，m/s
#define CALIBRATE 1			 // 弹速校准比率
//#define CHS_SPEED_LMT 2.5f

#define CHS_SPIN_LMT 0.5 * PI // 底盘自旋速度

// 比赛数据
#define DETECT_COLOR 1 // 0-red 1-blue //gimbal中更新修改
#define TASK_MODE 0	   // 0-auto 1-aim 2-buff


typedef enum
{
	ABS = 0,
	INCR = 1
} ValSet_Type_e;


typedef enum
{
	OUTER = 0,
	INNER = 1,
} PID_LOOP_e;

#endif

// TODO:5、看门狗

