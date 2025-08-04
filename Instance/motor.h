#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "can.h"
#include "pid.h"
#include "CanInst.h"
typedef enum
{
    OTHER,
    GM6020,
	M3508,
	M2006
}Motor_e;

typedef enum
{
    MOTORANGLE,
    MOTORSPEED,
    IMU
}MOTOR_Feedback_e;

//#define ORIGIN 0
//#define RAD 1
typedef enum
{
    ORIGIN,
    RAD
}DataType_e;

typedef struct
{
    uint16_t shoot_count;
    uint16_t shoot_num;
}SHOOT_CLA;

typedef struct 
{
    //TODO: add watchdog
    PID_Type_e pid_type;
    Motor_e motor_type;
    uint8_t enable;
    float output_max;
}Motor_InfoTypeDef;

typedef struct
{
    uint16_t angle;
	int16_t speed;
	int16_t current;       //电流
	uint8_t temp;          //温度
    uint8_t unused;
}__attribute__((__packed__)) Motor_CbkMsg;

typedef struct _Motor_TypeDef
{
    Can_InfoTypedef can;
    Motor_CbkMsg data;
    float target;
    /* switching data source accroding to Motor_InfoTypeDef->type
    NOTE: don't read derectly, use MotorGetVal() instead */

    float (*value)(struct _Motor_TypeDef *st);//函数指针，存放一个函数 --- motor.value = get_motor_value; float val = motor.value(&motor); 
    float feed_fwd;        //feedforward when type='IMU'
    float output;
    float output_Position;
    PID_TypeDef pid[2];    //outer(or angle)(0)、inner(or speed)(1) circle
    Motor_InfoTypeDef info;
    float voltage;
}Motor_TypeDef;

typedef struct
{
    uint8_t can_id;
    uint16_t txheader_id; 
    uint16_t rxheader_id;
    PID_Type_e pid_type; 
    PID_TypeDef pid_outer; 
    PID_TypeDef pid_inner; 
    uint8_t list_id; 
    CAN_HandleTypeDef *hcan;
    Motor_e motor_type;
    float output_max;
    float *source;      //source of ref in outer pid loop, valid only when pid_type='IMU'  即读取
}Motor_InitTypedef;

extern void MotorStatusChange(Motor_TypeDef *motor, PID_Type_e newtype, float *source);
extern void MotorCalc(void);

extern void MotorInit(void);
extern void MotorSetTar(Motor_TypeDef *motor, float val, ValSet_Type_e type);
extern float MotorGetVal(Motor_TypeDef *motor,MOTOR_Feedback_e feedback,DataType_e type);
extern void MotorStop(Motor_TypeDef *motor);
extern void MotorRestart(Motor_TypeDef *motor);
void Chassis_PowerCtrl(void);
extern Motor_TypeDef *motor_list[];
#endif
