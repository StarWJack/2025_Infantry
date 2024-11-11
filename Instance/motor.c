#include "motor.h"
#include "pid.h"
#include "IMU.h"
#include "string.h"
#include "config.h"
#include "CanInst.h"
#include "can.h"
#include "ErrorHandel.h"
#include "malloc.h"
#include "Chassis.h"
#include "Gimbal.h"


Motor_TypeDef *motor_list[MOTOR_NUM];
float data_view[10] ;


/*
 * @brief  	计算PID并限幅
 * @param	电机结构体指针数组
 * @retval 	无，结果在结构体中
 * @note：运行频率为1000Hz
 */
void MotorCalc(void)
{
    static float val_last=0;

    for(int i=0;i<MOTOR_NUM;i++)
    {

//        if(motor_list[i]->info.enable == 0)
//        {
//            motor_list[i]->output = 0;
//            continue;
//        }
        motor_list[i]->output = 0;
        PID_Type_e type = motor_list[i]->info.pid_type;

//        float val_now = MotorGetVal(motor_list[i], RAD);
        float val_now ;
        float *tar_now;
        tar_now = &motor_list[i]->target;
        if(type == SPEED)
        {
        	float tar_tmp = 0;
            //输出轴转速转为转子转速
        	if(motor_list[i]->info.motor_type == M3508)
        		tar_tmp = *tar_now *3591/187;
        	else
        		tar_tmp = *tar_now;

            motor_list[i]->output = PID_Increment(&(motor_list[i]->pid[SPEED]), MotorGetVal(motor_list[i], MOTORSPEED, RAD), tar_tmp);
        }
        else if(type == ANGLE)
        {
            //对Dji电机角度多圈转单圈
            if(motor_list[i]->info.motor_type != OTHER)
            {
                if(*tar_now - val_now > PI) *tar_now -= 2*PI;
                if(val_now - *tar_now > PI) *tar_now += 2*PI;
            }
            motor_list[i]->output = PID_Position(&(motor_list[i]->pid[ANGLE]), MotorGetVal(motor_list[i], MOTORANGLE, RAD), *tar_now);
        }
        else if(type == DUAL_LOOP )
        {
            //  motor_list[i]->output = PID_Increment(&(motor_list[i]->pid[SPEED]), MotorGetVal(motor_list[i],MOTORSPEED,RAD), *tar_now);
            if (motor_list[i]->value == NULL)
                val_now = MotorGetVal(motor_list[i], MOTORANGLE, RAD);
            else
                val_now = MotorGetVal(motor_list[i], IMU, RAD);
            // 对Dji电机角度多圈转单圈
            if (motor_list[i]->info.motor_type != OTHER)
            {
                while (*tar_now - val_now > PI)
                    *tar_now -= 2 * PI;
                while (val_now - *tar_now > PI)
                    *tar_now += 2 * PI;
            }
            motor_list[i]->output = PID_Position(&(motor_list[i]->pid[ANGLE]), val_now, *tar_now);
            if(i == MOTOR_PITCH)
            {
                motor_list[i]->output = motor_list[i]->output * -1;//pitch轴速度方向相反
            }
            else if(i == MOTOR_YAW)
            {
//                st->omega_z * 60 * CHASSIS_RADIUS * 187 / (3591 * WHEEL_RADIUS)
                motor_list[i]->output += chassis_cmd.omega_z_last ;//补偿小陀螺时云台的静态误差
            }
            motor_list[i]->output = PID_Increment(&(motor_list[i]->pid[SPEED]), MotorGetVal(motor_list[i], MOTORSPEED, RAD), motor_list[i]->output);
        }
        else if(type == AMMO)
        {
            if(gimbal_cmd . shooter_state == SHOOTER_NORMAL)
            {
                val_now = MotorGetVal(motor_list[i], MOTORANGLE, RAD);
                //多圈转单圈 && 编码转弧度
#ifdef AMMO_DIR_INV
                if(val_now > val_last + 0.005f)
                    *tar_now = *tar_now < 0 ? *tar_now + 2 * PI : *tar_now;
#elif
                if(val_now +  0.005f < val_last)
                    *tar_now = *tar_now> 2*PI ? *tar_now-2*PI : *tar_now;
#endif

                motor_list[i] -> output = PID_Position(&motor_list[i] -> pid[ANGLE], val_now, *tar_now);
                motor_list[i] -> output = PID_Increment(&motor_list[i] -> pid[SPEED], motor_list[i] -> data . speed,motor_list[i] -> output);
                val_last = val_now;
            }
            else if(gimbal_cmd . shooter_state == SHOOTER_REVERSE)
            {
                val_now = MotorGetVal(motor_list[i], MOTORANGLE, RAD);
                motor_list[i] -> output = PID_Position(&(motor_list[i] -> pid[ANGLE]), val_now, (val_now -PI));
                motor_list[i] -> output = PID_Increment(&(motor_list[i] -> pid[SPEED]), motor_list[i] -> data . speed,motor_list[i] -> output );
                gimbal_cmd . shooter_state = SHOOTER_NORMAL;
            }
        }


        else
        	sErrorHandel(OUT_OF_ENUM);

    //    MotorOptLmt(motor_list[i], motor_list[i]->info.output_max);

    }
    return;
}

/*
 * @brief  	更新PID类型参考值来源
 * @param	电机结构体地址
 * @param   PID类型枚举值
 * @param	参考值来源函数地址，float
 * @retval 	无
 * @note    '*source' can transmit as NULL if 'newtype' != 'IMU'
 */
//TODO:增加看门狗 && 整合、废弃
//void MotorStatusChange(Motor_TypeDef *motor, PID_Type_e newtype, float *source)
//{
//    if(newtype==motor->info.pid_type || source==NULL)
//        return;
//
//    PID_Clear(&motor->pid[INNER]);
//    PID_Clear(&motor->pid[OUTER]);
//    motor->info.pid_type = newtype;
//
//    if(motor->info.pid_type == ANGLE)
//        motor->value = &(motor->target);  //avoid null ptr access
//    else if(motor->info.pid_type == SPEED)
//        motor->value = &(motor->target);
//    else if(motor->info.pid_type == DUAL_LOOP)
//        motor->value = &(motor->target);
//    else if(motor->info.pid_type == DUAL_LOOP_IMU)
//        motor->value = source;
//    else
//        sErrorHandel(OUT_OF_ENUM);
//    return;
//}

/*
 * @brief  	获取电机当前值
 * @param	电机结构体地址
 * @param   style=ORIGIN, 读取原始数据；style=RAD, 弧度制
 * @retval 	电机当前值/是否过度封装/
 */

float MotorGetVal(Motor_TypeDef *motor,MOTOR_Feedback_e feedback,DataType_e type)
{
    float val ;
    if(feedback == MOTORANGLE)
        if(type == ORIGIN)
            val = motor->data.angle;
        else
            val = (((float)motor->data.angle/8192.0f)*2*PI);
    else if(feedback == MOTORSPEED)
        if(type == ORIGIN)
            val = motor->data.speed;
        else
            val = motor->data.speed*2*PI/60.0f;//RPM->RPS
    else
        if(motor == motor_list[MOTOR_YAW] ||motor == motor_list[MOTOR_PITCH] )
            val = motor->value(motor);
        else
            sErrorHandel(OUT_OF_ENUM);
    return val;
}

float MotorGetIMU0_Yaw(Motor_TypeDef *motor)
{
    return imu_list[0]->yaw(imu_list[0]);
}
float MotorGetIMU0_Pitch(Motor_TypeDef *motor)
{
    return imu_list[0]->pitch(imu_list[0]);
}

/*
 * @brief  	电机输出限幅
 * @param	电机结构体地址
 * @param   限幅值
 * @retval 	无
 */
void MotorOptLmt(Motor_TypeDef *motor, float lmt)
{
    float limit=0;
    limit = motor->info.output_max>lmt ? lmt : motor->info.output_max;
    if(motor->output >= limit)
        motor->output = limit;
    else if(motor->output <= -limit)
        motor->output = -limit;
}

void MotorCanCbk(Can_InfoTypedef *st, uint8_t *rxdata, uint8_t list_id)
{
	motor_list[list_id]->data.angle = (uint16_t)(rxdata[0]<<8) + rxdata[1];
	motor_list[list_id]->data.speed = (int16_t)(rxdata[2]<<8) + rxdata[3];
	motor_list[list_id]->data.current = (int16_t)(rxdata[4]<<8) + rxdata[5];
	motor_list[list_id]->data.temp = rxdata[6];
}

void MotorFillMsg(Can_InfoTypedef *st, uint8_t *txdata, uint8_t list_id)
{
    uint8_t tmp_id = st->CAN_id>4 ? (st->CAN_id)-4 : st->CAN_id;
    txdata[2 * (tmp_id-1)] = (int16_t)motor_list[list_id]->output >>8;
    txdata[(2*tmp_id) - 1] = (int16_t)motor_list[list_id]->output;
}

/*
 * @brief  	设置电机目标值
 * @param	电机结构体指针
 * @param	目标值
 * @param   ABS->absolute target; 
 *          INCR->add from perious target
 *          FEED->change feedforward
 * @retval 	无
 */
void MotorSetTar(Motor_TypeDef *motor, float val, ValSet_Type_e type)
{
    if(type == ABS)
        motor->target = val;
    else if(type == INCR)
        motor->target += val;
    else
        sErrorHandel(OUT_OF_ENUM);
}

/*
 * @brief  	停止指定或全部的电机输出
 * @param	电机结构体指针
 * @retval 	无
 * @note    传入空指针时，视为停止所有电机
 */
void MotorStop(Motor_TypeDef *motor)
{
    if(motor==NULL)
    {
        for(int i=0;i<MOTOR_NUM;i++)
        {
            motor_list[i]->info.enable = 0;
            motor_list[i]->output = 0;
            PID_Clear(&motor_list[i]->pid[INNER]);
            PID_Clear(&motor_list[i]->pid[OUTER]);
        }
    }
    else
    {
        motor->info.enable = 0;
        motor->output = 0;
        PID_Clear(&motor->pid[INNER]);
        PID_Clear(&motor->pid[OUTER]);
    }
}

void MotorAllEnabled(void)
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        motor_list[i]->info.enable = 1;
    }
}


/*
 * @brief  	将电机注册至列表
 * @param	电机初始化结构体
 * @retval 	无
 */
//TODO: 添加对其他电机的支持
CAN_TxHeaderTypeDef txheader = {
    .ExtId =   0x0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,

};
void MotorRegist(Motor_InitTypedef st)
{
    txheader.StdId = st.txheader_id;
    static Motor_TypeDef  motor;
    motor.can.txheader = txheader;
    motor.can.rxheader.StdId = st.rxheader_id;
    motor.can.rxheader.DLC = 8;
    motor.can.CAN_id = st.can_id;
    motor.can.FillTxMsg = MotorFillMsg;
    motor.can.CbkHandel = MotorCanCbk;
    motor.can.hcan_x = st.hcan;

    Motor_InfoTypeDef info = {
        .enable = 0,
        .motor_type = st.motor_type,
        .output_max = st.output_max,
        .pid_type = st.pid_type,
        .zero_point = 0
    };
    motor.info = info;

    motor.pid[OUTER] = st.pid_outer;
    motor.pid[INNER] = st.pid_inner;
    motor.feed_fwd = 0;
//    if(st.pid_type == DUAL_LOOP)
        motor.value = st.source;    //avoid null pointer
    motor_list[st.list_id] = malloc(sizeof(Motor_TypeDef));
    memcpy(motor_list[st.list_id], &motor, sizeof(Motor_TypeDef));
}

void MotorInit()
{

    PID_TypeDef pid_outer = {
    .Kp = 45,
    .Ki = 0,
    .Kd = 0,
    .Kf = 0,
    .Output_Max = 10,
    .Err_sum_Max = 500,
	.dead_band = 0.0f
    };

    PID_TypeDef pid_inner = {
    pid_inner.Kp = 2750,
    pid_inner.Ki = 12,
    pid_inner.Kd = 12,
    .Kf = 0,
    .Output_Max = 15000,
	.dead_band = 0.0f
    };

    Motor_InitTypedef init = {
    .can_id = 1,
    .hcan = &hcan1,
    .list_id = MOTOR_YAW,
    .motor_type = GM6020,
    .pid_type = DUAL_LOOP,
    .source = MotorGetIMU0_Yaw,
    .txheader_id = 0x1FF,
	.rxheader_id = 0x205,
    .pid_outer = pid_outer,
    .pid_inner = pid_inner,
    .output_max = 15000
};

    /* Yaw init */
    MotorRegist(init);   

    /* Pitch init */
    init.can_id = 1;
    init.hcan = &hcan2;
    init.list_id = MOTOR_PITCH;
    init.pid_type = DUAL_LOOP;
    init.source = MotorGetIMU0_Pitch;
    pid_outer.Kp = 8;
    pid_outer.Ki = 0;
    pid_outer.Kd = 0;
    pid_outer.Kf = 0;
    pid_outer.Output_Max = 10;
	pid_outer.dead_band = 0.0f;
    init.pid_outer = pid_outer;

    pid_inner.Kp = 3500;
    pid_inner.Ki = 100;
    pid_inner.Kd = 0;
    pid_inner.Output_Max = 20000;
	pid_inner.dead_band = 0.0f;
    init.pid_inner = pid_inner;
    MotorRegist(init);

    /* Chassis init */
    pid_inner.Kp = 1;
    pid_inner.Ki = 0.1f;
    pid_inner.Kd = 0;
    pid_inner.Err_sum_Max = 1000;
    pid_inner.Output_Max = 5000;
	pid_inner.dead_band = 0.0f;
    init.pid_inner = pid_inner;
    init.can_id = 1;
    init.hcan = &hcan1;
    init.list_id = MOTOR_CHS_1;
    init.motor_type = M3508,
    init.pid_type = SPEED;
    init.output_max = 5000;
    init.txheader_id = 0x200;
//    .rxheader_id = 0x200 + init.can_id,
    init.rxheader_id = 0x201;
    MotorRegist(init);

    init.can_id = 2;
    init.list_id = MOTOR_CHS_2;
//    .rxheader_id = 0x200 + init.can_id,
    init.rxheader_id = 0x202;
    MotorRegist(init);

    init.can_id = 3;
    init.list_id = MOTOR_CHS_3;
//    .rxheader_id = 0x200 + init.can_id,
    init.rxheader_id = 0x203;
    MotorRegist(init);

    init.can_id = 4;
    init.list_id = MOTOR_CHS_4;
//    .rxheader_id = 0x200 + init.can_id,
    init.rxheader_id = 0x204;
    MotorRegist(init);

    //shooter init
    pid_inner.Kp = 0;
    pid_inner.Ki = 0;
    pid_inner.Kd = 0;
    init.pid_inner = pid_inner;
    init.hcan = &hcan2;
    init.can_id = 1;
    init.list_id = MOTOR_SHT_R;
    init.txheader_id = 0x200;
//    .rxheader_id = 0x200 + init.can_id,
    init.rxheader_id = 0x201;
    MotorRegist(init);

    init.can_id = 2;
    init.list_id = MOTOR_SHT_L;
//    .rxheader_id = 0x200 + init.can_id,
    init.rxheader_id = 0x202;
    MotorRegist(init);

    //ammofeeder init
    pid_outer.Kp = 0;
    pid_outer.Ki = 0;
    pid_outer.Kd = 0;
    pid_outer.Output_Max = 15000;
    pid_outer.Err_sum_Max = 1000;
	pid_outer.dead_band = 0.0f;
    pid_inner.Kp = 0;
    pid_inner.Ki = 0;
    pid_inner.Kd = 0;
    pid_inner.Output_Max = 8000;
	pid_inner.dead_band = 0.0f;
    init.pid_inner = pid_inner;
    init.pid_outer = pid_outer;
    init.can_id = 7;
    init.motor_type = M2006;
    init.pid_type = AMMO;
    init.list_id = MOTOR_AMMO;
    init.txheader_id = 0x1FF;
//    .rxheader_id = 0x200 + init.can_id,
    init.rxheader_id = 0x207;
    MotorRegist(init);

    pid_outer.Kp = 0;
    pid_outer.Ki = 0;
    pid_outer.Kd = 0;
    pid_outer.Kf = 0;
    pid_outer.Output_Max = 10;//量程：PI
    pid_outer.Err_sum_Max = 100;
	pid_outer.dead_band = 0.0f;

    pid_inner.Kp = 0;
    pid_inner.Ki = 0;
    pid_inner.Kd = 0;
    pid_inner.Kf = 0;
    pid_inner.Output_Max = 4*PI;//量程：RPS
    pid_inner.Err_sum_Max = 100;
	pid_inner.dead_band = 0.0f;

    chassis.pid[INNER] = pid_inner;
    chassis.pid[OUTER] = pid_outer;



}

