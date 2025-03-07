//接口头文件是ui.h
//为了使消息能够发送给裁判系统，必须要自定义ui_interface.c中的ui_self_id变量（该变量也可以使用其他代码从裁判系统读取后由程序修改）
//由于裁判系统接收限制，可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数在ui_interface.h中替换掉SEND_MESSAGE的宏定义HAL_UART_Transmit_DMA
#include "ui.h"
#include "math.h"
#include "main.h"
#include "Control.h"
#include "Gimbal.h"
#include "SuperCap.h"
#include "remote.h"
#include "config.h"
#include "Chassis.h"

osThreadId_t ui_task_handel;
uint8_t ui_diff = 0;
uint8_t fric_flag;


uint8_t supercap_energy = 0;
float delta_gim;

void UI_Init(void) //创建所有元素的函数，调用一次即可，如果只能显示部分图案，就尝试重新创建
{
	osThreadAttr_t task_attr = {
			.name = "UITask",
			.stack_size = 128 * 4,
			.priority = (osPriority_t) osPriorityNormal ,
	};

	ui_task_handel = osThreadNew (UITask, NULL, &task_attr);
}


void ui_reinit(void) //重新创建所有元素的函数，由于下位机上电时主控不一定连接到服务器，建议设置一个遥控按键，在按下按键后重新创建，避免由于丢包等各种因素ui没有显示，如果还是只能显示部分图案，可尝试在每行后面适当加上延时
{
	ui_remove_shooter1_group1();
	osDelay(100);
	ui_remove_shooter1_group2();
	osDelay(100);
	ui_remove_shooter1_group3();
	osDelay(100);
	ui_remove_shooter1_group5();
	osDelay(100);

	ui_init_shooter1_group1();
	osDelay(100);
	ui_init_shooter1_group2();
	osDelay(100);
	ui_init_shooter1_group3();
	osDelay(100);
	ui_init_shooter1_group5();
	osDelay(100);
}


void gimbal_direction(float delta) //更新云台和底盘相对位置的函数，delta为（云台角度-底盘角度），正中间为0度，顺时针为正，弧度制
{
	ui_shooter1_group3_Front->end_x = ui_shooter1_group3_Front->start_x + (int)(85 * sin(delta));
	ui_shooter1_group3_Front->end_y = ui_shooter1_group3_Front->start_y + (int)(85 * cos(delta));

	_ui_update_shooter1_group3_0();
}


void capasitor_level(uint8_t level) //更新超电剩余电量的函数，level为剩余百分比
{
	ui_shooter1_group5_electricity->end_y = ui_shooter1_group5_electricity->start_y + 4 * level;

	_ui_update_shooter1_group5_0();
}


void auto_shift(uint8_t status) //更新自瞄状态的函数，需要自行根据自瞄状态赋值给status，status = 2为自瞄中，1为自瞄已准备好，否则为0
{
	if(status == 2)
	{
		ui_shooter1_group2_auto_point->color = 0;
		_ui_update_shooter1_group2_0();
	}
	else if(status == 1)
	{
		ui_shooter1_group2_auto_point->color = 2;
		_ui_update_shooter1_group2_0();	
	}
	else
	{
		ui_shooter1_group2_auto_point->color = 8;
		_ui_update_shooter1_group2_0();

	}
}


void fric_shift(uint8_t status) //更新摩擦轮状态的函数，需要自行根据摩擦轮状态赋值给status，开启为1，否则为0
{
	if(status == 1)
	{
		ui_shooter1_group2_fric_point->color = 2;
		_ui_update_shooter1_group2_0();
	}
	else
	{
		ui_shooter1_group2_fric_point->color = 8;
		_ui_update_shooter1_group2_0();

	}
}



void UITask (void *argument)
{

	uint32_t time = osKernelGetTickCount ();

	ui_init_shooter1_group1();
	osDelay(100);
	ui_init_shooter1_group2();
	osDelay(100);
	ui_init_shooter1_group3();
	osDelay(100);
	ui_init_shooter1_group5();
	osDelay(100);
	for ( ;; )
	{
		if(gimbal_cmd.friction_state)
		{
			if(!fric_flag)
			{
				fric_shift(1);//摩擦轮开启标志位
				fric_flag=1;
			}
		}
		else
		{
			if(fric_flag)
			{
				fric_shift(0);
				fric_flag=0;
			}
		}

//		if(gimbal_cmd.shooter == AUTO)
//			auto_shift(2);//云台跟随+火控开启标志位
//		else if(gimbal_cmd.shooter == SHOOT)//云台跟随
//			auto_shift(1);
//		else
//			auto_shift(0);
//
		delta_gim = -chassis.error/8192 *2*PI;
		gimbal_direction(delta_gim);
		supercap_energy = (uint8_t)(supercap_rx_data.capEnergy * 100/255);
		capasitor_level(supercap_energy);//超电容量百分比
//		capasitor_level(50);//超电容量百分比
		ui_diff = osKernelGetTickCount () - time;
		time = osKernelGetTickCount ();

		osDelayUntil (time + UI_TASK_PERIOD);
	}
}

