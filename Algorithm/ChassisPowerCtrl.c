// #include "referee.h"
// #include "remote.h"
// #include "SuperCap.h"
// #include "math.h"
// #include "motor.h"
// #include "config.h"

// uint8_t cap_state = 0;
// float initial_give_power[4]; // initial power from PID calculation
// float initial_total_power = 0;
// float total_power_view = 0;

// void chassis_power_control(void)
// {

// 	uint16_t max_power_limit = 60;
// 	float chassis_max_power = 0;
// 	float input_power = 0;		 // input power from battery (referee system)
// 	float scaled_give_power[4];
// 	float chassis_power = 0.0f;
// 	float chassis_power_buffer = 0.0f;

// 	//经验参数
// 	float toque_coefficient = 1.99688994e-6f;    // (20/16384)*(0.3)*(187/3591)/9.55    //扭矩常数     9.55 * P = n *T;   T = 0.3 *I;  n = n1*187/3591
// 	float a = 1.23e-07;						     // k1
// 	float k2 = 1.453e-07;					     // k2
	
// 	float constant = 4.081f;					 // 常量

// //  float toque_coefficient = 0.00000199688999f; // (20/16384)*(0.3)*(187/3591)/9.55    //扭矩常数     9.55 * P = n *T;   T = 0.3 *I;  n = n1*187/3591
// //	float a = 0.000000123f;						 // k1
// //	float k2 = 0.0000001453f;					 // k2
// //	float constant = 4.081f;					 // 常量

// 	float speed_rpm = 0;

// 	chassis_max_power = max_power_limit;

// 	//电机功率模型
// 	for (uint8_t i = MOTOR_CHS_1; i <= MOTOR_CHS_4; i++) // first get all the initial motor power and total motor power
// 	{
// 		// speed_rpm = motor_list[i] -> data . speed;
// 		initial_give_power[i] = motor_list[i] -> output * toque_coefficient * speed_rpm +//电机机械功率
// 								k2 * speed_rpm * speed_rpm +
// 								a * motor_list[i] -> output * motor_list[i] -> output + constant;

// 		if (initial_give_power < 0) // negative power not included (transitory)
// 			continue;
// 		initial_total_power += initial_give_power[i];//输出总功率
// 		total_power_view = initial_total_power;
// 	}

// 	if (initial_total_power > chassis_max_power) // determine if larger than max power
// 	  {
// 	  	float power_scale = chassis_max_power / initial_total_power;
//          float k1 = a;
// 	  	for (uint8_t i = 0; i < 4; i++)
// 	  	{
// 	  		scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			
// 	  		if (scaled_give_power[i] < 0)
// 	  		{
// 	  			continue;
// 	  		}
// 	  		// 逆解出电机控制电流，解二元方程
// 	  		float b = toque_coefficient * speed_rpm;
// 	  		float c = k1 * speed_rpm * speed_rpm - scaled_give_power[i] + constant;
// 	  		float inside = b * b - 4 * a * c;

// 	  		if (inside < 0)
// 	  		{
// 	  			continue;
// 	  		}
// 	  		else if (motor_list[i] -> output > 0) // Selection of the calculation formula according to the direction of the original moment
// 	  		{
// 	  			float temp = (-b + sqrt(inside)) / (2 * a);
// 	  			if (temp > 16000)
// 	  			{
// 	  				motor_list[i] -> output = 16000;
// 	  			}
// 	  			else
// 	  				motor_list[i] -> output = temp;
// 	  		}
// 	  		else
// 	  		{
// 	  			float temp = (-b - sqrt(inside)) / (2 * a);
// 	  			if (temp < -16000)
// 	  			{
// 	  				motor_list[i] -> output = -16000;
// 	  			}
// 	  			else
// 	  				motor_list[i] -> output = temp;
// 	  		}
// 	  	}
// 	}

// 	// else if(supercap_rx_data)
// }

