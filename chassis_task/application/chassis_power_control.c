/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       在官方功率控制基础上，添加西交利物浦的功率控制，PID是得调的
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
	*  V2.0.0     Apr-2	-2024			胡子健						2. add SUPERC power	control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "super_c_task.h"

//以下三个宏定义只能开一个
#define CHASSIS_POWER_CONTROL_OFFICIAL //打开官代底盘功率控制函数
//#define CHASSIS_POWER_CONTROL_NO_POWER_BUFF //不使用超级电容 底盘功率控制函数
//#define  CHASSIS_POWER_CONTROL_POWER_BUFF //使用超级电容 底盘功率控制函数





#define POWER_LIMIT         120.0f
uint16_t WARNING_POWER = 50.0f; 
uint16_t LEAST_BUFFER = 5.0f;
uint16_t WARNING_POWER_BUFF=50.0f;

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f    //4000 * 4
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
	
#ifdef CHASSIS_POWER_CONTROL_OFFICIAL
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = get_robot_id();
    uint8_t robot_level = get_robot_level();
    WARNING_POWER = 50;
    if(robot_level == 1)
    {
        WARNING_POWER = 55;
    }
    else if(robot_level == 2)
    {
        WARNING_POWER = 60;
    }
    else if(robot_level == 3)
    {
        WARNING_POWER = 65;
    }
    
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }    
		else
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
        //功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
			
				//当buffer小于warningpower
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
						//大于5.0f的时候，逐渐增大限制
            if(chassis_power_buffer > LEAST_BUFFER)
            {
                //scale down WARNING_POWER_BUFF
                //缩小WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                //only left 10% of WARNING_POWER_BUFF
							power_scale = 0.0f;
//                power_scale = 0.0fLEAST_BUFFER / WARNING_POWER_BUFF;
            }
            //scale down
            //缩小
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //power > WARNING_POWER
            //功率大于WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {
                fp32 power_scale;
                //power < 80w
                //功率小于80w
                if(chassis_power < POWER_LIMIT)
                {
                    //scale down
                    //缩小
                      power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                //power > 80w
                //功率大于80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //power < WARNING_POWER
            //功率小于WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    
    total_current = 0.0f;
    //calculate the original motor current set
    //计算原本电机电流设定
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
    

    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
}
#endif
#ifdef CHASSIS_POWER_CONTROL_NO_POWER_BUFF
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = get_robot_id();
    uint8_t robot_level = get_robot_level();
		if(get_robot_chassis_power_limit()!=0)
			    WARNING_POWER = get_robot_chassis_power_limit();
		else
			WARNING_POWER = 55;

		fp32 current_power, current_buffer, power_scale;
		get_chassis_power_and_buffer(&current_power, &current_buffer);
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
		else
		{
			if(current_power >= WARNING_POWER)
			{
				power_scale = WARNING_POWER / current_power;
				usart_printf("%f\r\n", power_scale);
			
			
			
				total_current_limit = POWER_TOTAL_CURRENT_LIMIT * power_scale;
				
				total_current = 0.0f;
				//calculate the original motor current set
				//计算原本电机电流设定
				for(uint8_t i = 0; i < 4; i++)
				{
						total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
				}
				

				if(total_current > total_current_limit)
				{
						fp32 current_scale = total_current_limit / total_current;
						chassis_power_control->motor_speed_pid[0].out*=current_scale;
						chassis_power_control->motor_speed_pid[1].out*=current_scale;
						chassis_power_control->motor_speed_pid[2].out*=current_scale;
						chassis_power_control->motor_speed_pid[3].out*=current_scale;
						
				}
			}
		}
}
#endif
#ifdef CHASSIS_POWER_CONTROL_POWER_BUFF
extern cap_measure_t cap_measure; //超级电容数据指针
extern RC_ctrl_t rc_ctrl;			//远程控制数据指针
uint8_t cap_state = 0;
void chassis_power_control(chassis_move_t *chassis_power_control)
{

	uint16_t max_power_limit = 40;
	fp32 chassis_max_power = 0;
	float input_power = 0;		 // input power from battery (referee system)
	float initial_give_power[4]; // initial power from PID calculation
	float initial_total_power = 0;
	fp32 scaled_give_power[4];

	fp32 chassis_power = 0.0f;
	fp32 chassis_power_buffer = 0.0f;

//	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55 currentset * toque_coefficient = toque 4
	fp32 toque_coefficient = 2.792302849e-6;
	fp32 a = 1.23e-07;						 // k1 
	fp32 k2 = 1.453e-07;					 // k2 
	fp32 constant = 4.081f;

	get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
	PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 50);
	max_power_limit = get_robot_chassis_power_limit();
	input_power = max_power_limit - chassis_power_control->buffer_pid.out; // Input power floating at maximum power

	CAN2_send_super_c_control(input_power,150,150); // set the input power of capacitor controller

	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
	{
		cap_state = 0;
	}
	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
	{
		cap_state = 1;
	}

	if (cap_measure.cap_percent > 5)
	{
		if (cap_state == 0)
		{
			chassis_max_power = input_power + 5; // Slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
		}
		else
		{
			chassis_max_power = input_power + 200;
		}
	}
	else
	{
		chassis_max_power = input_power;
	}

	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
								k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;

		if (initial_give_power < 0) // negative power not included (transitory)
			continue;
		initial_total_power += initial_give_power[i];
	}

	if (initial_total_power > chassis_max_power) // determine if larger than max power
	{
		fp32 power_scale = chassis_max_power / initial_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			if (scaled_give_power[i] < 0)
			{
				continue;
			}

			fp32 b = toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
			fp32 c = k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

			if (chassis_power_control->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					chassis_power_control->motor_speed_pid[i].out = 16000;
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
			else
			{
				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					chassis_power_control->motor_speed_pid[i].out = -16000;
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
		}
	}
}

#endif

void set_warning_power(uint16_t warning_power)
{
	WARNING_POWER = warning_power;
}
void set_warning_power_buff(uint16_t warning_power_buff)
{
	WARNING_POWER_BUFF = warning_power_buff;
}
void set_least_power_buff(uint16_t least_power_buff)
{
	LEAST_BUFFER=least_power_buff;
}