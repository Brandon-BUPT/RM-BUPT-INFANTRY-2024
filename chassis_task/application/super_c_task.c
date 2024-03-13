#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "CAN_receive.h"
#include "super_c_task.h"

#include "referee.h"

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, 
int16_t b_min)
{
int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + 
(float)b_min + 0.5f;
//加 0.5 使向下取整变成四舍五入
 return b;
}

void super_c_task(void const * arguement)
{
	  fp32 power, buffer;
		get_chassis_power_and_buffer(&power, &buffer);
	while(1)
	{		
		CAN2_send_super_c_buffer(float_to_int16(buffer,0.0,60.0,-32000,32000));
		CAN2_send_super_c_control(get_robot_chassis_power_limit(),100,100);
		osDelay(10);
	}
}