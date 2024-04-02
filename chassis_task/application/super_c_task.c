#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "CAN_receive.h"
#include "super_c_task.h"

#include "referee.h"
#include <math.h>
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, 
int16_t b_min)
{
int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + 
(float)b_min + 0.5f;
//加 0.5 使向下取整变成四舍五入
 return b;
}
cap_measure_t cap_measure;
static void super_c_init(){
	cap_measure.cap_percent = 0;
}
static void super_c_update(){
	cap_measure.cap_percent = get_superc_measure_point()->U*get_superc_measure_point()->U/24.1/24.1;
}
void super_c_task(void const * arguement)
{
	
	super_c_init();
	fp32 power, buffer;
	while(1)
	{		
		super_c_update();
		get_chassis_power_and_buffer(&power, &buffer);


		osDelay(2);
		CAN2_send_super_c_buffer((int16_t)buffer);

//		osDelay(2);
//		CAN2_send_super_c_control(get_robot_chassis_power_limit()-15,150,150);
//		usart_printf("%d,%lf,%d,%f\r\n",t,8000*sin((double)t/500),get_chassis_motor_measure_point(1)->speed_rpm,power);
		
//				int t = xTaskGetTickCount();
//		CAN_cmd_chassis(0,8000*sin((double)t/500),0,0);
//		usart_printf("%f,%f,%d,%f\r\n",get_superc_measure_point()->U,get_superc_measure_point()->I,get_robot_chassis_power_limit(),buffer);
//		

	}
}