/**
 ******************************************************************************
 * @file    communication_task.c
 * @author  Hu Zijian
 * @version V2.0.0
 * @date    2024/3/10
 * @brief
 ******************************************************************************
 * @attention
 *  用来同上位机和底盘c版通信
 ******************************************************************************
 */
 
#include "communication_task.h"
#include "main.h"
#include "CAN_receive.h"
#include "nucCommu.h"
#include "INS_task.h"
#include "usart.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "robot_total_mode.h"


//******************全局变量*******************
static uint8_t GimbalAngleMsg[38];


void communication_task(void const *pvParameters){
		
    while (1)
    {
        Encode(GimbalAngleMsg,(double)get_INS()->Yaw,(double)get_INS()->Pitch,(double)get_INS()->Roll,get_refree_point()->robot_color,3,1,robotIsAuto());
				HAL_UART_Transmit(&huart1, GimbalAngleMsg, 38, 100);
				usart1_tx_dma_enable(GimbalAngleMsg, 38);
				osDelay(4);
				CAN1_send_yaw();
				CAN1_send_channel();    
				osDelay(2);
		}
}
