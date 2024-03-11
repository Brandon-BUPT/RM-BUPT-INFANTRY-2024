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

//******************全局变量*******************
static uint8_t GimbalAngleMsg[34];
void communication_task(void const *pvParameters){
		
    while (1)
    {
        Encode(GimbalAngleMsg,(double)get_INS()->Yaw,(double)get_INS()->Pitch,(double)get_INS()->Roll,0,3,1,0);
				HAL_UART_Transmit(&huart1, GimbalAngleMsg, 34, 100);
				usart1_tx_dma_enable(GimbalAngleMsg, 34);
				HAL_Delay(4);
				CAN1_send_yaw();
				CAN1_send_channel();
				osDelay(1);    
		}
}
