/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM����ϵͳ���ݴ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_usart.h"
#include "detect_task.h"
#include <stdlib.h>

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

#include "ui_interface.h"
#include "ui_types.h"


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

static void referee_ui(void);

static void referee_ui_init(void);

static void referee_ui_refresh(void);

static void referee_ui_transmit(void);
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ����ϵͳ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

		int count = 0;
    while(1)
    {
			if(count%10==0)
        referee_unpack_fifo_data();
			if(count%100 == 0)	
				referee_ui();
			
			
			count++;
      osDelay(1);
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          ���ֽڽ��
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

//主要发送逻辑
static void referee_ui(void)
{
	referee_ui_init();
	referee_ui_transmit();
}
/*
	代码思路：
		先把需要的结构体打包好
		0 - 5
		1 - 2
		2 - 5
		3 - 2
		4 - 2
		5 - 7
	*/
ui_5_frame_t layer0;//射击线
ui_2_frame_t layer1;//车道线
ui_5_frame_t layer2;//字符串
ui_2_frame_t layer3;//动态圆形
ui_2_frame_t layer4;//动态圆形加条
ui_7_frame_t layer5;//射击线2
//增加静态图层
void ui_draw_lineandrect(ui_interface_figure_t *line,const char figure_name[3], int operate_tpyel, int layer, int color,
                  int start_x, int start_y, int width, int end_x, int end_y) 
{
	line->figure_name[0] = figure_name[0];
	line->figure_name[1] = figure_name[1];
	line->figure_name[2] = figure_name[2];
	line->operate_tpyel = operate_tpyel;
	line->figure_tpye = 0 ;
	line->layer = layer;
	line->color = color;
	line->start_x = start_x;
	line->start_y = start_y;
	line->width = width;
	line->_a = 0;
	line->_b = 0;
	line->_c = 0;
	line->_d = end_x;
	line->_e = end_y;
}


static void referee_ui_init(){
	//layer0
	layer0.data[0].figure_name[0] = 'a';
	layer0.data[0].figure_name[1] = 'a';
	layer0.data[0].figure_name[2] = 'a';
	layer0.data[0].operate_tpyel = 1;
	layer0.data[0].figure_tpye  = 0;
	layer0.data[0].layer = 0;
	layer0.data[0].color = 2;
	layer0.data[0].start_x = 672;
	layer0.data[0].start_y = 538;
	layer0.data[0].width = 1;
	layer0.data[0]._a = 0;
	layer0.data[0]._b = 0;
	layer0.data[0]._c = 0;
	layer0.data[0]._d = 1255;
	layer0.data[0]._e = 538;
	
	ui_draw_lineandrect(&layer0.data[1],"bbb",1,0,2,1920/2,119,1,1920/2,539);
	ui_draw_lineandrect(&layer0.data[2],"ccc",1,0,3,737,344,3,737+454,344+454);
	ui_draw_lineandrect(&layer0.data[3],"ddd",1,0,2,743,480,1,1181,480);
	ui_draw_lineandrect(&layer0.data[4],"eee",1,0,2,882,441,1,1033,441);
	
	//layer1
}
//刷新动态图层

//填充打包发送
char buffer[200];
static void referee_ui_transmit(){
	ui_proc_5_frame(&layer0);
	memcpy(buffer, &layer0, sizeof(ui_5_frame_t));
	usart6_tx_dma_enable((uint8_t *)buffer,sizeof(layer0));
//	ui_proc_2_frame(&layer1);
//	ui_proc_5_frame(&layer2);
//	ui_proc_2_frame(&layer3);
//	ui_proc_2_frame(&layer4);
//	ui_proc_7_frame(&layer5);
}

void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
    }
}


