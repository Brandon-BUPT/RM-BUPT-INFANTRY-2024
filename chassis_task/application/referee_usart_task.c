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

#include "CAN_receive.h"


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

static void referee_ui(int count);

static void referee_ui_init(void);

static void referee_ui_refresh(void);

static void referee_ui_transmit(int count);
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
static const can_send_data_channel_s* RC_channel;
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
		RC_channel = get_channel_measure_point();
		int count = 0;
    while(1)
    {
			if(count%10==0)
        referee_unpack_fifo_data();
			if(count%100 == 0)	
				referee_ui(count);
			
			
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
static void referee_ui(int count)
{
	referee_ui_init();
	referee_ui_refresh();
	referee_ui_transmit(count);
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
ui_string_frame_t layer2;//字符串
ui_2_frame_t layer3;//动态圆形
ui_2_frame_t layer4;//动态圆形加条
ui_7_frame_t layer5;//射击线2
ui_1_frame_t layer6;//电容框
//增加静态图层
void ui_draw(ui_interface_figure_t *line,const char figure_name[3], int operate_tpyel, int figure_tpye,int layer, int color,
                  int start_x, int start_y, int width, int end_x, int end_y) 
{
	line->figure_name[0] = figure_name[0];
	line->figure_name[1] = figure_name[1];
	line->figure_name[2] = figure_name[2];
	line->operate_tpyel = operate_tpyel;
	line->figure_tpye = figure_tpye ;
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

void ui_draw_round(ui_interface_figure_t *line,const char figure_name[3], int operate_tpyel, int figure_tpye,int layer, int color,
                  int start_x, int start_y, int width, int radis)
{
	line->figure_name[0] = figure_name[0];
	line->figure_name[1] = figure_name[1];
	line->figure_name[2] = figure_name[2];
	line->operate_tpyel = operate_tpyel;
	line->figure_tpye = figure_tpye ;
	line->layer = layer;
	line->color = color;
	line->start_x = start_x;
	line->start_y = start_y;
	line->width = width;
	line->_c = radis;
}

void ui_draw_string(ui_interface_string_t *str,const char figure_name[3], int operate_tpyel, int figure_tpye,int layer, int color,
                  int start_x, int start_y,int width,int font,int length,const char *string)
{
	str->figure_name[0] = figure_name[0];
	str->figure_name[1] = figure_name[1];
	str->figure_name[2] = figure_name[2];
	str->operate_tpyel = operate_tpyel;
	str->figure_tpye = figure_tpye ;
	str->layer = layer;
	str->color = color;
	str->start_x = start_x;
	str->start_y = start_y;
	str->width = width;
	str->font_size = font;
	str->str_length = 30;
	for(int i=0;i<strlen(string);i++)
		str->string[i]=string[i];
}


static void referee_ui_init(){
	//layer0
	layer0.data[0].figure_name[0] = 'a';
	layer0.data[0].figure_name[1] = 'a';
	layer0.data[0].figure_name[2] = '\0';
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
	ui_draw(&layer0.data[1],"bb",1,0,0,2,1920/2,119,1,1920/2,539);
	ui_draw(&layer0.data[2],"cc",1,1,0,3,737,344,3,737+454,344+454);
	ui_draw(&layer0.data[3],"dd",1,0,0,2,743,480,1,1181,480);
	ui_draw(&layer0.data[4],"ee",1,0,0,2,882,441,1,1033,441);
	
	
	//layer1
	ui_draw(&layer1.data[0],"ff",1,0,1,8,551,207,1,652,632);
	ui_draw(&layer1.data[1],"gg",1,0,1,8,1268,610,1,1369,207);
	
  //layer2
	ui_draw_string(&layer2.option,"en",1,7,2,2,115,730,5,40,30,"AIM :\nSPIN:\nFRIC:\n");
	
	//layer3
	ui_draw_round(&layer3.data[0],"spi",1,2,0,2,316,644,1,18);
	ui_draw_round(&layer3.data[1],"frc",1,2,0,2,316,575,1,18);
	
	//layer4
	ui_draw_round(&layer4.data[0],"aut",1,2,0,2,316,719,1,18);
	ui_draw(&layer4.data[1],"cap",1,0,1,2,660,80,20,1260,80);
	
	//layer5
	ui_draw(&layer5.data[0],"hh",1,0,0,2,814,423,1,1103,423);
	ui_draw(&layer5.data[1],"ii",1,0,0,2,877,411,1,1037,411);
	ui_draw(&layer5.data[2],"jj",1,0,0,2,933,382,1,983,382);
	ui_draw(&layer5.data[3],"kk",1,0,0,2,933,395,1,981,395);
	ui_draw(&layer5.data[4],"ll",1,0,0,2,935,369,1,979,369);
	ui_draw(&layer5.data[5],"mm",1,0,0,2,934,356,1,983,356);
	ui_draw(&layer5.data[6],"nn",1,0,0,2,935,342,1,985,342);
}
//刷新动态图层
static void referee_ui_refresh()
{
	if(RC_channel->fric_on)
		ui_draw_round(&layer3.data[1],"frc",1,2,0,3,316,575,1,18);
	else
		ui_draw_round(&layer3.data[1],"frc",1,2,0,2,316,575,1,18);
	if(RC_channel->servo_state)
		ui_draw(&layer4.data[1],"cap",1,0,1,3,660,80,20,1260,80);
	else
		ui_draw(&layer4.data[1],"cap",1,0,1,2,660,80,20,1260,80);
	if(RC_channel->robot_auto)
		ui_draw_round(&layer4.data[0],"aut",1,2,0,3,316,719,1,18);
	else
		ui_draw_round(&layer4.data[0],"aut",1,2,0,2,316,719,1,18);
	if(RC_channel->spin)
		ui_draw_round(&layer3.data[0],"spi",1,2,0,3,316,644,1,18);
	else
		ui_draw_round(&layer3.data[0],"spi",1,2,0,2,316,644,1,18);
}
//填充打包发送
uint8_t buffer[400];
static void referee_ui_transmit(int count){
	if((count/100)%10==0)
	{
		ui_proc_5_frame(&layer0);
		memcpy(buffer, &layer0, sizeof(ui_5_frame_t));
		HAL_UART_Transmit(&huart6,buffer,sizeof(ui_5_frame_t),100);
		usart6_tx_dma_enable(buffer,sizeof(ui_5_frame_t));
		osDelay(2);
	}
		if((count/100)%10==1)
	{
		ui_proc_2_frame(&layer1);
		memcpy(buffer, &layer1, sizeof(ui_2_frame_t));
		HAL_UART_Transmit(&huart6,buffer,sizeof(ui_2_frame_t),100);
		usart6_tx_dma_enable(buffer,sizeof(ui_2_frame_t));
		osDelay(2);
	}
		if((count/100)%10==2)
	{
		ui_proc_7_frame(&layer5);
		memcpy(buffer, &layer5, sizeof(ui_7_frame_t));
		HAL_UART_Transmit(&huart6,buffer,sizeof(ui_7_frame_t),200);
		usart6_tx_dma_enable(buffer,sizeof(ui_7_frame_t));
		osDelay(2);
	}
		if((count/100)%10==3)
	{
		ui_proc_string_frame(&layer2);
		memcpy(buffer, &layer2, sizeof(layer2));
		HAL_UART_Transmit(&huart6,buffer,sizeof(layer2),200);
		usart6_tx_dma_enable(buffer,sizeof(layer2));
		osDelay(2);
	}
		if((count/100)%3==0)
	{
		ui_proc_2_frame(&layer3);
		memcpy(buffer, &layer3, sizeof(ui_2_frame_t));
		HAL_UART_Transmit(&huart6,buffer,sizeof(ui_2_frame_t),200);
		usart6_tx_dma_enable(buffer,sizeof(ui_2_frame_t));
		osDelay(2);
	}

			if((count/100)%3==1)
	{
		ui_proc_2_frame(&layer4);
		memcpy(buffer, &layer4, sizeof(ui_2_frame_t));
		HAL_UART_Transmit(&huart6,buffer,sizeof(ui_2_frame_t),200);
		usart6_tx_dma_enable(buffer,sizeof(ui_2_frame_t));
		osDelay(2);
	}

//	ui_proc_5_frame(&layer2);
//	ui_proc_2_frame(&layer3);
//	ui_proc_2_frame(&layer4);
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


