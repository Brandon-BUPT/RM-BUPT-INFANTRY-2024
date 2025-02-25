/**
* author: Hu Zijian
*	done : 2024.3.26 
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"

#include "referee.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static can_send_encode_data_s      yaw_encode_data;
static can_send_data_s   yaw_data;
static can_send_data_keyboard_s keyboard_data;
static can_send_data_channel_s channel_data;
static can_send_data_nuc_yaw_s nuc_yaw_data;
static decode_data_superc_s superc_data;

static can_send_data_trigger_s trigger_data;
static CAN_TxHeaderTypeDef  can1_tx_trigger_message;
		


float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, 
float b_min)
{
 float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) 
+ b_min;
 return b;
}


		
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }
				case 0x003:
				{
					typedef union
					{
						can_send_encode_data_s data_s;
						uint8_t data_uint8_0[sizeof(can_send_encode_data_s)];
					}data_u_0;
					data_u_0 data_0;
					for(int i = 0;i < sizeof(can_send_encode_data_s);i++)
					{
						data_0.data_uint8_0[i] = rx_data[i];
					}
					yaw_encode_data = data_0.data_s;
					channel_data.channel_0=yaw_encode_data.channel_0;
					channel_data.channel_2=yaw_encode_data.channel_2;
					channel_data.channel_3=yaw_encode_data.channel_3;
					channel_data.mode = yaw_encode_data.mode;
					
					channel_data.W = ((yaw_encode_data.keyboard & 128) == 128);
					channel_data.A = ((yaw_encode_data.keyboard & 64) == 64);
					channel_data.S = ((yaw_encode_data.keyboard & 32) == 32);
					channel_data.D = ((yaw_encode_data.keyboard & 16) == 16);
					channel_data.fric_on = ((yaw_encode_data.keyboard & 8) == 8);
					channel_data.servo_state = ((yaw_encode_data.keyboard & 4) == 4);
					channel_data.robot_auto = ((yaw_encode_data.keyboard & 2)==  2);
					channel_data.spin = ((yaw_encode_data.keyboard & 1)==  1);
					break;
				}
				case 0x002:
				{
					typedef union
					{
						can_send_data_s data_s1;
						uint8_t data_uint8_1[sizeof(can_send_data_s)];
					}data_u_1;
					data_u_1 data_1;
					for(int i = 0;i < sizeof(can_send_data_s);i++)
					{
						data_1.data_uint8_1[i] = rx_data[i];
					}
					yaw_data.yaw=data_1.data_s1.yaw;
					yaw_data.nuc_yaw=data_1.data_s1.nuc_yaw;
				//usart_printf("1\r\n");
					break;
				}
				case 0x30:
				{
					typedef union
					{
					can_send_data_superc_s data_s_4;
					uint8_t data_4[sizeof(can_send_data_superc_s)];
					}can_send_data_superc_u;
					can_send_data_superc_u can_send_data_006_u;
					can_send_data_006_u.data_s_4.U = rx_data[0]<<8|rx_data[1];
					can_send_data_006_u.data_s_4.I = rx_data[2]<<8|rx_data[3];
					can_send_data_006_u.data_s_4.state = rx_data[4]<<8|rx_data[5];
					superc_data.U =  int16_to_float(can_send_data_006_u.data_s_4.U,32000,-32000,30.0,0.0);
					superc_data.I = int16_to_float(can_send_data_006_u.data_s_4.I,32000,-32000,20.0,-20.0);
					superc_data.error = can_send_data_006_u.data_s_4.state;

				}
        
        default:
        {
            break;
        }
    }
}

const  can_send_data_s *get_yaw_measure_point()
{
	return &yaw_data;
}

const  can_send_data_channel_s *get_channel_measure_point()
{
	return &channel_data;
}
const  decode_data_superc_s *get_superc_measure_point()
{
	return &superc_data;
}

const can_send_data_trigger_s *get_trigger_measure_point()
{
  return &trigger_data;
}



//给超级电容发送剩余buffer
static uint8_t can2_send_super_c_buffer[8];
CAN_TxHeaderTypeDef   can2_tx_message_super_c_buffer;
void CAN2_send_super_c_buffer(int16_t buffer)
{
	  uint32_t send_mail_box;
	  can2_tx_message_super_c_buffer.StdId=0x2E;
		can2_tx_message_super_c_buffer.IDE=CAN_ID_STD;
	  can2_tx_message_super_c_buffer.RTR=CAN_RTR_DATA;
		can2_tx_message_super_c_buffer.DLC=0x08;
		can2_send_super_c_buffer[0]=buffer>>8;
		can2_send_super_c_buffer[1]=buffer;	
	  HAL_CAN_AddTxMessage(&hcan1,&can2_tx_message_super_c_buffer,can2_send_super_c_buffer,&send_mail_box);
}
//给云台转发裁判数据
static uint8_t can1_send_referee[8];
CAN_TxHeaderTypeDef can1_tx_message_referee;
typedef struct {
	int16_t shootspeed;
	uint8_t robot_id;
}can_send_referee_s;
void CAN1_send_referee(int16_t shootspeed,uint8_t robot_color){
	uint32_t send_mail_box;
	typedef union{
		can_send_referee_s data_s_refree;
		uint8_t data[sizeof(can_send_data_keyboard_s)];
	}can_send_referee_u;
	can_send_referee_u data_u;
	data_u.data_s_refree.shootspeed=get_shoot_speed();
	data_u.data_s_refree.robot_id=get_robot_id();
	for(int i=0;i<sizeof(can_send_referee_u);i++){
	can1_send_referee[i]=data_u.data[i];
	}
	can1_tx_message_referee.StdId=0x004;
	can1_tx_message_referee.IDE=CAN_ID_STD;
	can1_tx_message_referee.RTR=CAN_RTR_DATA;
	can1_tx_message_referee.DLC=0x08;
	HAL_CAN_AddTxMessage(&hcan1,&can1_tx_message_referee,can1_send_referee,&send_mail_box);
}

//给超级电容发送控制包
uint8_t can2_send_super_c_ctrl[8];
CAN_TxHeaderTypeDef	can2_tx_message_super_c_ctrl;
void CAN2_send_super_c_control(int16_t a,int16_t b,int16_t c){
	uint32_t send_mail_box;
	can2_tx_message_super_c_ctrl.StdId=0x2F;
	can2_tx_message_super_c_ctrl.IDE=CAN_ID_STD;
	can2_tx_message_super_c_ctrl.RTR=CAN_RTR_DATA;
	can2_tx_message_super_c_ctrl.DLC=0x08;
	can2_send_super_c_ctrl[0]=a>>8;
	can2_send_super_c_ctrl[1]=a;
	can2_send_super_c_ctrl[2]=b>>8;
	can2_send_super_c_ctrl[3]=b;
	can2_send_super_c_ctrl[4]=c>>8;
	can2_send_super_c_ctrl[5]=c;
	can2_send_super_c_ctrl[6]=0x3F;
	can2_send_super_c_ctrl[7]=0x3f;
	HAL_CAN_AddTxMessage(&hcan1,&can2_tx_message_super_c_ctrl,can2_send_super_c_ctrl,&send_mail_box);
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
