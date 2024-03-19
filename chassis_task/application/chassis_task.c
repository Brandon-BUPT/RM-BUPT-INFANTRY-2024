/**
  ********************************************************
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "kalman.h"


#define OMNI_CHASSIS 
//#define MEC_CHASSIS 


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

#define range_limit_inside(toBeLimited, range)                      \
    {                                                               \
         if((toBeLimited)>(range))                                  \
        {                                                           \
            (toBeLimited)=(range);                                  \
        }                                                           \
        else if((toBeLimited)<-(range))                             \
        {                                                           \
            (toBeLimited)=-(range);                                 \
        }                                                           \
    }
		//**********************************//
#define YAW_INIT_ECD 2522 //yaw的初始ecd
//*********************************************//
#define YAW_RC_SEN      -0.000004f
#define PITCH_RC_SEN    -0.000005f //0.005
#define YAW_MOUSE_SEN   -0.00005f
#define PITCH_MOUSE_SEN -0.00005f

#define HANDLE_LEFT_LR 2    
#define HANDLE_LEFT_BF 3    
#define HANDLE_RIGHT_LR 0    
#define HANDLE_RIGHT_BF 1    /


#define COMMON_FORTH            KEY_PRESSED_OFFSET_E    //前
#define COMMON_BACK             KEY_PRESSED_OFFSET_D    //
#define COMMON_LEFT_ROTATE      KEY_PRESSED_OFFSET_S    //转
#define COMMON_RIGHT_ROTATE     KEY_PRESSED_OFFSET_F    //转
#define COMMON_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //平
#define COMMON_RIGHT_MOVE       KEY_PRESSED_OFFSET_G    //平


#define COMMON_BF_RC            HANDLE_LEFT_BF      //前,
#define COMMON_LR_RC            HANDLE_LEFT_LR      //平,平
#define COMMON_LR_ROTATE_RC     HANDLE_RIGHT_LR     //转转


#define SPINNER_FORTH            KEY_PRESSED_OFFSET_W    //前
#define SPINNER_BACK             KEY_PRESSED_OFFSET_S    //
#define SPINNER_LEFT_MOVE        KEY_PRESSED_OFFSET_A    //平
#define SPINNER_RIGHT_MOVE       KEY_PRESSED_OFFSET_D    //平

#define SPINNER_BF_RC            HANDLE_LEFT_BF      //前,
#define SPINNER_LR_MOVE_RC       HANDLE_LEFT_LR      //转,转
//****************************锟剿科筹拷***************//
#define KEYBOARD_CONTROL_ROBOT_SPEED_X 2.8f      //wasd锟狡蒂讹拷
#define KEYBOARD_CONTROL_ROBOT_SPEED_Y 2.8f
#define KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_X 4.5f
#define KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_Y 4.5f
#define KEYBOARD_CONTROL_ROBOT_SPEED_W 1.0f
#define SPINNER_W   6.5f        //小陀螺速度调试
#define SPINNER_MAX_ROUNDS  20   //小锟剿讹拷时锟饺︼拷玫锟�
#define ECD_FULL_ROUND 8192 //一圈ECD值实取值0-8191

//yawPID锟剿诧拷
#define YAW_SPD_KP 1300000.0f
#define YAW_SPD_KI 1000.0f
#define YAW_SPD_KD 100.0f

#define YAW_VOLT_MAX_OUT  30000.0f
#define YAW_VOLT_MAX_IOUT 3000.0f

// #define YAW_AGL_KP 0.07f
#define YAW_AGL_KP 0.05f
#define YAW_AGL_KI 0.00f
#define YAW_AGL_KD 0.00f

#define YAW_AGL_SPD_MAX_OUT (50.0f)
#define YAW_AGL_SPD_MAX_IOUT (11.7f)

#define YAW_SPD_FILTER_NUM 1.0f
//************************锟结构锟藉定**********//
//锟教抵憋拷涌疲锟街伙拷俣锟�
struct MotorControl_s{
    pid_type_def    vpid;
    fp32            presentMotorSpeed;
    fp32            wantedMotorSpeed;
    int16_t         giveCurrent;
};
//系
enum MovingAxis_e{
    MovingAxis_e_GimbalAxis,
    MovingAxis_e_ChassisAxis,
};
//锟剿匡拷
struct RobotControl_s{
    fp32 vx,vy,w;
    first_order_filter_type_t vx_filter,vy_filter,w_filter;
    enum MovingAxis_e axis;
};
//台yaw
struct gimbalMotorCtrl_s{
    //rads
    const fp32 * anglePoint;          //态玫慕嵌锟轿�
    fp32 nowAbsoluteAngle;      //前态
    uint32_t nowTime;           //前时
    fp32 lastAbsoluteAngle;     //锟较达拷态
    uint32_t lastTime;          //锟较达拷时
    fp32 radSpeed;              //锟劫讹拷 rad/s
    fp32 wantedAbsoluteAngle;   //目态

    //ECDs
    const uint16_t * ECDPoint;        //ECD位
    uint16_t maxECD,minECD,middleECD,nowECD;    //锟叫★拷锟街碉拷锟角癊CD
    int16_t nowRounds;          //转圈
    int16_t maxRounds;          //转锟饺�

    //PIDs
    pid_type_def spd_pid;       //锟节匡拷锟劫度碉拷PID俣锟� rad/s, 锟紾M6020压 int16
    pid_type_def agl_pid;       //锟节科角度碉拷PID嵌锟� rad,俣锟絩ad/s

    //
    int16_t giveVolt;       //GM6020锟酵碉拷压
    uint8_t zeroVoltMark;

    // 一锟剿诧拷锟剿劫讹拷
    first_order_filter_type_t spd_filter;
};
//台yaw锟节堤角度结构
struct Angle_s{
    int16_t initGimbalYawECD;       //时ECD
    int16_t nowGimbalYawECD;        //目前ECD
    int32_t rotateRounds;           //目前转锟剿讹拷圈
    fp32    gimbalAngleFromChassis; //台锟侥角度ｏ拷时为锟津。碉拷位锟饺★拷
};

//************************锟节科碉拷全锟街憋拷**********//
const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD}; //PID 始
static struct MotorControl_s driveMotor[4];
static struct Angle_s relativeAngle;   //锟节硷拷录台锟教角讹拷锟街�,台锟狡猴拷小要
static const RC_ctrl_t *rc_p;   //遥位指锟诫，要始
static enum RobotState_e robotMode;   //模式
static struct RobotControl_s robotTotalSpeedControl;
static int16_t vx_channel,vy_channel,w_channel; //遥deadband limit锟街�
static const motor_measure_t* yaw_measure;
const int16_t * gimbalRounds;
static int16_t vx_channel,vy_channel,w_channel; //遥deadband limit锟街�
//yaw
static struct gimbalMotorCtrl_s gimbalYawCtrl;
static struct gimbalMotorCtrl_s *gimbal_yaw_ctrl_point=&gimbalYawCtrl;
static fp32 yawSpdPIDco[3]={YAW_SPD_KP,YAW_SPD_KI,YAW_SPD_KD};
static fp32 yawAglPIDco[3]={YAW_AGL_KP,YAW_AGL_KI,YAW_AGL_KD};
extKalman_t spd_pid_kalman_out;
//************************锟秸的科憋拷**********//
static const can_send_data_s* YAW;
static const can_send_data_channel_s* RC_channel;
static const can_send_data_nuc_yaw_s* NUC_YAW;
static const can_send_data_s* keyboard;

enum RobotState_e getRobotPresentMode()
{
	if(toe_is_error(DBUS_TOE))
		return RobotState_e_Powerless;
	if(switch_is_up(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_GimbalCar;
	else if(switch_is_mid(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_CommonCar;
	else if(switch_is_down(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_Spinner;
}
static void initChassis(void){
	//PID始
	int i;
	for(i=0;i<4;i++)
		PID_init(&(driveMotor[i].vpid),PID_POSITION,motor_speed_pid,M3505_MOTOR_SPEED_PID_MAX_OUT,M3505_MOTOR_SPEED_PID_MAX_IOUT);
	//锟剿诧拷始
  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  const static fp32 chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM}; 
  first_order_filter_init(&(robotTotalSpeedControl.vx_filter), CHASSIS_CONTROL_TIME_MS, chassis_x_order_filter);
  first_order_filter_init(&(robotTotalSpeedControl.vy_filter), CHASSIS_CONTROL_TIME_MS, chassis_y_order_filter);
	first_order_filter_init(&(robotTotalSpeedControl.w_filter), CHASSIS_CONTROL_TIME_MS, chassis_w_order_filter);
  //yaw锟绞�
	relativeAngle.initGimbalYawECD=YAW_INIT_ECD;
	yaw_measure=get_yaw_gimbal_motor_measure_point();
	relativeAngle.nowGimbalYawECD=yaw_measure->ecd;
	for(int i=0;i<4;i++){
	  driveMotor[i].presentMotorSpeed=get_chassis_motor_measure_point(i)->speed_rpm;
	}
}

//模式取通值
static void analyseTotalControl(){
	robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;//默锟较碉拷系
	robotTotalSpeedControl.vx=robotTotalSpeedControl.vy=robotTotalSpeedControl.w=0;
	
	if(robotMode==RobotState_e_Powerless){
		return;
	}
	else if(RobotState_e_CommonCar==robotMode)
    {
        robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;
        

          if(RC_channel->E)  
              robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
          if(RC_channel->D)  
              robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
          if(RC_channel->S)  
              robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
          if(RC_channel->F)  
              robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
					if(RC_channel->A)  
              robotTotalSpeedControl.w-=KEYBOARD_CONTROL_ROBOT_SPEED_W;
					if(RC_channel->G)  
              robotTotalSpeedControl.w+=KEYBOARD_CONTROL_ROBOT_SPEED_W;


					rc_deadband_limit(RC_channel->channel_3,vx_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(RC_channel->channel_2,vy_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(RC_channel->channel_0,w_channel,CHASSIS_RC_DEADLINE);
					robotTotalSpeedControl.vx -=vx_channel*CHASSIS_VX_RC_SEN;
          robotTotalSpeedControl.vy +=vy_channel*CHASSIS_VY_RC_SEN;
          robotTotalSpeedControl.w -=w_channel*CHASSIS_WZ_RC_SEN;
//				}
    }
	//台小使台系实锟角侯开始锟侥碉拷锟叫癸拷
  //锟狡凤拷式同
	else if(robotMode==RobotState_e_GimbalCar||robotMode==RobotState_e_Spinner)
   {
				robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;
				
		 if (robotMode==RobotState_e_Spinner) {
				if(RC_channel->W)  //前
						robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_X;
				if(RC_channel->S)  //
						robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_X;
				if(RC_channel->A)  //
						robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_Y;
				if(RC_channel->D)  //
						robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_Y;
			} 
		 else {
				if(RC_channel->W)  //前
						robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
				if(RC_channel->S)  //
						robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
				if(RC_channel->A)  //
						robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
				if(RC_channel->D)  //
						robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
			}
			
			  rc_deadband_limit(RC_channel->channel_3,vx_channel,CHASSIS_RC_DEADLINE);
				rc_deadband_limit(RC_channel->channel_2,vy_channel,CHASSIS_RC_DEADLINE);
			  robotTotalSpeedControl.vx -= vx_channel*CHASSIS_VX_RC_SEN;
        robotTotalSpeedControl.vy += vy_channel*CHASSIS_VY_RC_SEN;

			if(robotMode==RobotState_e_Spinner){
				robotTotalSpeedControl.w=SPINNER_W;
			}
		
	 }
}
//锟矫德碉拷ECD值越嵌锟�
void refreshECD(){
  relativeAngle.nowGimbalYawECD=get_yaw_gimbal_motor_measure_point()->ecd;
	relativeAngle.gimbalAngleFromChassis=(relativeAngle.nowGimbalYawECD-relativeAngle.initGimbalYawECD)*2*PI/ECD_FULL_ROUND;
	//usart_printf("%d %f\r\n",relativeAngle.nowGimbalYawECD,relativeAngle.gimbalAngleFromChassis);
}
//平
static void firstOrderFilt()
{
    first_order_filter_cali(&robotTotalSpeedControl.vx_filter
	,robotTotalSpeedControl.vx);
    first_order_filter_cali(&robotTotalSpeedControl.vy_filter,robotTotalSpeedControl.vy);
    first_order_filter_cali(&robotTotalSpeedControl.w_filter,robotTotalSpeedControl.w);
    robotTotalSpeedControl.vx=robotTotalSpeedControl.vx_filter.out;
    robotTotalSpeedControl.vy=robotTotalSpeedControl.vy_filter.out;
    robotTotalSpeedControl.w=robotTotalSpeedControl.w_filter.out;
}
static void calcWheelVelocity(){
	fp32 sin_yaw,cos_yaw;
	fp32 vx,vy,w;
	sin_yaw=arm_sin_f32(relativeAngle.gimbalAngleFromChassis);
	cos_yaw=arm_cos_f32(relativeAngle.gimbalAngleFromChassis);
	
	w=robotTotalSpeedControl.w;
	if(robotTotalSpeedControl.axis==MovingAxis_e_GimbalAxis){
		vx=cos_yaw*robotTotalSpeedControl.vx-sin_yaw*robotTotalSpeedControl.vy;
		vy=sin_yaw*robotTotalSpeedControl.vx+cos_yaw*robotTotalSpeedControl.vy;
	}
	else if(robotTotalSpeedControl.axis==MovingAxis_e_ChassisAxis)
    {
        vx=robotTotalSpeedControl.vx;
        vy=robotTotalSpeedControl.vy;
    }
		#ifdef OMNI_CHASSIS
	  driveMotor[0].wantedMotorSpeed=-vx+vy-w;
	  driveMotor[1].wantedMotorSpeed=-vx-vy-w;
	  driveMotor[2].wantedMotorSpeed=vx-vy-w;
		driveMotor[3].wantedMotorSpeed=vx+vy-w;
		#endif
		#ifdef MEC_CHASSIS
	  driveMotor[0].wantedMotorSpeed=-vx-vy-w;    //锟劫度笺公式
    driveMotor[1].wantedMotorSpeed=vx-vy-w;
    driveMotor[2].wantedMotorSpeed=vx+vy-w;
    driveMotor[3].wantedMotorSpeed=-vx+vy-w;
		#endif
}
static void calcGivenCurrent(){
   uint8_t i;   
	 chassis_move_t chassis_power_control_data;

    for(i=0;i<4;i++)
    {
        driveMotor[i].presentMotorSpeed=get_chassis_motor_measure_point(i)->speed_rpm*M3508_MOTOR_RPM_TO_VECTOR;    
        PID_calc(&(driveMotor[i].vpid),driveMotor[i].presentMotorSpeed,driveMotor[i].wantedMotorSpeed);   
        driveMotor[i].giveCurrent=driveMotor[i].vpid.out;    
    }
		for(i = 0; i < 4; i++)
    {
        chassis_power_control_data.motor_speed_pid[i].out = driveMotor[i].giveCurrent;
    }
		   chassis_power_control(&chassis_power_control_data);
    for(i = 0; i < 4; i++)
    {
        driveMotor[i].giveCurrent = chassis_power_control_data.motor_speed_pid[i].out;
    }
}
static void initGimbalYaw(void)
{
    uint8_t i;
    //始锟皆角讹拷位
    gimbalYawCtrl.anglePoint=&YAW->yaw; 

    // 始ECD
    gimbalYawCtrl.ECDPoint=&(get_yaw_gimbal_motor_measure_point()->ecd);


    //始转锟侥角讹拷
    gimbalYawCtrl.maxRounds=30000;


    // 始PID
	  PID_init(&(gimbal_yaw_ctrl_point->agl_pid),PID_POSITION,yawAglPIDco,YAW_AGL_SPD_MAX_OUT,YAW_AGL_SPD_MAX_IOUT);
    PID_init(&(gimbal_yaw_ctrl_point->spd_pid),PID_POSITION,yawSpdPIDco,YAW_VOLT_MAX_OUT,YAW_VOLT_MAX_IOUT);
    
    //锟剿诧拷始
    const static fp32 gimbal_yaw_order_filter[1] = {YAW_SPD_FILTER_NUM};

    first_order_filter_init(&(gimbalYawCtrl.spd_filter), CHASSIS_CONTROL_TIME_MS, gimbal_yaw_order_filter);
    
    //始前锟角度猴拷目嵌取锟絧id瘫锟秸伙拷锟�
		gimbal_yaw_ctrl_point->zeroVoltMark=0;
		gimbal_yaw_ctrl_point->nowAbsoluteAngle=*(gimbal_yaw_ctrl_point->anglePoint);
		gimbal_yaw_ctrl_point->nowTime=HAL_GetTick();
		gimbal_yaw_ctrl_point->wantedAbsoluteAngle=gimbal_yaw_ctrl_point->nowECD;
    
		
		PID_clear(&(gimbalYawCtrl.spd_pid));
    PID_clear(&(gimbalYawCtrl.agl_pid));
    
}
void refreshAngleStates(struct gimbalMotorCtrl_s * c)
{
    //前态锟角和劫讹拷
    c->lastAbsoluteAngle=c->nowAbsoluteAngle;
    c->lastTime=c->nowTime;

    c->nowAbsoluteAngle=*(c->anglePoint);
    c->nowTime=xTaskGetTickCount();
	
		float angleDifference = c->nowAbsoluteAngle - c->lastAbsoluteAngle;

		if (angleDifference > PI) {
				angleDifference -= 2 * PI; // 超过π，减去2π，保持在-π到π之间
		} else if (angleDifference < -PI) {
				angleDifference += 2 * PI; // 小于-π，加上2π，保持在-π到π之间
		}

		c->radSpeed = angleDifference / (c->nowTime - c->lastTime);
    //前ECD锟角讹拷
    c->nowECD=*(c->ECDPoint);
}
/**
 * @brief 锟捷伙拷状态pitchyaw 转叽痰俣锟�
 * 
 */
int y_data=0;
void getControlAngles(void)
{

    static fp32 selfTargetOffsetPitch=0,selfTargetOffsetYaw=0;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(RC_channel->channel_0, yaw_channel, CHASSIS_RC_DEADLINE);
    //锟揭ｏ拷锟教╮ad锟劫度的匡拷

    //未锟皆猴拷锟狡斤拷樱锟揭伙拷嵌龋锟狡拷睿拷偏锟狡碉拷
    
    //bad yaw car转模式台突然转为台没锟叫碉拷希位锟角ｏ拷一转锟教★拷
    // 一模式一模式同时希锟侥角讹拷为前锟角度★拷
    static enum RobotState_e lastMode=RobotState_e_Powerless;   //始为
    
    lastMode=robotMode;
	  gimbalYawCtrl.wantedAbsoluteAngle=0.0f;
	  

    gimbalYawCtrl.zeroVoltMark=0;

//模式选锟街�

    if(robotMode==RobotState_e_Powerless)
    {
        gimbalYawCtrl.zeroVoltMark=1;
    }    
    else 
    {
    
     	gimbalYawCtrl.wantedAbsoluteAngle=YAW->nuc_yaw;
      //usart_printf("%f,%f\r\n",gimbalYawCtrl.wantedAbsoluteAngle,gimbalYawCtrl.nowAbsoluteAngle);			
			
			
    }
    if(rc_p->key.v&KEY_PRESSED_OFFSET_Z)
			CAN_cmd_chassis_reset_ID();
    
    
}
/**
 * @brief 锟角度伙拷为(-PI,PI)围
 * 
 * @param rawAngle 
 * @return fp32 
 */
fp32 radFormat(fp32 rawAngle)   //test done
{
    while(rawAngle>PI)
        rawAngle-=(2*PI);
    while(rawAngle<(-PI))
        rawAngle+=(2*PI);
    return rawAngle;
}

/**
 * @brief ECD值锟饺伙拷为(0,8191)围
 * 
 * @param rawECD 
 * @return uint16_t 
 */
static uint16_t ECDFormat(int16_t rawECD)     //test done
{
    while(rawECD<0)
        rawECD+=ECD_FULL_ROUND;
    while(rawECD>=ECD_FULL_ROUND)
        rawECD-=ECD_FULL_ROUND;
    return (uint16_t)rawECD;
}

/**
 * @brief 0-8191ECD值转为-PI~PI锟侥伙拷值,只映一一去
 * 0->-PI,8192->PI
 * 
 * @param ecd 
 * @return fp32 
 */
fp32 ECD2Rad(uint16_t ecd)  //test done
{
    return ((fp32)ecd)*(2*PI)/ECD_FULL_ROUND-PI;
}
void limitAngles(struct gimbalMotorCtrl_s * c)
{
//test done
    if(radFormat(c->wantedAbsoluteAngle - c ->nowECD) > 4096)
        c->wantedAbsoluteAngle = c ->nowECD + 4096;
    if(radFormat(c->wantedAbsoluteAngle - c ->nowECD) < -4096)
        c->wantedAbsoluteAngle = c ->nowECD - 4096;
    
#ifndef HAVE_SLIP_RING

    fp32 posiSafe=radFormat(ECD2Rad(c->maxECD)-ECD2Rad(c->nowECD));      //0
    fp32 negSafe=radFormat(ECD2Rad(c->minECD)-ECD2Rad(c->nowECD));       //小0
#endif


}
fp32 PID_spd_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        range_limit_inside(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
    }
    return pid->out;
}
fp32 gimbal_PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = radFormat(set - ref);

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        range_limit_inside(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
//			usart_printf("%f,%f,%f,%f,%f\r\n",gimbalYawCtrl.nowAbsoluteAngle,gimbalYawCtrl.wantedAbsoluteAngle,pid->error[0],pid->Iout,pid->out*YAW_SPD_KP);
    }
    else if (pid->mode == PID_DELTA)
    {
			  if(pid->error[0]<=0){
					pid->error[0]=0;
				}
				else{
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
				}
    }
		
    return pid->out;
}
struct gimbalMotorCtrl_s c;

void calcPID(void)
{
				c=*gimbal_yaw_ctrl_point;
	   
      	gimbal_PID_calc(&(c.agl_pid),c.nowAbsoluteAngle,c.wantedAbsoluteAngle);  // 关乎旋转方向的PID控制器
        // 输出了所需旋转速度。
        //对速度进行一阶滤波d
				first_order_filter_cali(&(c.spd_filter),(c.agl_pid).out);
//				c.agl_pid.out = 0.005;
        PID_spd_calc(&(c.spd_pid),c.radSpeed,c.spd_filter.out);   // 普通的速度控制环
				
        gimbal_yaw_ctrl_point->giveVolt=c.spd_pid.out;     //给电压
	//调试yaw轴pid
//				usart_printf("%f,%f,%f,%f\r\n",c.radSpeed,c.nowAbsoluteAngle,c.spd_filter.out,c.agl_pid.out);
	      usart_printf("%f,%f,%f,%f,%f\r\n",c.nowAbsoluteAngle,c.wantedAbsoluteAngle,c.agl_pid.out,gimbal_yaw_ctrl_point->giveVolt,c.spd_pid.out);
}


void chassis_task(void const *pvParameters)
{
	osDelay(CHASSIS_TASK_INIT_TIME);

	rc_p=get_remote_control_point();
	YAW=get_yaw_measure_point();
	RC_channel=get_channel_measure_point();
	
	initChassis();
	initGimbalYaw();
	gimbalYawCtrl.wantedAbsoluteAngle=0.0f;
	while(1){
		robotMode=RC_channel->mode;
		  
	  analyseTotalControl();
		refreshECD();
		getControlAngles();
		refreshAngleStates(gimbal_yaw_ctrl_point); 
		limitAngles(gimbal_yaw_ctrl_point);
    calcPID();

		firstOrderFilt();
		calcWheelVelocity();
		
		calcGivenCurrent();
		if(robotMode==RobotState_e_Powerless){
			CAN_cmd_chassis(0,0,0,0);
		  CAN_cmd_gimbal(0,0,0,0);
		}
		else
		{
		CAN_cmd_chassis(driveMotor[0].giveCurrent,driveMotor[1].giveCurrent,
		driveMotor[2].giveCurrent,driveMotor[3].giveCurrent);
		CAN_cmd_gimbal(gimbal_yaw_ctrl_point->giveVolt,0,0,0);
		}
		osDelay(CHASSIS_CONTROL_TIME_MS);
		
	}
}
