/**
	作者：胡子健
	
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

//是否开启底盘速度的一阶低通滤波
//#define FILTER_CHASSIS_ON



//第一版，写一个buffer来控制
//第一目标，原地旋转功率是最大的，buffer
//第二目标，当需要移动时，减小转速空出buffer
//第三目标，当需要移动时，

/*本套代码底盘向前的ID设置
		1  |  4
		-------
		2  |  3
*/

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
//****************************底盘控制量***************//
#define KEYBOARD_CONTROL_ROBOT_SPEED_X 2.8f      
#define KEYBOARD_CONTROL_ROBOT_SPEED_Y 2.8f
#define KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_X 1.5f
#define KEYBOARD_CONTROL_ROBOT_SPINNER_SPEED_Y 1.5f
#define KEYBOARD_CONTROL_ROBOT_SPEED_W 1.0f //暂时不用，全向轮不需要底盘旋转功能
#define SPINNER_W   4.0f        //2.0对应70w 小陀螺最多使用当前功率的百分之七十
#define SPINNER_MAX_ROUNDS  20   
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
//************************电机控制结构体**********//
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
//速度控制结构体
struct RobotControl_s{
    fp32 vx,vy,w;
    first_order_filter_type_t vx_filter,vy_filter,w_filter;
    enum MovingAxis_e axis;
};
//云台yaw
struct gimbalMotorCtrl_s{
    //rads
    const fp32 * anglePoint;          
    fp32 nowAbsoluteAngle;      //当前姿态角
    uint32_t nowTime;           //当前时间
    fp32 lastAbsoluteAngle;     //上一姿态
    uint32_t lastTime;          //上一时间
    fp32 radSpeed;              //角速度 rad/s，目前还没有完全解析成真值，有待改进
    fp32 wantedAbsoluteAngle;   //目标姿态

    //ECDs
    const uint16_t * ECDPoint;        //ECD位
    uint16_t maxECD,minECD,middleECD,nowECD;    
    int16_t nowRounds;          
    int16_t maxRounds;          

    //PIDs
    pid_type_def spd_pid;       
    pid_type_def agl_pid;       

    //
    int16_t giveVolt;       
    uint8_t zeroVoltMark;

    first_order_filter_type_t spd_filter;
};
//台yaw锟节堤角度结构
struct Angle_s{
    int16_t initGimbalYawECD;       
    int16_t nowGimbalYawECD;        
    int32_t rotateRounds;           
    fp32    gimbalAngleFromChassis; 
};

//************************全局变量**********//
const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD}; //PID 始
static struct MotorControl_s driveMotor[4];
static struct Angle_s relativeAngle;   
static const RC_ctrl_t *rc_p;   
static enum RobotState_e robotMode;   //模式
static struct RobotControl_s robotTotalSpeedControl;
static int16_t vx_channel,vy_channel,w_channel; 
static const motor_measure_t* yaw_measure;
const int16_t * gimbalRounds;
//yaw
static struct gimbalMotorCtrl_s gimbalYawCtrl;
static struct gimbalMotorCtrl_s *gimbal_yaw_ctrl_point=&gimbalYawCtrl;
static fp32 yawSpdPIDco[3]={YAW_SPD_KP,YAW_SPD_KI,YAW_SPD_KD};
static fp32 yawAglPIDco[3]={YAW_AGL_KP,YAW_AGL_KI,YAW_AGL_KD};
extKalman_t spd_pid_kalman_out;
//************************can通信变量*********//
static const can_send_data_s* YAW;
static const can_send_data_channel_s* RC_channel;

enum RobotState_e getRobotPresentMode()
{
	if(toe_is_error(DBUS_TOE))
		return RobotState_e_Powerless;
	if(switch_is_up(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_Spinner;
	else if(switch_is_mid(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_GimbalCar;
	else if(switch_is_down(rc_p->rc.s[CHASSIS_MODE_CHANNEL]))
		return RobotState_e_CommonCar;
}
static void initChassis(void){
	//PID初始化
	int i;
	for(i=0;i<4;i++)
		PID_init(&(driveMotor[i].vpid),PID_POSITION,motor_speed_pid,M3505_MOTOR_SPEED_PID_MAX_OUT,M3505_MOTOR_SPEED_PID_MAX_IOUT);
  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  const static fp32 chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM}; 
  first_order_filter_init(&(robotTotalSpeedControl.vx_filter), CHASSIS_CONTROL_TIME_MS, chassis_x_order_filter);
  first_order_filter_init(&(robotTotalSpeedControl.vy_filter), CHASSIS_CONTROL_TIME_MS, chassis_y_order_filter);
	first_order_filter_init(&(robotTotalSpeedControl.w_filter), CHASSIS_CONTROL_TIME_MS, chassis_w_order_filter);
	
	relativeAngle.initGimbalYawECD=YAW_INIT_ECD;
	yaw_measure=get_yaw_gimbal_motor_measure_point();
	relativeAngle.nowGimbalYawECD=yaw_measure->ecd;
	for(int i=0;i<4;i++){
	  driveMotor[i].presentMotorSpeed=get_chassis_motor_measure_point(i)->speed_rpm;
	}
}

//模式取通值
int stop;
int count = 0;
static int press=0;
static void analyseTotalControl(){
	robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;//底盘坐标系
	robotTotalSpeedControl.vx=robotTotalSpeedControl.vy=robotTotalSpeedControl.w=0;
	
	if(robotMode==RobotState_e_Powerless){
		return;
	}
	else if(RobotState_e_CommonCar==robotMode)
    {
        robotTotalSpeedControl.axis=MovingAxis_e_ChassisAxis;
				if(RC_channel)
				{
					if(RC_channel->W)  //前
							robotTotalSpeedControl.vx-=KEYBOARD_CONTROL_ROBOT_SPEED_X;
					if(RC_channel->S)  //后
							robotTotalSpeedControl.vx+=KEYBOARD_CONTROL_ROBOT_SPEED_X;
					if(RC_channel->A)  //左
							robotTotalSpeedControl.vy-=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
					if(RC_channel->D)  //右
							robotTotalSpeedControl.vy+=KEYBOARD_CONTROL_ROBOT_SPEED_Y;
					
					
					rc_deadband_limit(RC_channel->channel_3,vx_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(RC_channel->channel_2,vy_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(RC_channel->channel_0,w_channel,CHASSIS_RC_DEADLINE);
				}
				
				//测试底盘用的，不常用
				else
				{
					rc_deadband_limit(rc_p->rc.ch[HANDLE_LEFT_BF],vx_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(rc_p->rc.ch[HANDLE_LEFT_LR],vy_channel,CHASSIS_RC_DEADLINE);
					rc_deadband_limit(rc_p->rc.ch[HANDLE_RIGHT_LR],w_channel,CHASSIS_RC_DEADLINE);
				}



					robotTotalSpeedControl.vx -=vx_channel*CHASSIS_VX_RC_SEN;
          robotTotalSpeedControl.vy +=vy_channel*CHASSIS_VY_RC_SEN;
          robotTotalSpeedControl.w -=w_channel*CHASSIS_WZ_RC_SEN;
				
    }
	else if(robotMode==RobotState_e_GimbalCar||robotMode==RobotState_e_Spinner)
   {
				robotTotalSpeedControl.axis=MovingAxis_e_GimbalAxis;

		 //整抽象活，之后得大改
		 if (robotMode==RobotState_e_Spinner) {
				robotTotalSpeedControl.w=SPINNER_W;
			 	set_warning_power(120);
				set_warning_power_buff(55);
				set_least_power_buff(40);
			 
			 //进入初始模式
			 if(RC_channel->W||RC_channel->S||RC_channel->A||RC_channel->D)
			 {
				 robotTotalSpeedControl.w = SPINNER_W/3;
				set_warning_power(120);
				set_warning_power_buff(50);
				set_least_power_buff(5);
				 press = 1;
			 }
			 if(!((RC_channel->W)||(RC_channel->S)||(RC_channel->A)||(RC_channel->D))&&press ==1)
			 {
				 press = 0;
				 stop = 1;
			 }
			 if(stop == 1 && count<200)
			 {
				 set_warning_power(40);
				 set_warning_power_buff(50);
				 set_least_power_buff(50);
			 }
			 count++;
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
			 set_warning_power(50);
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

		
	 }
}

void refreshECD(){
  relativeAngle.nowGimbalYawECD=get_yaw_gimbal_motor_measure_point()->ecd;
	relativeAngle.gimbalAngleFromChassis=(relativeAngle.nowGimbalYawECD-relativeAngle.initGimbalYawECD)*2*PI/ECD_FULL_ROUND;
}

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
	  driveMotor[0].wantedMotorSpeed=-vx-vy-w;    
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
    gimbalYawCtrl.anglePoint=&YAW->yaw; 

    gimbalYawCtrl.ECDPoint=&(get_yaw_gimbal_motor_measure_point()->ecd);


    gimbalYawCtrl.maxRounds=30000;


	  PID_init(&(gimbal_yaw_ctrl_point->agl_pid),PID_POSITION,yawAglPIDco,YAW_AGL_SPD_MAX_OUT,YAW_AGL_SPD_MAX_IOUT);
    PID_init(&(gimbal_yaw_ctrl_point->spd_pid),PID_POSITION,yawSpdPIDco,YAW_VOLT_MAX_OUT,YAW_VOLT_MAX_IOUT);
    
    //滤波参数
    const static fp32 gimbal_yaw_order_filter[1] = {YAW_SPD_FILTER_NUM};

    first_order_filter_init(&(gimbalYawCtrl.spd_filter), CHASSIS_CONTROL_TIME_MS, gimbal_yaw_order_filter);
    
    gimbal_yaw_ctrl_point->zeroVoltMark=0;
		gimbal_yaw_ctrl_point->nowAbsoluteAngle=*(gimbal_yaw_ctrl_point->anglePoint);
		gimbal_yaw_ctrl_point->nowTime=HAL_GetTick();
		gimbal_yaw_ctrl_point->wantedAbsoluteAngle=gimbal_yaw_ctrl_point->nowECD;
    
		
		PID_clear(&(gimbalYawCtrl.spd_pid));
    PID_clear(&(gimbalYawCtrl.agl_pid));
    
}
void refreshAngleStates(struct gimbalMotorCtrl_s * c)
{
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
    c->nowECD=*(c->ECDPoint);
}
/**
 * @brief 控yaw使用，目前先不使用
 */
int y_data=0;
void getControlAngles(void)
{

    static fp32 selfTargetOffsetPitch=0,selfTargetOffsetYaw=0;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(RC_channel->channel_0, yaw_channel, CHASSIS_RC_DEADLINE);
    static enum RobotState_e lastMode=RobotState_e_Powerless;   //初始为无力模式
    
    lastMode=robotMode;
	  gimbalYawCtrl.wantedAbsoluteAngle=0.0f;
	  

    gimbalYawCtrl.zeroVoltMark=0;

    if(robotMode==RobotState_e_Powerless)
    {
        gimbalYawCtrl.zeroVoltMark=1;
    }    
    else 
    {
    
     	gimbalYawCtrl.wantedAbsoluteAngle=YAW->nuc_yaw;
			
			
    }
    if(rc_p->key.v&KEY_PRESSED_OFFSET_Z)
			CAN_cmd_chassis_reset_ID();
    
    
}
/**
 * @brief format the angle into (-PI,PI)
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
 * @brief format ECD into 0 to full ECD-1
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
 * @brief 0-8191ECD值转为-PI~PI
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
        PID_spd_calc(&(c.spd_pid),c.radSpeed,c.spd_filter.out);   // 普通的速度控制环
				
        gimbal_yaw_ctrl_point->giveVolt=c.spd_pid.out;     //给电压
	//调试yaw轴pid
//				usart_printf("%f,%f,%f,%f\r\n",c.radSpeed,c.nowAbsoluteAngle,c.spd_filter.out,c.agl_pid.out);
	      
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
		if(toe_is_error(DBUS_TOE)&&RC_channel)
			robotMode = RC_channel->mode;
		else
			robotMode = getRobotPresentMode();
		  
	  analyseTotalControl();
		refreshECD();
		getControlAngles();
		refreshAngleStates(gimbal_yaw_ctrl_point); 
		limitAngles(gimbal_yaw_ctrl_point);
    calcPID();

		#ifdef CHASSIS_FILTER_ON
				firstOrderFilt();
		#endif
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
		CAN1_send_referee(get_shoot_speed(),get_robot_id());
//		usart_printf("%f,%f\r\n",driveMotor[0].presentMotorSpeed,driveMotor[0].wantedMotorSpeed);
//		usart_printf("%d\r\n",gimbal_yaw_ctrl_point->nowECD);
		osDelay(CHASSIS_CONTROL_TIME_MS);
		
	}
}

