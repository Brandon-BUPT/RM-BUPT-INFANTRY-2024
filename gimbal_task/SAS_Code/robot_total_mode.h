/**
 * @file robot_total_mode.h
 * @author 陈卓勋 电控组 UCAS-SAS 中国科学院大学机器人队
 * @brief 根据遥控器宏观判断步兵机器人的总状态（包括是否自动），让云台、底盘、发射控制任务根据总状态判断分状态，保证一致性
 *          另外，在.c文件中定义变量，记录robotAuto状态。
 *
 * @version 0.2
 * @date 2022-02-05 修改为四个模式：无力、普通车、小陀螺车、坏yaw走位车
 *                                 掉线    C       V        B
 *                                          上      中      下
 * 当键盘开始控制后，遥控器拨杆就暂时失去控制了。当拨杆状态发生改变，那么清除键盘控制标记，遥控器重新获得更改状态的权限
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef ROBOT_TOTAL_MODE_H
#define ROBOT_TOTAL_MODE_H

//version 0.2
//shift 开启摩擦轮，ctrl关闭摩擦轮
// 鼠标左键按一下发射一个，按住连发，右键按住允许nuc控制发射（nuc可以选择单发或者连发）
// cvb进入不同车的模式
// z开启自瞄
// x关闭自瞄
// 鼠标上下pitch，左右：在badyaw车里是整体左右旋转，在普通车和小陀螺车里是云台独立左右旋转
// 前后控制有两种。es或者ws。
// 云台在一定时间内控制无法到达指定位置，就放弃控制
enum RobotState_e{
    RobotState_e_Powerless,     //无力模式，遥控器掉线时进入

    RobotState_e_CommonCar,     //普通车模式，键盘控制底盘坐标系运动，edsfag分别对应前后平移、左右旋转、左右平移。
                                //鼠标控制云台独立旋转。
                                // 此时，遥控器左边摇杆控制底盘在底盘坐标系前后平移、左右旋转。右拨杆确定云台独立的旋转
                                // 拨杆朝上时进入，以及按c时进入
                                // auto NUC控制云台独立的方向

    RobotState_e_BadYawCar,     //坏yaw 车模式，比赛中yaw可能卡住。进入此模式时，会尝试回正云台，一定时间内回正不了就放弃
                                //wsad前后左右平移，鼠标pitch上下和整体旋转
                                // 此时，遥控器左边摇杆控制机器人在云台坐标系中前后左右平移，
                                //拨杆在下时进入，以及按b时进入
                                // auto NUC控制云台pitch和整体转向：角度PID控制获得的角速度乘以系数后交给底盘控制任务
                                

    RobotState_e_Spinner,       //小陀螺模式。云台根据机械结构自动确定是小陀螺还是准小陀螺。
                                //拨杆在中时进入，以及按v时进入
                                //遥控器左摇杆控制在云台坐标系下的前后左右平移，右摇杆控制云台旋转
                                // auto NUC控制云台独立方向
};

extern int robotIsAuto(void);  // 对外提供的接口，返回目前机器人是否自动
const enum RobotState_e * getRobotPresentMode(void);  //返回机器人当前状态指针
void robot_total_mode_task(void const *pvParameters);

#endif // !ROBOT_TOTAL_MODE_H

/*
抽象出各个数据之间的联系和本质。

PID和电机本身所处的环境有关。不随控制目标的单位改变而改变。




*/
