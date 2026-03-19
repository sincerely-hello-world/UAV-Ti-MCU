#pragma once

#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>

#include "AC_Math.h"
#include "Receiver.h"
#include "drv_Uart2.h"
#include "drv_Uart7.h"
#include "drv_Uart0.h"
#include "drv_PWMOut.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"

#include "sysctl.h"
#include "hw_memmap.h"
#include "pin_map.h"
#include "gpio.h"
#include "pwm.h"

#include "drv_Uart2.h"

#include "drv_LED.h"

#define Pi 3.14

//由于采用了P参数算法所以会有一个高度的损失故采用loss_z来进行分析
#define loss_z 10
//xyz的误差值估计
#define err_x 3
#define err_y 3
#define err_z 5
//t265的数据的长度设定
#define T265_DATA_LEN 19


//t265数据发送通信协议
bool Data_Send_t265(uint8_t* info, int len);
//红色LED
void LED_Red( float on );
//绿色LED
void LED_Green( bool on );
//蓝色LED
void LED_Blue( bool on );
//清空颜色
void LED_Clean();
//初始化使能
void ALL_Enable();
//设定正确的xyz目标值
void set_current_x_y_z(float x, float y, float z);
//查看正确的xyz目标值
void see_current_x_y_z(float* pos);
//查看t265的xyz目标值
void see_t265_x_y_z(float* pos);
//起飞
void Takeoff(int zt,float Height);
//控高
bool Goto_z(int zt, float want_Z);
//设置PID函数
float PID_set(float kp, float kd, float now, float old, float want);
//置xy向速度
void Vel_xy(float vx, float vy);
//设定翻滚角和俯仰角，并返回翻滚角和俯仰角当前数值
bool setRoll_Pitch(float  pos_roll, float pos_pitch, float* roll, float* pitch);
//快速起飞函数
void Fast_Takeoff();
