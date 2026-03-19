#include "Control_Liu.h"

//定义正确的位置参量
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;
//定义T265位置数据量
//static float t265_x = 0.0f;
//static float t265_y = 0.0f;
//static float t265_z = 0.0f;
//定义老的t265位置数据量（便于pid）
static float t265_x_old = 0.0f;
static float t265_y_old = 0.0f;
static float t265_z_old = 0.0f;

static int Modeinfo;


//引入超声波高度
extern float ultrasonic;

extern float t265_x;
extern float t265_y;
extern float t265_z;//引入t265数据

static int vol = 2;//加速起飞，积分量

static float now_volz; //现在V_z的速度

/*******************************************
函数名：Data_Send_t265
作用：t265数据发送通信协议
作者：Lcs
输入参数：uint8_t* info 发送数据的包
          int len 发送数据的长度
输出参数：bool ok 是否成功
封装等级：2
*******************************************/
/*******************************************
透传数据结构分析：【查看是否是数据 x 作为验证位】[x坐标(6个字符)][y坐标(6个字符)][z坐标(6个字符)]
单个数据结构解析：[符号][百位][十位][个位][小数点后一位][小数点后两位]
单位：cm
*******************************************/
//t265数据发送通信协议
bool Data_Send_t265(uint8_t* info, int len){
	bool ok = false;//设定成功标志变量
//	uint8_t look[2] = "N";
//	Uart7_Send(look,1);
//	int count = 0;
//	while(look[0] != 88 || count < 22){
//		Uart7_Send(look,1);
//		ok = true;
//		count++;
//	}
//	Uart7_Send(info , len);
//	t265_x = (44 - info[0]) * 1.0 * ((info[1] - 48) * 100 + (info[2] - 48) * 10 + (info[3] - 48) * 1 + (info[4] - 48) * 0.1 + (info[5] - 48) * 0.01); //带符号的数据字符串与字符转换 
//	t265_y = (44 - info[6]) * 1.0 * ((info[7] - 48) * 100 + (info[8] - 48) * 10 + (info[9] - 48) * 1 + (info[10] - 48) * 0.1 + (info[11] - 48) * 0.01); // 带符号的数据字符与字符串转换
//	t265_z = (44 - info[12]) * 1.0 * ((info[13] - 48) * 100 + (info[14] - 48) * 10 + (info[15] - 48) * 1 + (info[16] - 48) * 0.1 + (info[17] - 48) * 0.01); // 带符号的数据字符与字符串转换
//	Modeinfo = info[18]; // 指令读取
	
	return ok;
}

/*******************************************
函数名：LED_Red
作用：飞控板亮红灯
作者：Lcs
输入参数：float on 亮灯的亮度
输出参数：无
封装等级：1
*******************************************/
//红色LED灯
void LED_Red( float on )
{
    if( on > 100 )
        on = 100;
    else if( on < 0 )
        on = 0;
    PWMPulseWidthSet( PWM1_BASE, PWM_OUT_7, (5000 - on*49.99f) );
}

/*******************************************
函数名：LED_Green
作用：飞控板亮绿灯
作者：Lcs
输入参数：bool on 亮灯与否（真亮灯，假不亮）
输出参数：无
封装等级：1
*******************************************/
//绿色LED
void LED_Green( bool on )
{
    if( on )
        GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_4, 0 );
    else
        GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4 );
}

/*******************************************
函数名：LED_Blue
作用：飞控板亮蓝灯
作者：Lcs
输入参数：bool on 亮灯与否（真亮灯，假不亮）
输出参数：无
封装等级：1
*******************************************/
//蓝色LED
void LED_Blue( bool on )
{
    if( on )
        GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_3, 0 );
    else
        GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3 );
}

/*******************************************
函数名：LED_Clean
作用：飞控板关闭所有灯
作者：Lcs
输入参数：无
输出参数：无
封装等级：2
*******************************************/
//LED清空状态
void LED_Clean( ){
    LED_Blue(0);
    LED_Red(0);
    LED_Green(0);	
}

/*******************************************
函数名：ALL_Enable
作用：初始化ControlSystem中的各种控制授权
作者：Lcs
输入参数：无
输出参数：无
封装等级：3
*******************************************/
//初始化使能
void ALL_Enable(){
	  Position_Control_Enable();//位置控制使能
	  Altitude_Control_Enable();//高度控制使能
}

/*******************************************
函数名：set_current_x_y_z
作用：设定正确的xyz目标值
作者：Lcs
输入参数：float x 设定的正确x值
          float y 设定的正确y值
          float z 设定的正确z值
输出参数：无
封装等级：0
*******************************************/
//设定正确的xyz目标值
void set_current_x_y_z(float x, float y, float z){
	  current_x = x;
	  current_y = y;
	  current_z = z;
}

/*******************************************
函数名：see_current_x_y_z
作用：查看正确的xyz目标值
作者：Lcs
输入参数：float* pos 存入正确的xyz数据
输出参数：无
封装等级：0
*******************************************/
//查看正确的xyz目标值
void see_current_x_y_z(float* pos){
	  pos[0] = current_x;
	  pos[1] = current_y;
	  pos[2] = current_z;
}

/*******************************************
函数名：see_t265_x_y_z
作用：查看t265的xyz值
作者：Lcs
输入参数：float* pos 存入正确的xyz数据
输出参数：无
封装等级：0
*******************************************/
//查看t265的xyz目标值
void see_t265_x_y_z(float* pos){
//	  pos[0] = t265_x;
//	  pos[1] = t265_y;
//	  pos[2] = t265_z;
}



/*******************************************
函数名：Takeoff
作用：起飞
作者：Lcs
输入参数：int zt 起飞状态，选择为1或者2【1为对地超声波主导起飞，2为 t265 主导起飞】
          float Height 起飞之后所要达到的高度（cm）
输出参数：无
封装等级：3
*******************************************/
//起飞
void Takeoff(int zt,float Height){
	  switch (zt){
		  case 0://原生起飞函数，用于空桨验证
				Position_Control_Takeoff_HeightRelative(Height);
			  break;
			
			default:
				LED_Red(100);
			  break;
	  }
}

/*******************************************
函数名：Goto_z
作用：去到Z位置点上（t265定位）【软件锁】
作者：Lcs
输入参数：int zt 定位状态选择1或2【1是对地超声反馈位置，2是t265反馈位置】
          float want_Z 想要到达的Z点
输出参数：bool 看是否到达，真为到达位置
封装等级：3
*******************************************/
//直线到Z位置
bool Goto_z(int zt, float want_Z){
	  bool dao = false;//是否到达标志变量
	  switch(zt){
			case 1:
				if(ultrasonic > want_Z){
					if(ultrasonic >= want_Z + err_z && ultrasonic <= want_Z - err_z){//设定误差区间
					  Position_Control_set_TargetVelocityZ(0);
						Position_Control_set_ZLock();
						dao = true;
					}
				}
				else{
					Position_Control_set_TargetVelocityZ(constrain_float((want_Z - ultrasonic) * 0.5, 10));
				}
				break;
			case 2:
				if(t265_z >= want_Z + 5 && t265_z <= want_Z - 5){
					Position_Control_set_TargetVelocityZ(0);
					Position_Control_set_ZLock();
					dao = true;
				}
				else{
					Position_Control_set_TargetVelocityZ(constrain_float((want_Z - t265_z) * 0.5, 10));
				}
				break;
			default:
				LED_Red(100);
			  break;
		}
		return dao;
}

/*******************************************
函数名：PID_set
作用：设置PID数值
作者：Lcs
输入参数：float kp P系数
          float kd D系数
          float now 现在量
          float old 过去量
          float want 期望量
输出参数：float bao 打包好的PID数据包
封装等级：0
*******************************************/
//设置PID函数
float PID_set(float kp, float kd, float now, float old, float want){
	  float bao = kp * (want - now) + kd * (now - old);
	  return bao;
}

/*******************************************
函数名：Vel_xy
作用：置xy向速度
作者：Lcs
输入参数：float vy y轴速度
          float vy y轴速度
输出参数：无
封装等级：2
*******************************************/
//置xy向速度
void Vel_xy(float vx, float vy){
	  vx = constrain_float(vx, 10);
	  vy = constrain_float(vy, 10);
	  Position_Control_set_TargetVelocityBodyHeadingXY(vx,vy);
}

/*******************************************
函数名：setRoll_Pitch
作用：置Roll（翻滚角）Pitch（俯仰角）位置
作者：Lcs
输入参数：float pos_roll 设定的俯仰角
          float vy y轴速度
输出参数：bool ok 是否成功到达预定位置
封装等级：3
*******************************************/
//设定翻滚角和俯仰角，并返回翻滚角和俯仰角当前数值
bool setRoll_Pitch(float  pos_roll, float pos_pitch, float* roll, float* pitch){
	  bool ok = false;//设定是否完成的标志变量
	  roll[0] = Attitude_Control_get_Target_Roll();
	  pitch[0] = Attitude_Control_get_Target_Pitch();
	  ok = Attitude_Control_set_Target_RollPitch(pos_roll , pos_pitch);
	  return ok;
}

/*******************************************
函数名：Contorl_Yaw
作用：设置yaw偏航角的速度
作者：Lcs
输入参数：float yawrate 偏航角的速度
          float vy y轴速度
输出参数：bool ok 是否成功到达预定位置
封装等级：3
*******************************************/
//设定翻滚角和俯仰角，并返回翻滚角和俯仰角当前数值
bool Control_Yaw(float yawrate, float yaw_judge, float* yawlook){
	bool ok = false;//设定标志变量
	ok = Attitude_Control_set_Target_YawRate(yawrate);//写入速度
	yawlook[0] = Attitude_Control_get_Target_Yaw();
	if(yawlook[0] == yaw_judge){
		
	}
	return ok;
}
/*******************************************
函数名：Fast_Takeoff
作用：利用积分量加大马力快速起飞
作者：Lcs
输入参数：N
输出参数：N
封装等级：2
*******************************************/

void Fast_Takeoff(){
	//参数表
	float position_base = 60;                                 //初始响应高度（加速高度的判决，由飞机底盘决定）
	int vol_limit_takeoff = 40;                                  //起飞加速度限制，由飞机重量决定
	
	if(ultrasonic <= position_base){
			Position_Control_set_TargetVelocityZ(3 + vol);
			Position_Control_set_TargetVelocityBodyHeadingXY(0,0);
			now_volz = 3 + vol;
			if(vol < vol_limit_takeoff){
				vol++;
			}
		}
		else if(ultrasonic < 70 && ultrasonic > position_base){
			Position_Control_set_TargetVelocityZ(now_volz);
			if(now_volz > 70 - ultrasonic+5){
				now_volz--;
			}
			else if(now_volz < 70 - ultrasonic - 5){
				now_volz++;
			}
			else{
				now_volz = 0;
			}
			
			//Position_Control_set_TargetVelocityZ(constrain_float((70 - ultrasonic) * 0.5, 10));
		}
		else{
			Position_Control_set_TargetVelocityZ(-10.0f);
			Position_Control_set_TargetVelocityBodyHeadingXY(0,0);
 		}
}















