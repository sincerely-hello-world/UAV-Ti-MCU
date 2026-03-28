#include "M37.h"
// #include "Control_Liu.h"
#include "drv_Uart7.h"
#include "drv_Uart2.h"
#include <math.h>
#include <stdint.h>
#include "pid.h"
// #include "drv_I2C1.h"

//            1   2   3   4   5   6   7     //只进行前5个点就降落
float Fx[] = {  0,115,115,115,  0,  0,	0,};  //前向x+
float Fy[] = {  0,  0,-60,-60,  0,  0,	0,};  //左边y+
float Fz[] = {210,210,210,150,150,	0,	0,};  //上方z+  

////            1   2   3   4   5   6   7     //只进行前5个点就降落
//int Fx[] = {  0,150,150,150,  0,  0,	0,};  //前向x+
//int Fy[] = {  0,  0,-60,-60,  0,  0,	0,};  //左边y+
//int Fz[] = {150,150,150,100,100,	0,	0,};  //上方z+  

//static float pos[4];
// static float c2_old_x;
// static float c2_old_y;

float mark_x;
float mark_y;
float mark_z;
float mark_zt;

//extern tPid pidMotorSpeed;
//extern tPid pidMotorAngle;

//extern float t265_x;
//extern float t265_y;
//extern float t265_z; // 引入的t265数据


// extern float aim_x;
// extern float aim_y;
extern int shu_taskid;

extern bool qifeixuke;
extern bool xiajiangxuke;
extern bool target_liscence;
extern bool yaw_liscence;
extern bool chongfeixuke;
extern bool huoyuan;
extern bool chuxianhuoyuan;
extern float yaw_fix,yaw_y;


static int vol = 2;	   // 加速起飞，积分量


static float huan = 10; // 缓冲函数

// uint8_t a = 10;
static void M37_Liu_MainFunc();
static void M37_Liu_enter();
static void M37_Liu_exit();


extern int firepoint;

/*lhq*/
static float now_volz; // 现在V_z的速度
static float now_volx; // 现在V_x速度
static float now_voly; // 现在V_y速度
static double now_yaw;	// 现在yaw角度

const Mode M37_Liu = {
	50,				  // 模式运行频率
	M37_Liu_enter,	  // 进入模式触发函数
	M37_Liu_exit,	  // 离开模式触发函数
	M37_Liu_MainFunc, // 模式主函数（运行频率执行）
};
typedef struct
{
	// 退出模式计数器
	uint16_t exit_mode_counter;

	

	float target_x;
	float target_y;
	float target_z; // 目标坐标坐标

	float target_yaw; // 目标偏航
	
	bool position_lock; // xy位置锁定
	bool height_lock;	// z位置锁定
	bool yaw_lock;		// yaw角锁定

	int zhu_distance; // 柱子间的距离

	int zt; // 任务状态

	int listen; // 聆听任务状态，用于观察Jetson执行到哪一步了

	int count; // 50Hz 1s计数50下，用于摇晃飞机
	
	int delay_count;
	
	bool Flying_flag;
	bool TakeoffFlag;
	bool GoFlag;
	
} MODE_INF;
static MODE_INF *Mode_Inf; // 指针



// 功能函数说明
/************************************************************************************************/

static void Control_Enable_All(); // 所有控制权限开放
static void Control_Disable_All(); //关闭所有控制权
static bool Lock_position(float x, float y); // 位置锁定
static bool Lock_position_same();			 // 原位锁定
static bool Lock_h(float height);			 // 高度锁定
static bool Lock_h_same();					 // 原高锁定
static bool Lock_yaw_same();				 // 原yaw角锁定
static bool Lock_yaw(float aim_yaw);
static bool UnLock_position();				 // 位置解锁
static bool UnLock_h();						 // 高度解锁
static bool UnLock_yaw();					 // yaw解锁

// static bool H_cushion();                                                       //H缓冲函数
static bool Takeoff_Initial(float height);										  // 起飞初始化函数
static bool Takeoff_h(float height, int v);										  // 高度锁定起飞
static bool Takeoff_h_45d(float height, uint8_t sign_x, uint8_t sign_y, float v); // 45度角起飞

static int8_t Fast_Takeoff(float band, float v_begin); // 快速起飞

static bool Move_to_X(float x, float v,uint8_t clear_count);										  // 直线X移动
static uint8_t Move_to_X_coll(float x, float v);								  // 直线X移动协作函数
static bool Move_to_Y(float y, float v,uint8_t clear_count);										  // 直线Y移动
static uint8_t Move_to_Y_coll(float y, float v);								  // 直线Y移动协作函数
static bool Move_to_XYLine(float x, float y, float v);							  // 线性移动到XY某位置
static uint8_t Move_to_XYLine_coll(float x, float y, float v, float detail);	  // 线性移动到XY某位置协作函数
static int8_t Move_to_XYLine_smoth_coll(float x, float y, float v, float detail); // 线性移动到XY某位置平滑函数
static bool Move_Z(float z, float v);											  // 改变Z位置到z;
static bool RotateToYaw(float aim_yaw, float max_yaw_rate);

static char Move_xyz(float x, float y, float z, float v); // 目标XYZ移动

static bool Huanxian(float max_x, float max_y, uint8_t use_taskid, float v); // 环线
static bool Though_Line(float *pos, float v);								 // 穿圈
static bool Though_Line_happy(float *pos_main, float v);					 // 傻瓜式穿圈

static bool Fast_Circle2(float o_x, float o_y, float r, float yaw_v, int8_t taskbreak); // 快速绕圈2 定义圆心,半径,速度

static bool Rotate(float aim_yaw, float v_yaw); // 自旋到规定角度

unsigned char Land(float v, unsigned char flag ); // 降落仅有速度版

static bool Listen(); // 聆听函数，用于聆听任务

static bool Record_position(); // 记录当前位置
static bool Takeoff_h(float height, int v);
static int8_t Takeoff(float x, float y, float z, float v);

void cal_barrier(void);

static bool Map_fly(float v);

static bool barrier_fly(float v);

static bool rescue_fly(float _rescue_x, float _rescue_y, float _rescue_z, float v);

static bool FlyAgain(float v);
static bool Landstop(float v);
static bool delay_loop(int delay_count);

void maintain_hover();
static bool Land_g(float v);
/*************************************************************************************************/

/*static void M37_Liu_enter()函数通过加上static关键字，确保了这个函数只能在本文件中使用，不会与其他文件中的函数名冲突，同时也表明了这个函数是文件私有的，不会被外部模块调用，有助于保持代码的整洁和模块化。*/
static void M37_Liu_enter()
{
	Led_setStatus(LED_status_running1); // 红绿蓝闪烁

	// Initialize Variable Mode_Inf

	Mode_Inf = malloc(sizeof(MODE_INF));
	Mode_Inf->exit_mode_counter = 0;

	Mode_Inf->Flying_flag = false; // 启动锁锁住

	Mode_Inf->target_x = 0.0f;
	Mode_Inf->target_y = 0.0f;
	Mode_Inf->target_z = 0.0f; // 目标位置初始化

	Mode_Inf->target_yaw = 0.0f; // 目标yaw角

	Mode_Inf->zhu_distance = 0.0f; // 绕柱子目标初始化

	Mode_Inf->zt = 0;	  // 初始化任务状态为0
	Mode_Inf->listen = 0; // 初始化Jetson观察状态为0

	Mode_Inf->height_lock = false;
	Mode_Inf->position_lock = false; // 关闭高度锁和位置锁
	Mode_Inf->yaw_lock = false;		 // 关闭偏转角锁

	Mode_Inf->count = 0;
	Mode_Inf->delay_count=0;
	
	Mode_Inf->TakeoffFlag=0; //用于起飞时，标定一次起飞位置
	Mode_Inf->GoFlag = 0;
	Position_Control_Enable(); //?lhq没看懂
}

static void M37_Liu_exit()
{
	Position_Control_Disable();
	Altitude_Control_Disable();
	free(Mode_Inf);
}

extern int listen_ready;

void SetGood(void);
extern int taskid;

#define TAKEOFF_HEIGHT 100.0f
#define HEIGHT 100.0f

#define MAP_SPEED 50.0f
#define FIRE_SPEED 30.0f

//ENU坐标转换为BodyHeading（x为机头朝向（与地面平行），y为朝向机头左方（与地面平行），z为上方）
#define map_ENU2BodyHeading_x( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( enu_x*Yaw_cos + enu_y*Yaw_sin )
#define map_ENU2BodyHeading_y( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( -enu_x*Yaw_sin + enu_y*Yaw_cos )
//BoduHeading转换为ENU坐标
#define map_BodyHeading2ENU_x( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_cos - body_y*Yaw_sin )
#define map_BodyHeading2ENU_y( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_sin + body_y*Yaw_cos )

//extern float detect_x,detect_y,detect_z;
//extern int gofly;
extern int backfly;
//extern int points[50][2];
//extern int point_count;
int mark=1;
int rescue_flag=0;
static int cs=0;
static float now_x=0,now_y=0,now_fire_x=0,now_fire_y=0,delta_bood_x=0,delta_bood_y=0;
static float bood_vx = 20.0f;
static float bood_vx_slow_flag = false;
static int bood_slow_count=0;
static bool now_liscence=false;
static bool success_now_liscence=false;
static bool now_fire_liscence=false;
volatile uint8_t fire_1 = 0;
volatile uint8_t fire_2 = 0;
volatile uint8_t fire_3 = 0;
static uint8_t clear_count = 0;
static int saved_cs=0;
static bool returning_to_route = false;
volatile uint8_t dot_now = 0;
uint8_t dot_back_now = 0;
int Dot_X,Dot_Y = 0;
int Dot_X_OLD , Dot_Y_OLD = 25;

void Uart3_Send( const uint8_t* data, uint16_t length );

extern bool go_back; // 是否返回巡线状态

const float STOP_DIST = 6.0f;       // 完全停止的距离阈值


//            1   2   3   4   5   6   7 
//int Fx[] = {  0,150,150,150,  0,  0,0,};
//int Fy[] = {  0,  0,-60,-60,  0,  0,0,};
//int Fz[] = {150,150,150,100,100,  0,0,};

int cntt = 0;
char ccc[2];
//vector3_float current_pos;

static void M37_Liu_MainFunc()
{
	static uint8_t i=0;
	//current_pos = get_Position();
	++cntt;
	if(cntt >= 25)
	{		
			cntt=0;
			ccc[0] = Mode_Inf->zt +'0';
	}
	
	if (Mode_Inf->Flying_flag == false)  // 起飞确认
	{
			if (fabs(t265_x-0) < 10 && fabs(t265_y-0) < 10 && t265_z < 10 && T265_is_ready == 1 && takeoffflag == 1) // 首次起飞许可
			{
				Mode_Inf->delay_count++;
				if (Mode_Inf->delay_count < 150)
				{
					Led_setSignal(LED_signal_success); // 警告警告，我要起飞了！
				}
				else if (Mode_Inf->delay_count > 150) //  20*150=3000ms后起飞
				{
					Mode_Inf->delay_count = 0; // 清零计数
					Led_setSignal(LED_signal_null);
					Mode_Inf->Flying_flag = true;
					takeoffflag = 0; 
					Mode_Inf->zt = 0;
					Control_Enable_All();
				}
			}//end of 首次起飞许可
//			else if (T265_is_ready == 1 && takeoffflag == 1  && get_Motor_status() == 0 && landflag == 0){// 再飞许可
//					Mode_Inf->Flying_flag = true;
//					takeoffflag = 0; 
//					Mode_Inf->zt = 0;
//			}//end if 再飞许可
	}
	
	if(gofly == 1) { Mode_Inf->zt = 5; }
	
	if(t265_z > 235)  { landflag = 1; }  //安全保险
	if(landflag == 1 || t265_z > 235){ Land(50,1);	Mode_Inf->Flying_flag =false; Mode_Inf->zt=8; }//紧急降落后，不允许再次起飞
	
	else if( Mode_Inf->Flying_flag == true)
	{
		switch (Mode_Inf->zt)
		{   
				case 0: //0 起飞准备状态
				{
					//Takeoff_Initial(TAKEOFF_HEIGHT);
					Lock_position(t265_x,  t265_y); //标定起飞后平面位置！！！ //首次起飞 和 再起飞都需要标定
					set_inFlight_true(); 
					Control_Enable_All(); 					
					Mode_Inf->zt=1;
					break;
				}
				case 1: //1 起飞动作
				{
					if(1 <= Takeoff_h(HEIGHT, 40) ) { Mode_Inf->zt = 2;}
					break;
				}
				case 2: //2 悬停2s 等待跳转
				{
						Mode_Inf->delay_count++;
						if (100 < Mode_Inf->delay_count) // 悬停2s = 2000ms = 20ms * 100
						{
								Mode_Inf->delay_count = 0; // 清零计数
								Mode_Inf->zt = 3;          // 跳转到 任务执行状态
						}
						break;
				}
			  case 3: { //矩形飞行
						
						if (Move_xyz(Fx[i], Fy[i],Fz[i], 25 ) == 2)
						{
								
							  Mode_Inf->zt = 4; // 跳转到delay
								Mode_Inf->delay_count=0;
							  ++i; // 5++ = 6
								if (i >= 6)
								{
										Mode_Inf->zt = 7; // 跳转到降落
								}
						}
						break;
				 }
				case 4:{	
						// 悬停2s  
						Mode_Inf->delay_count++;
						if (100 < Mode_Inf->delay_count) // 悬停2s = 2000ms = 20ms * 1000
						{
								Mode_Inf->delay_count = 0; // 清零计数
								Mode_Inf->zt = 3;          // 跳转到降落
						}
						break;
				}
				case 5:              
				{
						if(gofly == 1)
						{
								Mode_Inf->target_x = detect_x;
								Mode_Inf->target_y = detect_y;
								Mode_Inf->target_z = detect_z;
								Mode_Inf->GoFlag = 1;
								gofly = 0;
						}
						if( Mode_Inf->GoFlag){
								Mode_Inf->delay_count++;
								if( Move_xyz(Mode_Inf->target_x,Mode_Inf->target_y,Mode_Inf->target_z,25) == 2)
								{
									  if ( Mode_Inf->delay_count > 10 ) // 0.2s = 200ms = 20ms * 10
										{
												Mode_Inf->delay_count=0;
												Mode_Inf->GoFlag = 0;  
												Uart2_Send("Gdone\n",6);
										}
								}
								else if ( Mode_Inf->delay_count > 20 ) // 悬停2s = 2000ms = 20ms * 100
								{
										Mode_Inf->delay_count=0;
										Uart2_Send("Gdoing\n",7);
								}
						}
						break;
				}
				case 6:                     //飞回原点
				{
					if(Move_xyz(0,0,50,30) == 2 ){
						Mode_Inf->zt++;
					}
					break;
				}
				case 7:
				{
					if(Land(40,0) == 1){
						Mode_Inf->zt=8;
					}
					break;
				}
				case 8:
				{
					//等待降落完成
					if( get_is_inFlight() == false )
					{
						 Control_Disable_All();
						 Mode_Inf->Flying_flag = 0;
					}
					break;
				}
				
			}//end --- switch
	  maintain_hover();
	}//end --- else if( Mode_Inf->Flying_flag == true)
}



//以下为所需函数
/*******************************************
函数名：Move_xyz
作用：xyz位置移动
作者：Lcs
输入参数：float x 目标端点x,float y 目标端点y.float z目标端点z, float v 移动速度
输出参数：int8_t 返回精确程度
封装等级：3
*******************************************/
static char Move_xyz(float x, float y, float z, float v)
{
		char fanhui = 0;
		float delta_x = x - t265_x;
		float delta_y = y - t265_y;
		float delta_z = z - t265_z;
		float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
		
		UnLock_position();
		UnLock_h();
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		Mode_Inf->target_z = z;

		//这样Z轴调控很稳定
		if (fabsf(delta_z) > 25){
			now_volz = now_volz + sign_f(delta_z)*3;
			if( fabsf(now_volz) > 30){	now_volz = sign_f(delta_z)*30;}
		}
		else{	now_volz = delta_z;	}
		
		
		if (fabsf(delta_x) > 25){
			now_volx = now_volx + sign_f(delta_x)*3;
			if( fabsf(now_volx) > 25){	now_volx = sign_f(delta_x)*25;}
		}
		else{	now_volx = delta_x;	}
		
		if (fabsf(delta_y) > 25){
			now_voly = now_voly + sign_f(delta_y)*3;
			if( fabsf(now_voly) > 25){	now_voly = sign_f(delta_y)*25;}
		}
		else{	now_voly = delta_y;	}
		
		
		
//		if (distance > 55)
//		{
//			// 远距离移动，使用方向向量乘以速度
//			float delta_vx = now_volx - v * delta_x / distance;
//			float delta_vy = now_voly - v * delta_y / distance;
//			float delta_vz = now_volz - v * delta_z / distance;

//			// X方向速度控制
////			if (fabsf(delta_vx) > 22)
////			{
////				now_volx = now_volx - 20 * sign_f(delta_vx) * fabs(delta_x / distance);
////			}
////			else
//			{
//				now_volx = v * delta_x / distance;
//			}

//			// Y方向速度控制
////			if (fabsf(delta_vy) > 22)
////			{
////				now_voly = now_voly - 20 * sign_f(delta_vy) * fabs(delta_y / distance);
////			}
////			else
//			{
//				now_voly = v * delta_y / distance;
//			}
//		}
//		else if (distance > 15)
//		{
//			// 中距离移动，使用比例控制
//			float delta_vx = now_volx - delta_x;
//			float delta_vy = now_voly - delta_y;
//			float delta_vz = now_volz - delta_z;
//			
//			// X方向速度控制
//			if (fabsf(delta_x) > 22)
//			{
//				//now_volx = now_volx - 16 * sign_f(delta_vx) * fabs(delta_x / distance);
//				now_volx = v * delta_x / distance;
//			}
//			else
//			{
//				now_volx = delta_x;
//			}
//			
//			// Y方向速度控制
//			if (fabsf(delta_y) > 22)
//			{
//				//now_voly = now_voly - 16 * sign_f(delta_vy) * fabs(delta_y / distance);
//				now_voly = v * delta_y / distance;
//			}
//			else
//			{
//				now_voly = delta_y;
//			}			
//		}

	if( distance < 13)
	{
		// 到达目标位置
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		Mode_Inf->target_z = z;
		Lock_position(Mode_Inf->target_x,  Mode_Inf->target_y);
		Lock_h(Mode_Inf->target_z);
		
		now_volx = 0; now_voly = 0;
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
		fanhui = 2;
		return fanhui;
	}
	
	// 设置速度
	Position_Control_set_TargetVelocityXY(now_volx, now_voly);
	Position_Control_set_TargetVelocityZ(now_volz);
	return fanhui;
}


void maintain_hover(){
	static unsigned char cnt_time = 0;
	cnt_time++;
	if(cnt_time > 50) //20ms * 25 = 500ms = 0.5s
	{
		cnt_time = 0;
		if( Mode_Inf->position_lock == true &&   Mode_Inf->height_lock == true )
		{
				Uart2_Send("Gdone\n",6); //标明是 空闲状态，回传给串口
		}
	}
	if (Mode_Inf->position_lock == true )
	{
		float delta_x = Mode_Inf->target_x - t265_x;
		float delta_y = Mode_Inf->target_y - t265_y;

		now_volx = delta_x;
		if (fabsf(delta_x) > STOP_DIST)
		{
				if (fabsf(delta_x) < 15)
				{
						now_volx = delta_x;
				}
				else{
					now_volx = sign_f(delta_x)*15;
				}
		}
		now_voly = delta_y;
		if (fabsf(delta_y) > STOP_DIST)
		{
				if (fabsf(delta_y) < 15)
				{
						now_voly = delta_y;
				}
				else{
						now_voly = sign_f(delta_y)*15;
				}
		}
		if(landflag == 1){ now_volx = 0 ; now_voly = 0;	}
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
	}
	
	if (Mode_Inf->height_lock == true )
	{
		float delta_z = Mode_Inf->target_z - t265_z;
		if( delta_z > 15 )  delta_z = 15;
		else if( delta_z < -15  ) delta_z = -15;
		now_volz = delta_z;
		Position_Control_set_TargetVelocityZ(now_volz);
	}
}

/*******************************************
函数名：Move_to_X
作用：x移动_线性
作者：Lcs
输入参数：float x 目标x位置float v 速度
输出参数：bool 是否到位
封装等级：3
*******************************************/
static bool Move_to_X(float x, float v, uint8_t clear)
{
    static uint8_t lx_step = 0;
    float delta_x = x - t265_x;
    float delta_y = Mode_Inf->target_y - t265_y;
    now_voly = delta_y*1.5;
    
    // 常量定义（根据实际调试调整）
    const float SLOW_DOWN_DIST = 30.0f;  // 开始减速的距离阈值
    const float STOP_DIST = 5.0f;        // 完全停止的距离阈值
    const float MIN_SPEED = 3.0f;        // 最小速度限制
    const float MAX_ACCEL = 8.0f;        // 最大加速度限制

    // 方向处理
    if (v * delta_x < 0) {
        v = -v;
    }

    if (clear == 1)
        lx_step = 0;

    if (lx_step == 0) {
        UnLock_position();
        Mode_Inf->target_x = x;
        // 初始速度设为0，避免突变
        now_volx = 0;
        Position_Control_set_TargetVelocityXY(now_volx, now_voly);
        lx_step++;
    } else if (lx_step == 1) {
        float abs_delta = fabs(delta_x);
        float target_speed = 0;
        
        // 根据距离动态调整速度
        if (abs_delta > SLOW_DOWN_DIST) {
            // 远距离保持设定速度
            target_speed = v;
        } else if (abs_delta > STOP_DIST) {
            // 减速区：速度随距离线性减小
            float speed_ratio = abs_delta / SLOW_DOWN_DIST;
            target_speed = sign_f(delta_x) * fmaxf(MIN_SPEED, fabs(v) * speed_ratio);
        } else {
            // 近距离直接锁定位置
            Mode_Inf->target_x = x;
            lx_step = 0;
            Lock_position_same();
            now_volx = 0;  // 确保停止移动
            Position_Control_set_TargetVelocityXY(now_volx, now_voly);
            return true;
        }
        
        // 平滑加速/减速
        float speed_diff = target_speed - now_volx;
        if (fabs(speed_diff) > MAX_ACCEL) {
            now_volx += sign_f(speed_diff) * MAX_ACCEL;
        } else {
            now_volx = target_speed;
        }
        
        Position_Control_set_TargetVelocityXY(now_volx, now_voly);
    }
    return false;
}


/*******************************************
函数名：Move_to_Y
作用：y移动_线性
作者：Lcs
输入参数：float y 目标y位置float v 速度
输出参数：bool 是否到位
封装等级：3
*******************************************/
static bool Move_to_Y(float y, float v,uint8_t clear)
{
	static uint8_t ly_step = 0;
	float delta_y = y - t265_y;
	float delta_x = Mode_Inf->target_x - t265_x;
	now_volx = delta_x*1.5;
	// 常量定义（根据实际调试调整）
    const float SLOW_DOWN_DIST = 30.0f;  // 开始减速的距离阈值
    const float STOP_DIST = 5.0f;        // 完全停止的距离阈值
    const float MIN_SPEED = 2.0f;        // 最小速度限制

    // 方向处理
    if (v * delta_y < 0) {
        v = -v;
    }

    if (clear == 1)
        ly_step = 0;

    if (ly_step == 0) {
        UnLock_position();
        Mode_Inf->target_y = y;
        Position_Control_set_TargetVelocityXY(now_volx, now_voly);
        ly_step++;
    } else if (ly_step == 1) {
        float abs_delta = fabs(delta_y);
        
        // 根据距离动态调整速度
        if (abs_delta > SLOW_DOWN_DIST) {
            // 远距离保持设定速度
            now_voly = v;
        } else if (abs_delta > STOP_DIST) {
            // 减速区：速度随距离线性减小
            float speed_ratio = abs_delta / SLOW_DOWN_DIST;
            now_voly = sign_f(delta_y) * fmaxf(MIN_SPEED, fabs(v) * speed_ratio);
        } else {
            // 近距离直接锁定位置
            Mode_Inf->target_y = y;
            ly_step = 0;
            Lock_position_same();
            now_voly = 0;  // 停止移动
            Position_Control_set_TargetVelocityXY(now_volx, now_voly);
            return true;
        }
        
        Position_Control_set_TargetVelocityXY(now_volx, now_voly);
    }
    return false;
}

/*******************************************
函数名：Record_position
作用：记录当前的位置，便于返回当前位置
作者：Hanqi Lyu
输入参数：着火点的位置fire_x,fire_y,fire_z,移动到着火点的速度v
输出参数：记录位置是否成功 1成功 0失败
封装等级：无
*******************************************/
static bool Record_position()
{
	mark_x = t265_x;
	mark_y = t265_y;
	mark_z = t265_z;
	mark_zt = Mode_Inf->zt;
	return true;
}
extern int downfly;
static bool rescue_fly(float _rescue_x, float _rescue_y, float _rescue_z, float v)
{
    static int rescue_step=0;
    switch(rescue_step)
    {
        case 0:
        {
            uint8_t look = Move_xyz(_rescue_x, _rescue_y, HEIGHT, v);
			if (look == 2)
			{
                if(downfly)
                {
                    downfly=0;
                    rescue_step++;
                }
			}
			break;
            
        }
        case 1:
        {
            uint8_t look = Move_xyz(_rescue_x, _rescue_y, 40, v);
			if (look == 2)
			{
                if(backfly)
                {
                    backfly=0;
                    rescue_step++;
                }
			}
			break;
            
        }
        
            
        
        case 2:
        {
            uint8_t look = Move_xyz(mark_x, mark_y, HEIGHT, v);
			if (look == 2)
			{
                firepoint=0;
                rescue_step=0;
                mark = 1;
				Mode_Inf->zt=mark_zt; 
                return true;                
			}
			break;
            
        }
    }
    return false;
    
}
/*******************************************
函数名：firework_fly                                                    
作用：记录当前的位置，便于返回当前位置
作者：yhc
输入参数：
输出参数：记录位置是否成功 1成功 0失败
封装等级：无
*******************************************/
static bool firework_fly(float x, float y, float v)
{
    static int rescue_step=0;
    switch(rescue_step)
    {
			case 0:
			{
            uint8_t look = Move_xyz(x, y, HEIGHT, v);
			if (look == 2)
			{
				if(downfly)
				{
						downfly=0;
						rescue_step++;
				}
			}
			break;                       

        }
        
    }
	
    return false;
    
}
/*******************************************
函数名：Lock_position
作用：锁定位置，主状态机改变
作者：Lcs
输入参数：float x 目标锁定位置，float y 目标锁定位置y
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool Lock_position(float x, float y)
{
	Mode_Inf->position_lock = true;
	Mode_Inf->target_x = x;
	Mode_Inf->target_y = y;
	return true;
}

/*******************************************
函数名：Lock_position_same
作用：锁定原位置
作者：Lcs
输入参数：
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool Lock_position_same()
{
	Mode_Inf->position_lock = true;
	return true;
}

/*******************************************
函数名：Lock_h
作用：锁定高度，主状态机改变
作者：Lcs
输入参数：float height 锁定高度
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool Lock_h(float height)
{
	Mode_Inf->height_lock = true;
	Mode_Inf->target_z = height;
	return true;
}

/*******************************************
函数名：Lock_h_same
作用：锁定原来位置高度
作者：Lcs
输入参数：
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool Lock_h_same()
{
	Mode_Inf->height_lock = true;
	return true;
}

/*******************************************
函数名：Lock_yaw
作用：锁定aim_yaw角
作者：yhc
输入参数： float aim_yaw
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool Lock_yaw(float aim_yaw)
{
	Mode_Inf->yaw_lock = true;
	Mode_Inf->target_yaw=aim_yaw;
	return true;
}

/*******************************************
函数名：Lock_yaw_same
作用：锁定原来yaw角
作者：Lcs
输入参数：
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool Lock_yaw_same()
{
	Mode_Inf->yaw_lock = true;
	return true;
}

/*******************************************
函数名：UnLock_position
作用：解锁位置，主状态机改变
作者：Lcs
输入参数：N
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool UnLock_position()
{
	Mode_Inf->position_lock = false;
	Position_Control_set_TargetPositionXYRelative(0.f, 0.f);
	return true;
}

/*******************************************
函数名：UnLock_h
作用：解锁高度，主状态机改变
作者：Lcs
输入参数：N
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool UnLock_h()
{
	Mode_Inf->height_lock = false;
	Position_Control_set_TargetPositionZRelative(0.f);
	return true;
}

/*******************************************
函数名：UnLock_yaw
作用：解锁yaw角，主状态机改变
作者：Lcs
输入参数：N
输出参数：bool 返回是否成功
封装等级：4
*******************************************/
static bool UnLock_yaw()
{
	Mode_Inf->yaw_lock = false;
	return true;
}

/*******************************************
函数名：Control_Enable_All
作用：初始化ControlSystem中的各种控制授权
作者：Lcs
输入参数：无
输出参数：无
封装等级：3
*******************************************/
static void Control_Enable_All()
{
	Position_Control_Enable(); // 位置控制 使能
	Altitude_Control_Enable(); // 高度控制 使能
	Enable_Motor();						 // 电机输出 开启
}

static void Control_Disable_All()
{
	Position_Control_Disable(); // 位置控制 关闭
	Altitude_Control_Disable(); // 高度控制 关闭
	Disable_Motor();            // 电机输出 关闭
}




/*******************************************
函数名：Move_to_XYLine
作用：xy平面最短距离移动_线性
作者：Lcs
输入参数：float x 目标x位置 float y 目标y位置 float v 速度
输出参数：bool 是否到位
封装等级：3
*******************************************/
static bool Move_to_XYLine(float x, float y, float v)
{
	static uint8_t l_step = 0;
	float delta_x = x - t265_x;
	float delta_y = y - t265_y;
	float distance = sqrt(delta_x * delta_x + delta_y * delta_y);

	if (l_step == 0)
	{
		UnLock_position();
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		l_step++;
	}
	else if (l_step == 1)
	{
		if (distance > 54)
		{
			float delta_vx = now_volx - v * delta_x / distance;
			float delta_vy = now_voly - v * delta_y / distance;

			if (fabs(delta_vx) > 22)
			{
				now_volx = now_volx - 20 * sign_f(delta_vx) * fabs(delta_x / distance);
			}
			else                                
			{
				now_volx = v * delta_x / distance;
			}

			if (fabs(delta_vy) > 22)
			{
				now_voly = now_voly - 20 * sign_f(delta_vy) * fabs(delta_y / distance);
			}
			else
			{
				now_voly = v * delta_y / distance;
			}
		}

		else if (distance > 7)
		{
			float delta_vx = now_volx - delta_x;
			float delta_vy = now_voly - delta_y;
			if (fabs(delta_vx) > 22)
			{
				now_volx = now_volx - 20 * sign_f(delta_vx) * fabs(delta_x / distance);
			}
			else
			{
				now_volx = delta_x; // 20 * delta_x / distance;
			}
			if (fabsf(now_volx) < 10)
			{
				now_volx = delta_x * 3;
			}
			else if (fabsf(now_volx) < 20)
			{
				now_volx = delta_x * 2;
			}
			if (fabs(delta_vy) > 22)
			{
				now_voly = now_voly - 20 * sign_f(delta_vy) * fabs(delta_y / distance);
			}
			else
			{
				now_voly = delta_y; // 20 * delta_y / distance;
			}
			if (fabsf(now_voly) < 10)
			{
				now_voly = delta_y * 3;
			}
			else if (fabsf(now_voly) < 20)
			{
				now_voly = delta_y * 2;
			}
		}
		else
		{
			Mode_Inf->target_x = x;
			Mode_Inf->target_y = y;
			l_step = 0;
			Lock_position_same();
			Lock_h_same();
			return true;
		}
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
	}

	return false;
}

/*******************************************
函数名：Move_to_XYLine_coll
作用：xy平面最短距离移动_线性 内部协作函数，用于内部对函数的调用
作者：Lcs
输入参数：float x 目标x位置 float y 目标y位置 float v 速度 float detail 要求精度
输出参数：uint8_t 是否到位 1 粗到位 2 细到位
封装等级：3
*******************************************/
static uint8_t Move_to_XYLine_coll(float x, float y, float v, float detail)
{
	uint8_t fanhui = 0;
	static uint8_t l_step = 0;
	float delta_x = x - t265_x;
	float delta_y = y - t265_y;
	float distance = sqrt(delta_x * delta_x + delta_y * delta_y);
	if (distance < 10)
	{
		fanhui = 1;
	}
	else if (distance < 20)
	{
		fanhui = -1;
	}

	if (l_step == 0)
	{
		UnLock_position();
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		l_step++;
	}
	else if (l_step == 1)
	{
		UnLock_position();
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		if (distance > 55)
		{
			float delta_vx = now_volx - v * delta_x / distance;
			float delta_vy = now_voly - v * delta_y / distance;

			if (fabs(delta_vx) > 22)
			{
				now_volx = now_volx - 20 * sign_f(delta_vx) * fabs(delta_x / distance);
			}
			else
			{
				now_volx = v * delta_x / distance;
			}

			if (fabs(delta_vy) > 22)
			{
				now_voly = now_voly - 20 * sign_f(delta_vy) * fabs(delta_y / distance);
			}
			else
			{
				now_voly = v * delta_y / distance;
			}
		}

		else if (distance > detail)
		{
			float delta_vx = now_volx - delta_x;
			float delta_vy = now_voly - delta_y;
			if (fabs(delta_vx) > 22)
			{
				now_volx = now_volx - 20 * sign_f(delta_vx) * fabs(delta_x / distance);
			}
			else
			{
				now_volx = delta_x;
			}
			if (fabsf(now_volx) < 10)
			{
				now_volx = delta_x * 3;
			}
			else if (fabsf(now_volx) < 20)
			{
				now_volx = delta_x * 2;
			}
			if (fabs(delta_vy) > 22)
			{
				now_voly = now_voly - 20 * sign_f(delta_vy) * fabs(delta_y / distance);
			}
			else
			{
				now_voly = delta_y;
			}
			if (fabsf(now_voly) < 10)
			{
				now_voly = delta_y * 3;
			}
			else if (fabsf(now_voly) < 20)
			{
				now_voly = delta_y * 2;
			}
		}
		else
		{
			Mode_Inf->target_x = x;
			Mode_Inf->target_y = y;
			l_step = 0;
			Lock_position_same();
			Lock_h_same();
			fanhui = 2;
		}
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
	}

	return fanhui;
}

/*******************************************
函数名：Move_to_XYLine_smoth_coll
作用：xy平面最短距离移动_线性
作者：Lcs
输入参数：float x 目标x位置 float y 目标y位置 float v 速度 detail定点精细度
输出参数：bool 是否到位
封装等级：3
*******************************************/
static int8_t Move_to_XYLine_smoth_coll(float x, float y, float v, float detail)
{
	static uint8_t l_step = 0;
	float delta_x = x - t265_x;
	float delta_y = y - t265_y;
	float distance = sqrt(delta_x * delta_x + delta_y * delta_y);
	int8_t smothend = 10;

	if (distance < 10)
	{
		smothend = 1;
	}
	else if (distance < 20)
	{
		smothend = 2;
	}

	if (l_step == 0)
	{
		UnLock_position();
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		l_step++;
	}
	else if (l_step == 1)
	{
		UnLock_position();
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		if (distance > 30)
		{
			float delta_vx = now_volx - v * delta_x / distance;
			float delta_vy = now_voly - v * delta_y / distance;

			if (fabs(delta_vx) > 5)
			{
				now_volx = now_volx - 5 * sign_f(delta_vx) * fabs(delta_x / distance);
			}
			else
			{
				now_volx = v * delta_x / distance;
			}

			if (fabs(delta_vy) > 5)
			{
				now_voly = now_voly - 5 * sign_f(delta_vy) * fabs(delta_y / distance);
			}
			else
			{
				now_voly = v * delta_y / distance;
			}
		}

		else if (distance > detail)
		{
			float delta_vx = now_volx - delta_x;
			float delta_vy = now_voly - delta_y;
			if (fabs(delta_vx) > 7)
			{
				now_volx = now_volx - 5 * sign_f(delta_vx) * fabs(delta_x / distance);
			}
			else
			{
				now_volx = delta_x; // 20 * delta_x / distance;
			}
			if (fabsf(now_volx) < 10)
			{
				now_volx = 2.5 * delta_x;
			}
			else if (fabsf(now_volx) < 20)
			{
				now_volx = 2 * delta_x;
			}
			if (fabs(delta_vy) > 7)
			{
				now_voly = now_voly - 5 * sign_f(delta_vy) * fabs(delta_y / distance);
			}
			else
			{
				now_voly = delta_y; // 20 * delta_y / distance;
			}
			if (fabsf(now_voly) < 10)
			{
				now_voly = 2.5 * delta_y;
			}
			else if (fabsf(now_voly) < 20)
			{
				now_voly = 2 * delta_y;
			}
		}
		else
		{
			Mode_Inf->target_x = x;
			Mode_Inf->target_y = y;
			l_step = 0;
			Lock_position_same();
			Lock_h_same();
			smothend = 0;
		}
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
	}

	return smothend;
}


/*******************************************
函数名：Land
作用：降落
作者：Lcs
输入参数：float v 速度, flag = 1,不进行位置精准降落，用于紧急降落/意外情况
输出参数：bool 是否落地
封装等级：3
*******************************************/
unsigned char Land(float v, unsigned char flag )
{
	if(flag == 1) //不进行降落坐标校准，直接降落
	{
		UnLock_position();
		now_volx = 0 ; now_voly = 0 ;
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
	}
	else{ //如果对降落坐标精度有要求，锁定降落位置
		Lock_position(Mode_Inf->target_x,  Mode_Inf->target_y);
	}
	
	UnLock_h();//解锁 高度输出

	if(  get_is_inFlight() == false ) // || ((fabsf(get_VelocityENU().z)) < 6 && t265_z < 13.0f)  
	{
		set_inFlight_false();
		Control_Disable_All();
		now_volx = 0 ; now_voly = 0 ; now_volz = 0 ;
		return true;
	}

	if (t265_z > 25){	now_volz = -60;	}//根据需要，-50很快，不要小于-60会摔坏。
	else{ now_volz = -50;} 

	Position_Control_set_TargetVelocityZ(now_volz);
	return false;
}


static bool Landstop(float v)
{
	UnLock_h();
	UnLock_position();
	Mode_Inf->target_z = -1;

	now_volx = (Mode_Inf->target_x - t265_x) * 2;
	now_voly = (Mode_Inf->target_y - t265_y) * 2;

	if (t265_z > 25.0f)
	{
		if (now_volz > -v)
		{
			vol = -2;
			now_volz = now_volz + vol;
		}

		Position_Control_set_TargetVelocityZ(now_volz);
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
	}
	else if (t265_z > -20.0f)
	{
		if (now_volz < -20.0f)
		{
			now_volz = now_volz + 1;
		}
		else
		{
			now_volz = -20;
		}
		Position_Control_set_TargetVelocityZ(now_volz);
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);
		if ((fabsf(get_VelocityENU().z)) < 5 && t265_z < 20.0f)
		{
			set_inFlight_false();
            Mode_Inf->zt++;
			// PWM_PullDownAll();可以直接拉低
		}
	}

	return false;
}

/*******************************************
函数名：Takeoff_Initial
作用：起飞初始化函数
作者：Lcs
输入参数：float height 需要的高度
输出参数：bool 是否起飞到预定要微调的位置
封装等级：3
*******************************************/
static bool Takeoff_Initial(float height)
{
	static int t_step = 0;
	switch (t_step)
	{
	case 0:
	{
		// Position_Control_Takeoff_HeightRelative(get_Position().z + height);
		set_inFlight_true();
		t_step++;
		break;
	}
	case 1:
	{
		if (get_is_inFlight() == true)
		{
			Mode_Inf->zt++;
			t_step = 0;
		}
		break;
	}
	}
	return true;
}
static bool FlyAgain(float v)
{
    static int t_step = 0;
    switch (t_step)
    {
    case 0:
    {
        /* 关键修改点1：清零所有速度控制变量 */
        now_volx = 0;
        now_voly = 0;
        now_volz = 0;  // 清除Landstop设置的下降速度
        
        /* 关键修改点2：重置控制器目标速度（与变量同步） */
        Position_Control_set_TargetVelocityXY(0, 0);
        Position_Control_set_TargetVelocityZ(0);
        
        /* 保持原有逻辑 */
        set_inFlight_true();
        t_step++;
        break;
    }
    case 1:
    {
        /* 保持原有状态切换逻辑 */
        if (get_is_inFlight() == true)
        {
            Mode_Inf->zt++;
            t_step = 0;
        }
        break;
    }
    }
    return true;
}

/*******************************************
函数名：Listen
作用：用于聆听Jetson任务状态
作者：Lcs
输入参数：
输出参数：bool 是否聆听
封装等级：1
*******************************************/
static bool Listen()
{
	Mode_Inf->listen = shu_taskid;
	return true;
}

/*******************************************
函数名：Takeoff_h_45d
作用：仅由高度控制的45度角起飞
作者：Lcs
输入参数：float height 你所想要的高度 uint8_t sign_x 起飞时的x向坐标 uint8_t sign_y 起飞时的y向坐标 v 起飞速度
输出参数：bool 是否起飞到预定要微调的位置
封装等级：3
*******************************************/
static bool Takeoff_h_45d(float height, uint8_t sign_x, uint8_t sign_y, float v){
														  //可能要加unlock h/position
	static uint8_t h_step = 0;
	float delta_x = t265_x - Mode_Inf->target_x;
	float delta_y = t265_y - Mode_Inf->target_y;
	float distance = sqrt(Mode_Inf->target_x * Mode_Inf->target_x + Mode_Inf->target_y * Mode_Inf->target_y + Mode_Inf->target_z * Mode_Inf->target_z);
	if(h_step == 0){
		Mode_Inf->target_x = height * (sign_x / sqrt(sign_x * sign_x + sign_y * sign_y));
	    Mode_Inf->target_y = height * (sign_y / sqrt(sign_x * sign_x + sign_y * sign_y));
	    Mode_Inf->target_z = height;
		h_step++;
	}
	else if(h_step == 1){
		if(t265_z < 10){
			now_volx = (0 - t265_x) * 1.5;
	    now_voly = (0 - t265_y) * 1.5;
		  Position_Control_set_TargetVelocityZ(40);
		  Position_Control_set_TargetVelocityXY(now_volx, now_voly);
		  now_volz = 40;
	  }
		else{
			if(t265_z < height - 30){
				float delta_vx = now_volx - v * delta_x / distance;
				float delta_vy = now_voly - v * delta_y / distance;
				if(fabs(delta_vx) > 7){
				  now_volx = now_volx - 10 * sign_f(delta_vx) * fabs(delta_x / distance);
			  }
			  else{
				  now_volx = v * delta_x / distance;
			  }
			
			  if(fabs(delta_vy) > 7){
				  now_voly = now_voly - 10 * sign_f(delta_vy) * fabs(delta_y / distance);
			  }
			  else{
				  now_voly = v * delta_y / distance;
			  }
			}
			else if(t265_z < height - 10){
				float delta_vx = now_volx - 20 * delta_x / distance;
				float delta_vy = now_voly - 20 * delta_y / distance;
				if(fabs(delta_vx) > 7){
				  now_volx = now_volx - 10 * sign_f(delta_vx) * fabs(delta_x / distance);
			  }
			  else{
				  now_volx = v * delta_x / distance;
			  }
			
			  if(fabs(delta_vy) > 7){
				  now_voly = now_voly - 10 * sign_f(delta_vy) * fabs(delta_y / distance);
			  }
			  else{
				  now_voly = v * delta_y / distance;
			  }
			}
			else{
				Lock_h_same();
				Lock_position_same();
				h_step = 0;
				Mode_Inf->zt++;
			}
		}
	}
	
	
	
	return true;
}
/*******************************************
函数名：Rotate
作用：自旋到固定角度
作者：Lcs
输入参数：float aim_yaw 目标角度 float v_yaw 角速度
输出参数：bool 是否到位
封装等级：3
*******************************************/
static bool Rotate(float aim_yaw, float v_yaw){
		Mode_Inf->target_yaw = aim_yaw;
		
	
	    float now_yaw = Quaternion_getYaw(get_Airframe_attitude());
	    
		
		float delta_yaw = Mode_Inf->target_yaw - now_yaw;
	
	    if(fabsf(aim_yaw - now_yaw) > 0.02){
			Attitude_Control_set_Target_Yaw(aim_yaw);
			return false;
		}
		else{
			Attitude_Control_set_Target_Yaw(aim_yaw);
			yaw_liscence=false;
			Mode_Inf->target_yaw = aim_yaw;
			Lock_position_same();
			Lock_h_same();
			
			return true;
			//Lock_yaw_same();
			
			//Mode_Inf->zt++;
		}
	
}

/*******************************************
函数名：Move_Z
作用：Z位置改变
作者：Lcs
输入参数：float z 目标z位置 float v 速度
输出参数：bool 是否到位
封装等级：3
*******************************************/
static bool Move_Z(float z, float v){
	static int8_t h_step = 0;
	float delta_z = z - t265_z;
	if((delta_z) * v < 0){
		v = -v;
	}
	now_volx = (Mode_Inf->target_x - t265_x) * 5;
	now_voly = (Mode_Inf->target_y - t265_y) * 5;
	if(h_step == 0){
		UnLock_position();
		UnLock_h();
		Mode_Inf->target_z = z;
		Lock_position_same();
		h_step++;
	}
	else if(h_step == 1){
		if(fabs(delta_z) > 20){
			float delta_vz = now_volz - v ;
			if(fabs(delta_vz) > 5){
				now_volz = now_volz - 4 * sign_f(delta_vz);
			}
			else{
				now_volz = v;
			}
		}
		else if(fabs(delta_z) > 10){
				now_volz = delta_z;
		}
		else{
			Mode_Inf->target_z = z;
			h_step = 0;
			Lock_position_same();
			Lock_h(z);
			Mode_Inf->zt++;
		}
		Position_Control_set_TargetVelocityZ(now_volz);
		
	}
	Position_Control_set_TargetPositionXY(now_volx, now_voly);
	return true;
}


/*******************************************
函数名：RotateToYaw
作用：改变yaw角度
作者：yhc
输入参数：float aim_yaw目标角度, float max_yaw_rate最大角速度
输出参数：bool 是否到位
封装等级：3
*******************************************/
static bool RotateToYaw(float aim_yaw, float max_yaw_rate)
{
    float now_yaw = Quaternion_getYaw(get_Airframe_attitude());
    // 保证yaw在0~2π范围
    if(now_yaw < 0) now_yaw += 2 * Pi;
    if(aim_yaw < 0) aim_yaw += 2 * Pi;

    float delta_yaw = aim_yaw - now_yaw;
    // 处理环绕，保证在-π~π
    while(delta_yaw > Pi) delta_yaw -= 2 * Pi;
    while(delta_yaw < -Pi) delta_yaw += 2 * Pi;

    const float yaw_threshold = 0.1f; // 约5度

    if(fabsf(delta_yaw) > yaw_threshold) {
        // 旋转方向和速度
        float set_rate = (delta_yaw > 0 ? 1.0f : -1.0f) * max_yaw_rate;
        Attitude_Control_set_Target_YawRate(set_rate);
        return false; // 还未到达目标
    } 
	else{
        // 到达目标角度，切换为角度锁定
		yaw_liscence=false;
		Attitude_Control_set_Target_Yaw(aim_yaw);
        Lock_position_same();
        Lock_h_same();
        return true; // 已到达目标
    }
}

/*******************************************
函数名：land_45d       （未完成版）
作用：45度角降落
作者：yhc
输入参数：float height降落时的高度uint8_t sign_x 降落时的x向坐标 uint8_t sign_y 降落时的y向坐标 v 起飞速度
输出参数：bool 是否降落到预定要微调的位置
封装等级：3
*******************************************/
static bool land_45d(float height, uint8_t sign_x, uint8_t sign_y, float v){
														  //可能要加unlock h/position
	static uint8_t h_step = 0;
	float delta_x = t265_x - Mode_Inf->target_x;
	float delta_y = t265_y - Mode_Inf->target_y;
	float distance = sqrt(Mode_Inf->target_x * Mode_Inf->target_x + Mode_Inf->target_y * Mode_Inf->target_y + Mode_Inf->target_z * Mode_Inf->target_z);
	if(h_step == 0){
		Mode_Inf->target_x = height * (sign_x / sqrt(sign_x * sign_x + sign_y * sign_y));
	    Mode_Inf->target_y = height * (sign_y / sqrt(sign_x * sign_x + sign_y * sign_y));
	    Mode_Inf->target_z = height;
		h_step++;
	}
	else if(h_step == 1){
		if(t265_z < 10){
			now_volx = (0 - t265_x) * 1.5;
			now_voly = (0 - t265_y) * 1.5;
		  Position_Control_set_TargetVelocityZ(40);
		  Position_Control_set_TargetVelocityXY(now_volx, now_voly);
		  now_volz = 40;
	  }
		else{
			if(t265_z < height - 30){
				float delta_vx = now_volx - v * delta_x / distance;
				float delta_vy = now_voly - v * delta_y / distance;
				if(fabs(delta_vx) > 7){
				  now_volx = now_volx - 10 * sign_f(delta_vx) * fabs(delta_x / distance);
			  }
			  else{
				  now_volx = v * delta_x / distance;
			  }
			
			  if(fabs(delta_vy) > 7){
				  now_voly = now_voly - 10 * sign_f(delta_vy) * fabs(delta_y / distance);
			  }
			  else{
				  now_voly = v * delta_y / distance;
			  }
			}
			else if(t265_z < height - 10){
				float delta_vx = now_volx - 20 * delta_x / distance;
				float delta_vy = now_voly - 20 * delta_y / distance;
				if(fabs(delta_vx) > 7){
				  now_volx = now_volx - 10 * sign_f(delta_vx) * fabs(delta_x / distance);
			  }
			  else{
				  now_volx = v * delta_x / distance;
			  }
			
			  if(fabs(delta_vy) > 7){
				  now_voly = now_voly - 10 * sign_f(delta_vy) * fabs(delta_y / distance);
			  }
			  else{
				  now_voly = v * delta_y / distance;
			  }
			}
			else{
				Lock_h_same();
				Lock_position_same();
				h_step = 0;
				//Mode_Inf->zt++;
			}
		}
	}
	
	
	
	return true;
}

/*******************************************
函数名：delay_loop
作用：延时停留
作者：yhc
输入参数：int delay_count 延时时间
输出参数：bool 是否延时结束
封装等级：3
*******************************************/
int delay_counter = 0;

static bool delay_loop(int delay_count) {
    if (delay_counter < delay_count) {
        delay_counter++;
        return false; // 延时未结束
    } else if (delay_counter == delay_count) {
        delay_counter = 0; // 可选：如需重复延时则清零
        return true; // 延时结束
    }
    return false;
}


static int8_t Takeoff(float x, float y, float z, float v)
{
	// Listen();
	int8_t xyz_fanhui = -100;
	float delta_x = x - t265_x;
	float delta_y = y - t265_y;
	float delta_z = z - t265_z;
	static int8_t xyz_step = 0;

	if (delta_x < 20 && delta_y < 20 && delta_z < 5)
	{
		xyz_fanhui = -1;
	}
	else if (delta_x < 10 && delta_y < 10 && delta_z < 5)
	{
		xyz_fanhui = 1;
	}

	if (xyz_step == 0)
	{
		UnLock_h();
		UnLock_position();
		Mode_Inf->target_x = x;
		Mode_Inf->target_y = y;
		Mode_Inf->target_z = z;
		xyz_step++;
	}
	else if (xyz_step == 1)
	{
		UnLock_h();
		UnLock_position();

		/*//lhq:fabsf处理float类型的取绝对值*/
		if (fabsf(delta_x) > 0.7 * v)
		{ // 0.6介稳飞行
			float delta_vx = now_volx - v * sign_f(delta_x);

			if (fabs(delta_vx) > 33)
			{
				now_volx = now_volx - 30 * sign_f(delta_vx);
			}
			else
			{
				now_volx = v * sign_f(delta_x);
			}
		}
		else
		{
			float delta_vx = now_volx - delta_x;

			if (fabs(delta_vx) > 33)
			{
				now_volx = now_volx - 30 * sign_f(delta_vx);
			}
			else
			{
				now_volx = delta_x;
			}
			if (now_volx < 10)
			{
				now_volx = 2 * delta_x;
			}
			else if (now_volx < 20)
			{
				now_volx = 1.5 * delta_x;
			}
		}
		if (fabsf(delta_y) > 0.7 * v)
		{ // 0.6介稳飞行

			float delta_vy = now_voly - v * sign_f(delta_y);

			if (fabs(delta_vy) > 33)
			{
				now_voly = now_voly - 30 * sign_f(delta_vy);
			}
			else
			{
				now_voly = v * sign_f(delta_y);
			}
		}
		else
		{
			float delta_vy = now_voly - delta_y;

			if (fabs(delta_vy) > 33)
			{
				now_voly = now_voly - 30 * sign_f(delta_vy);
			}
			else
			{
				now_voly = delta_y;
			}
			if (now_voly < 10)
			{
				now_voly = 2 * delta_y;
			}
			else if (now_voly < 20)
			{
				now_voly = 1.5 * delta_y;
			}
		}
		if (fabsf(delta_z) > 35)
		{
			if (v > 80)
			{
				v = 80;
			}
			float delta_vz = now_volz - v * sign_f(delta_z);

			if (fabs(delta_vz) > 4)
			{
				now_volz = now_volz - 4 * sign_f(delta_vz);
			}
			else
			{
				now_volz = v * sign_f(delta_z);
			}
		}
		else
		{
			float delta_vz = now_volz - delta_z;

			if (fabs(delta_vz) > 4)
			{
				now_volz = now_volz - 4 * sign_f(delta_vz);
			}
			else
			{
				now_volz = delta_z; // lhq 原来  now_volz = delta_z;      ->0.4 *
			}

			if (now_volz < 10)
			{
				now_volz = 1.8 * delta_z;     //原来是2
			}
			else if (now_volz < 20)
			{
				now_volz = 1.3 * delta_z;
			} // lhq6/13
		}

		Position_Control_set_TargetVelocityZ(now_volz);
		Position_Control_set_TargetVelocityXY(now_volx, now_voly);

		if (fabsf(delta_x) < 4 && fabsf(delta_y) < 4 && fabsf(delta_z) < 5) // 445
		{
			xyz_step++;
		}
	}
	else if (xyz_step == 2)
	{
		Lock_position(x, y);
		Lock_h(z);
		xyz_step = 0;
		xyz_fanhui = 2;
		// Mode_Inf->zt++;
	}

	return xyz_fanhui;
}

static bool Takeoff_h(float height, int v)
{
			float position_base = 10;                                 //初始响应高度（加速高度的判决，由飞机底盘决定）
	    int vol_limit_takeoff = v;                                  //起飞加速度限制，由飞机重量决定

//	    Mode_Inf->target_x = 0;
//	    Mode_Inf->target_y = 0;
//			Lock_position(Mode_Inf->target_x,  Mode_Inf->target_y);
	
//	    now_volx = (Mode_Inf->target_x - t265_x) * 1.5;
//	    now_voly = (Mode_Inf->target_y - t265_y) * 1.5;
	
      if(t265_z <= position_base){
				now_volz = 60; 
		    Position_Control_set_TargetVelocityZ(now_volz);
	    }///
	    else if(t265_z < height - 4 * position_base){
		    Position_Control_set_TargetVelocityZ( now_volz + 2);
		    if(now_volz < vol_limit_takeoff){now_volz = now_volz + 2;}
		    else{now_volz = now_volz - 2;}
	    }///
	    else if(t265_z < height - 5){
		    float now_hd = height - t265_z;
				now_volz = 1.5 * now_hd ;
				Position_Control_set_TargetVelocityZ(now_volz);
	    }///
			else{
				Lock_h(height);//锁定高度
				return true;
			}///
			return false;
}