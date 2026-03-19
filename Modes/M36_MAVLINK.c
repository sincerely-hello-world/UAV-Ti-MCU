#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>
#include "M36_MAVLINK.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "drv_Uart2.h"
#include "drv_Uart7.h"
#include "drv_Uart0.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"

#include "sysctl.h"
#include "hw_memmap.h"
#include "pin_map.h"
#include "gpio.h"
#include "pwm.h"

#include "drv_Uart2.h"

#define Pi 3.14

static inline void Buzzer(bool on)                           //蜂鸣器是否工作
{
	if (on)
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
	else
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
}
extern int Buzzer_flag;
bool guided_enabled = false;
bool pos_enabled = true;
static void M36_MAVLINK_MainFunc();
static void M36_MAVLINK_enter();
static void M36_MAVLINK_exit();
static vector3_float Position;
static vector3_float VelocityENU;
float sum1 = 0, sum2 = 0;

//对地k210
extern int k210_pos_x, k210_pos_y, k210_state;
extern bool k210_rcflag;
//对杆子k210
extern uint8_t k210_2_rc1;
extern bool k210_2_rcflag;
extern uint8_t mnist_num[4];
extern uint8_t mnist_count;
void k210_2_init(){
	mnist_num[0] = 0;
	mnist_num[1] = 0;
	mnist_num[2] = 0;
	mnist_num[3] = 0;
	mnist_count = 0;
	k210_2_rcflag =0;
	k210_2_rcflag = false;
}
uint8_t get_k210_2_num(){
	uint8_t max = 0,ind = 0;
	int i;
	for(i=0;i<4;i++){
		if (mnist_num[i]>max){
			max=mnist_num[i];
			ind = i+1;
		}
	}
	return ind;
}

void K210_SendCMD(uint8_t cmd)
{
	uint8_t send_array[3] = { 0x73, cmd, 0x3c };
	Uart2_Send(send_array, 3);
}
void K210_2_SendCMD(uint8_t cmd)
{
	uint8_t send_array[3] = { 0x73, cmd, 0x3c };
	Uart0_Send(send_array, 3);
}

const Mode M36_MAVLINK =
{
	50,  //mode frequency
	M36_MAVLINK_enter,  //enter
	M36_MAVLINK_exit,	//exit
	M36_MAVLINK_MainFunc,	//mode main func
};

typedef struct
{
	//退出模式计数器
	uint16_t exit_mode_counter;

} MODE_INF;
static MODE_INF* Mode_Inf;

static void M36_MAVLINK_enter()
{
	static int num = 5;
	Led_setStatus(LED_status_running1);

	//初始化模式变量
	Mode_Inf = malloc(sizeof(MODE_INF));
	Mode_Inf->exit_mode_counter = 0;
	//    Altitude_Control_Enable();
	Position_Control_Enable();
	//	Attitude_Control_Enable();
}

static void M36_MAVLINK_exit()
{
	Altitude_Control_Disable();
	Attitude_Control_Disable();
	free(Mode_Inf);
}

extern float receive_x, receive_y, receive_z, receive_QR, receive_QRType, receive_QRHeight, receive_QRx;
extern float receivek210_x, receivek210_y, receivek210_z, receivek210_vx, receivek210_vy, receivek210_vz, receivek210_yaw;
extern bool receive_camera_flag;
extern vector3_float pos_vision;
//cwr 1 cwg2 ccwr3 ccwg4



float AfterTurnYaw = 0;

//封装操作相关↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	//封装的操作所需的姿态数据
uint32_t time_count = 0;

vector3_float TgtPos;
uint8_t MaxPosLimit;

vector3_float current_pos;
float target_yaw;
float turnyaw_errx, turnyaw_erry;
float CircleRadiu, CirCleYaw_err, CirCleDelayYaw_err;
#define FlightR 10 //飞机半径
//Height为起飞时飞机自身的高度，Height_t265为起飞时t265的高度
extern float Height, Height_t265;
extern int ultra_dis;//对前超声 uart7
float Height = 0, Height_t265 = 0, Height_asonic = 0;
bool IsDirUp = false;
float tgt_height = 0, Lock_Height = 0;

#define get_height_self() (get_Position().z-Height)
#define get_height_t265() (get_position_t265().z-Height_t265)
#define get_height_asonic() (ultrasonic-Height_asonic)

//定高方式
#define LockHeightType_asonic 0
#define LockHeightType_t265 1
#define LockHeightType_self 2
uint8_t LockHeightType = LockHeightType_t265;

//超声高度
extern float ultrasonic;
//是否执行封装的操作
bool IsInflightOrder = false;
//是否软件锁定高度
bool IsLockHeight = false;
//封装的操作的类型
enum MyFlightOrder {
	Exit = 1,
	SetPositionWithCheck,
	TurnSelfCenter,
	Circlrmode,
	LandByUltrasonic,
	TakeOffWithInit,
	SetHeightWithCheck,
	TakeOff45degree
}OrderMode = Exit;

//封装的函数
	//绝对坐标系下限幅定点飞行，到达+-10cm后脱出
#define EXITThreshold 10 
#define get_position_t265() pos_vision
void OrderSetPositionWithCheck(float tgtx, float tgty, uint8_t limit);
void OrderSetPositionWithCheckRelative(float tgtx, float tgty, uint8_t limit);
void OrderSetPositionWithCheckRelative_BodyHeading(float tgtx, float tgty, uint8_t limit);
void OrderTurnSelfCenter(float tgtyaw, float yaw_rate);
void OrderCirclrmode(float tgtyaw, float yaw_rate, float radiu, float yaw_err);
void OrderLandByUltrasonic();
void OrderTakeOffWithInit(float myHeight);
void CtrlLockHeight(float myHeight);
void CtrlunLockHeight();
void OrderSetHeightWithCheck(float myheight, float myvelosity);
void CtrlSerLockHeightType(uint8_t type);
void OrderTakeOff45degree(float myHeight);
void ultra_order(void);
//封装操作相关end↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

static void M36_MAVLINK_MainFunc()
{
	ultra_order();
	static uint8_t qr_circle_mode = 0;
	//遥控器部分
	const Receiver* rc = get_current_Receiver();
	if (rc->available == false)
	{
		//接收机不可用
		//降落
		Position_Control_set_XYLock();
		Position_Control_set_TargetVelocityZ(-50);
		return;
	}
	float throttle_stick = rc->data[0];
	float yaw_stick = rc->data[1];
	float pitch_stick = rc->data[2];
	float roll_stick = rc->data[3];
	float mode_stick = rc->data[5];
	bool 	position_ready = get_Position_Measurement_System_Status() == Measurement_System_Status_Ready;
	/*判断退出模式 内八退出自控模式*/
	if (throttle_stick < 5 && yaw_stick < 5 && pitch_stick < 5 && roll_stick > 95)
	{
		if (++Mode_Inf->exit_mode_counter >= 50)
		{
			change_Mode(1);
			return;
		}
	}
	else
		Mode_Inf->exit_mode_counter = 0;
	/*判断退出模式*/

	//判断摇杆是否在中间
	bool sticks_in_neutral =
		in_symmetry_range_offset_float(throttle_stick, 5, 50) && \
		in_symmetry_range_offset_float(yaw_stick, 5, 50) && \
		in_symmetry_range_offset_float(pitch_stick, 5, 50) && \
		in_symmetry_range_offset_float(roll_stick, 5, 50);



	//MODE按钮在低位且摇杆在中位 未开遥控器也会进入
	if (mode_stick < 30 && sticks_in_neutral)
	{

		//打开水平位置控制
		Position_Control_Enable();
		//用飞控校准 屏蔽树莓派指令
		guided_enabled = false;
		
		//待办
		static int OKcount = 0;


		//软件锁高度功能
		if (IsLockHeight) {
			static uint8_t LockHeightDivCount = 0;
			if (++LockHeightDivCount > 10) {
				LockHeightDivCount = 0;
				switch (LockHeightType) {
				case LockHeightType_asonic: {
					Position_Control_set_TargetPositionZRelative(constrain_float(Lock_Height - get_height_asonic(), 10));
					break;
				}
				case LockHeightType_t265: {
					Position_Control_set_TargetPositionZRelative(constrain_float(Lock_Height - get_height_t265(), 10));
					break;
				}
				case LockHeightType_self: {
					Position_Control_set_TargetPositionZRelative(constrain_float(Lock_Height - get_height_self(), 10));
					break;
				}
				}
			}
		}


		//判断是否在执行功能函数 若否则执行主流程状态机
		if (!IsInflightOrder) {
			//主流程状态机 执行编写的任务 
			static int Modeindex = 0;
			switch (Modeindex)
			{
			case 0:
			{
				/*if (vision_present)
				{
					//K210_2_SendCMD(5);
					//k210_2_init();
					OrderTakeOffWithInit(80.f);
					//					OrderTakeOff45degree(110.f);
					//Modeindex++;
										Modeindex  = 90;//测试用
				}*/
				OrderTakeOffWithInit(80.f);
				Modeindex  = 90;//测试用
				
				break;
			}
			
			//测试用状态机
			{
			case 90:
			{
				if (get_Altitude_ControlMode() == Position_ControlMode_Position)
				{
					//Modeindex++;//目前为降落用
					//				Modeindex = 94;//对杆测试
									Modeindex = 97;//广播体操
					//				Modeindex = 198;//降落程序
				}
				break;
			}
			//降落用
			case 91:
			{
				if (modedelay_s(5)) {
					Modeindex++;
				}
				break;
			}
			case 92:
			{

				Position_Control_set_TargetVelocityBodyHeadingXY(0.f, 40.f);
				OrderLandByUltrasonic();
				Modeindex++;
				break;
			}
			case 93:
			{
				break;
			}
			//对杆测试
			case 94:
			{
				Led_setStatus(LED_status_error);
				if (receive_camera_flag) {
					receive_camera_flag = false;
					if (!receive_z) {
						Attitude_Control_set_Target_YawRate(constrain_float(receive_x / 5, 30) / 210);
					}
				}

				if (modedelay_s(5)) {
					Attitude_Control_set_YawLock();
					Led_setStatus(LED_status_running2);
					Modeindex++;
				}
				break;
			}
//			#define video_err_x 100
//			case 95:
//			{
//				if (receive_camera_flag) {
//					receive_camera_flag = false;
//					if (!receive_z) {
//						if (receive_y - video_err_x < 0) {
//							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y - video_err_x), 60), constrain_float(receive_x / 10, 20));
//						}
//						else {
//							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y - video_err_x) / 4, 20), constrain_float(receive_x / 10, 20));
//						}
//					}
//					else {
//						Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
//					}
//				}
//				break;
//			}
			
			case 95:
			{
				if (modedelay_s(3)) {
					OrderSetHeightWithCheck(80.f, -20.f);
					Modeindex ++;
				}
				break;
			}
			
			case 97:
			{
				if (modedelay_s(1)) {
					OrderSetHeightWithCheck(80.f, -20.f);
					Modeindex ++;
				}
				break;
			}
			case 98:
			{
				if (modedelay_s(3)) {
                    OrderSetPositionWithCheck(100.f, 0.f, 80);
					Modeindex ++;
				}
				break;
			}  
			case 99:
			{
				if (modedelay_s(3)) {
                    OrderSetPositionWithCheck(200.f, -50.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 100:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(250.f, -50.f, 80);
					Modeindex = 198;
				}
				break;
			}  
  			case 101:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(250.f, -100.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 102:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(200.f, -100.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 103:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(200.f, -150.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 104:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(250.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 105:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(250.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 106:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(200.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 107:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(150.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 108:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(100.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 109:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(50.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 110:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(0.f, -250.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 111:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(0.f, -300.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 112:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(0.f, -350.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 113:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(0.f, -350.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 114:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(50.f, -300.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 115:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(50.f, -250.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 116:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(50.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 117:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(100.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 118:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(150.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 119:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(200.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 120:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(250.f, -200.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 121:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(250.f, -250.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 122:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(200.f, -250.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 123:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(150.f, -250.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 124:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(150.f, -300.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 125:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(200.f, -300.f, 80);
					Modeindex ++;
				}
				break;
			}  
  			case 126:
			{
				if (modedelay_s(1)) {
                    OrderSetPositionWithCheck(250.f, -300.f, 80);
					Modeindex =198;
				}
				break;
			}  
              
//			case 97:
//			{
//				OrderSetPositionWithCheck(100.f, 0.f, 80);
//				Modeindex++;
//				break;
//			}
//			case 98:
//			{
//				if (modedelay_s(3)) {
//					OrderSetPositionWithCheck(100.f, 100.f, 80);
//					Modeindex++;
//				}
//				break;
//			}
//			case 99:
//			{
//				if (modedelay_s(3)) {
//					OrderTurnSelfCenter(-Pi / 2, -2 * Pi / 20);
//					Modeindex = 104;
//				}
//				break;
//			}
//			case 100:
//			{
//				if (modedelay_s(3)) {
//					Led_setStatus(LED_status_error);
//					OrderSetPositionWithCheck(100.f, 200.f, 80);
//					Modeindex++;
//				}
//				break;
//			}
//			case 101:
//			{
//				Led_setStatus(LED_status_running1);
//				if (modedelay_s(5)) {
//					OrderCirclrmode(-Pi / 2, 2 * Pi / 20, 100.f, 0.f);
//					Modeindex = 104;
//				}
//				break;
//			}
//			case 102:
//			{

//				if (modedelay_s(3)) {
//					Position_Control_set_TargetPositionZRelative(30.f);
//					//Position_Control_move_PositionXYRelative( 100.f , 0.f );
//					Modeindex++;
//				}
//				break;
//			}
//			case 103:
//			{
//				if (modedelay_s(3)) {
//					CtrlLockHeight(150.f);
//					Modeindex++;
//				}
//				break;
//			}
//			case 104:
//			{
//				if (modedelay_s(3)) {
//					OrderSetPositionWithCheck(0.f, 0.f, 50);
//					Modeindex++;
//				}
//				break;
//			}
//			case 105:
//			{
//				if (modedelay_s(3)) {
//					//						Position_Control_set_TargetVelocityZ(-20.f);
//					OrderSetHeightWithCheck(120.f, 20.f);
//					//						CtrlLockHeight(100.f);
//					Modeindex++;
//				}
//				break;
//			}
//			case 106:
//			{
//				if (modedelay_s(3)) {
//					Position_Control_set_ZLock();
//					Modeindex = 198;
//				}
//				break;
//			}
			//降落程序
			case 198:
			{
				if (modedelay_s(5)) {
					Modeindex++;
				}
				break;
			}
			case 199:
			{
				Position_Control_set_TargetVelocityZ(-40);
				CtrlunLockHeight();
				Modeindex++;
				break;
			}
			case 200:
			{
				//等待降落完成
				if (get_is_inFlight() == false)
				{
				}
				break;
			}
		}
			
			//主任务
			case 1:
			{
				if (get_Altitude_ControlMode() == Position_ControlMode_Position)
					//				if( get_is_inFlight())
				{
					//					Position_Control_set_XYLock();
					Modeindex++;
				}
				break;
			}
			case 2:
			{
				OrderSetHeightWithCheck(90.f, 20.f);
				CtrlSerLockHeightType(LockHeightType_asonic);
				Modeindex++;
				break;
			}

			case 3:
			{
				if (modedelay(&M36_MAVLINK, 1, 37))
				{
					Attitude_Control_set_Target_YawRate(-2 * 3.14 / 15);
					Modeindex++;
				}
				break;
			}
			case 4:
			{
				if (!receive_z)
				{
					if (receive_x <80 && receive_x > -80)
					{
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
						//Position_Control_set_TargetVelocityBodyHeadingXY(20, 0);
						Modeindex++;
					}
				}
				break;
			}
			case 5:
			{

				Led_setStatus(LED_status_error);
				if (receive_camera_flag) {
					receive_camera_flag = false;
					if (!receive_z) {
						Attitude_Control_set_Target_YawRate(constrain_float(receive_x / 5, 30) / 210);
					}
				}
				if (modedelay_s(4)) {
					Attitude_Control_set_YawLock();
					Led_setStatus(LED_status_running2);
					Modeindex++;
				}
				break;
			}
			case 6:
			{
#define video_err_x 100
				if (receive_camera_flag) {
					receive_camera_flag = false;

					if (!receive_z) {
						if (receive_y - video_err_x < 0) {
							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y - video_err_x), 60), constrain_float(receive_x / 10, 20));
						}
						else {
							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y - video_err_x) / 4, 20), constrain_float(receive_x / 10, 20));
						}
					}
					else {
						Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
					}
				}
				if (receive_y > 90 && receive_y < 110 && receive_x>-20 && receive_x < 20)
				{
					if (OKcount++ > 10) {
						OKcount = 0;
						Position_Control_set_TargetPositionXYRelative(0, 0);
						Modeindex = 7;
					}
				}
				break;
			}
			case 7:
			{
				if (modedelay_s(1))
				{
					Modeindex++;
				}
				break;
			}
			case 8:
			{
				static bool is_receiveqr_double = false;
				if (receive_QR)
				{
					if (is_receiveqr_double) {
						Attitude_Control_set_YawLock();
						Position_Control_set_TargetPositionXYRelative(0, 0);
						Modeindex++;
						k210_2_init();
					}
					is_receiveqr_double = true;
				}
				else
				{
					is_receiveqr_double = false;
					Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y - 90) * 1.25, 20), -17);
					//Attitude_Control_set_Target_YawRate(constrain_float((receive_x+100)/5,20)/70);
					Attitude_Control_set_Target_YawRate(constrain_float((receive_x + 200) / 10, 36) / 60);
				}
				break;
			}
			case 9:
			{
				Position_Control_set_TargetPositionXYRelativeBodyHeading(-30.f, 10.f);
				//				OrderSetPositionWithCheckRelative_BodyHeading(-30.f,0.f,30.f);
				Led_setStatus(LED_status_error);
				Modeindex++;
				break;
			}
			case 10:
			{
				if (modedelay_s(3)) {
					Modeindex++;
//					K210_2_SendCMD(1);
				}
				//				Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y-90)*1.25,20),constrain_float(receive_x/8 ,15));
				//				if (receive_x > -20 && receive_x < 20){
				//					if (++OKcount>50)
				//					{
				//						Modeindex ++;
				//						OKcount = 0;
				//					}
				//				}
				
				break;
			}
			case 11:
			{
				if (receive_camera_flag) {
					receive_camera_flag = false;
					#define video_qr_err 60
					if (!receive_z) {
						if (receive_y - video_qr_err < 0) {
							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y - video_qr_err), 60), constrain_float(receive_x / 10, 20));
						}
						else {
							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((receive_y - video_qr_err) / 4, 20), constrain_float(receive_x / 10, 20));
						}
					}
					else {
						Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
					}
				}
				if (modedelay_s(5)) {
					Modeindex = 15;
					Led_setStatus(LED_status_running2);
				}
				break;
			}
			case 15:
			{
				static uint16_t mycount = 0;
				float erry = receive_y - 45;
				if (receive_camera_flag) {
					receive_camera_flag = false;
					if (!receive_z) {
						if (erry < 0) {
							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((erry) * 1.5, 60), constrain_float(receive_x / 10, 20));
						}
						else {
							Position_Control_set_TargetVelocityBodyHeadingXY(constrain_float((erry) / 5, 20), constrain_float(receive_x / 10, 20));
						}
					}
					else {
						Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
					}
				}
				if (k210_2_rcflag && receive_x > -20 && receive_x < 20 && (erry)<10 && (erry)>0)
				{
					uint8_t k210num = get_k210_2_num();
					K210_2_SendCMD(k210num);
					K210_SendCMD(k210num);
					qr_circle_mode = k210num;
					Position_Control_set_TargetPositionXYRelative(0,0);
//					Position_Control_set_XYLock();
					Modeindex++;
					break;
				}
				if (++mycount > 750 && receive_x > -20 && receive_x < 20 && (erry)<10 && (erry)>0){
					K210_2_SendCMD(1);
					K210_SendCMD(1);
					qr_circle_mode = 1;
					Position_Control_set_TargetPositionXYRelative(0,0);
					Modeindex++;
					break;
				}
				break;
			}
			case 16:
			{
				//					Position_Control_set_TargetVelocityZ(0);
				AfterTurnYaw = Quaternion_getYaw(get_Airframe_attitude());
				Modeindex++;
				break;
			}
			case 17:
			{
				if (modedelay(&M36_MAVLINK, 1, 37))
				{
					OrderSetHeightWithCheck(160.f, 20.f);
					CtrlSerLockHeightType(LockHeightType_t265);
					Position_Control_set_TargetPositionXYRelative(0,0);
					//					Position_Control_set_TargetPositionZRelative(70);
					Modeindex++;
				}
				break;
			}
			case 18:
			{
				Modeindex++;
				break;
			}
			case 19:
			{
				if (modedelay(&M36_MAVLINK, 2, 37))
				{
//					CtrlSerLockHeightType(LockHeightType_t265);
//					CtrlLockHeight(get_height_t265());
					OrderSetPositionWithCheckRelative_BodyHeading(150.f, 0.f, 50.f);
					//					Position_Control_set_TargetPositionXYRelativeBodyHeading(150,0);
					Modeindex++;
				}
				break;
			}

			case 20:
			{

				Led_setStatus(LED_status_running2);
				Modeindex++;
				break;
			}
			case 21:
			{

				if (modedelay(&M36_MAVLINK, 1, 16))
				{
					OrderTurnSelfCenter(0.f, AfterTurnYaw > 0 ? (-2 * Pi / 20) : (2 * Pi / 20));
					//					Attitude_Control_set_Target_Yaw(0.f);
					Modeindex++;
				}
				break;
			}
			case 22:
			{
				//				if (AfterTurnYaw > PI_f*2/360*135 || AfterTurnYaw < -PI_f*2/360*45){
				//					OrderSetPositionWithCheck(0.f,300.f,60.f);
				//				}else{
				//					OrderSetPositionWithCheck(300.f,0.f,60.f);
				//				}
				//				Modeindex ++;
				if (modedelay(&M36_MAVLINK, 1, 17))
				{
					if (AfterTurnYaw > PI_f * 2 / 360 * 135 || AfterTurnYaw < -PI_f * 2 / 360 * 45) {
						Position_Control_set_TargetPositionXYRelativeBodyHeading(0, -(sinf(AfterTurnYaw) * 100 + 130));
					}
					else {
						Position_Control_set_TargetPositionXYRelativeBodyHeading(-cosf(AfterTurnYaw) * 100 + 150, 0);
					}

					Modeindex++;
				}
				break;
			}
			case 23:
			{
				//				OrderSetPositionWithCheck(300.f,300.f,60.f);
				//				Modeindex ++;
				if (modedelay(&M36_MAVLINK, 5, 18))
				{
					if (AfterTurnYaw > PI_f * 2 / 360 * 135 || AfterTurnYaw < -PI_f * 2 / 360 * 45) {
						Position_Control_set_TargetPositionXYRelativeBodyHeading(-cosf(AfterTurnYaw) * 100 + 150, 0);
					}
					else {
						Position_Control_set_TargetPositionXYRelativeBodyHeading(0, -(sinf(AfterTurnYaw) * 100 + 130));
					}
					Modeindex++;
				}
				break;
			}
			case 24:
			{
				Modeindex++;
				OrderSetHeightWithCheck(75.f, 20.f);
				k210_rcflag = false;
				//				if(modedelay(&M36_MAVLINK, 2, 19)){
				//					Modeindex ++;
				//					OrderSetHeightWithCheck(75.f,20.f);
				////					Position_Control_set_TargetVelocityZ(-30);
				//					k210_rcflag = false;
				//				}
				break;
			}
			case 25:
			{
				//				if((pos_vision.z-Height_t265)<75.f){
				//					Position_Control_set_TargetPositionZRelative(75.f-(pos_vision.z-Height_t265));
				//					Modeindex ++;
				//					k210_rcflag = false;
				//				}
				if(modedelay_s(3)){
					Modeindex++;
					k210_rcflag = false;
				}
				break;
			}
			case 26:
			{
				if (k210_rcflag && (k210_state % 10 == 1)) {//找到红色
//					if (abs(k210_pos_x) > 2 && abs(k210_pos_y) > 2)
						Position_Control_set_TargetPositionXYRelative(constrain_float(k210_pos_x - 120 - 2, 50), constrain_float(k210_pos_y - 160 - 15, 50));
					//					else
					//						Position_Control_set_TargetPositionXYRelative(0,0);				
					k210_rcflag = false;
				}
				if (modedelay(&M36_MAVLINK, 9, 21)) {
					Modeindex++;
					if (qr_circle_mode == 1 || qr_circle_mode == 3)
					{
						K210_SendCMD(8);//red
					}else
					{
						K210_SendCMD(7);//green
					}
				}
				break;
			}
			case 27:
			{
				if (modedelay(&M36_MAVLINK, 3, 22))
				{
					OrderSetPositionWithCheck(30.f, -300.f, 80.f);
//					CtrlLockHeight(100.f);
					//					Position_Control_set_TargetPositionXYRelativeBodyHeading(-300,0);
					K210_SendCMD(5);//呼叫小车
					Led_setStatus(LED_status_error);
					Modeindex++;
				}
				break;
			}
			case 28:
			{
				if (modedelay(&M36_MAVLINK, 1, 23))
				{
					//					OrderSetPositionWithCheck(0.f,0.f,80.f);
					Modeindex++;
					//					Modeindex = 50;
				}
				break;
			}
			case 29:
			{
				if (k210_state > 10)//小车就位
				{
					Position_Control_set_TargetVelocityBodyHeadingXY(0, 20);
					Modeindex++;
					Led_setStatus(LED_status_running2);
					k210_rcflag = false;
				}
				break;
			}
			case 30:
			{
				if (k210_rcflag ) {//找到绿色
					if (k210_pos_x!=0&&k210_pos_y!=0)	
					{
//						CtrlLockHeight(75.f);
						Position_Control_set_TargetPositionXYRelative(constrain_float(k210_pos_x - 120 - 5, 50), constrain_float(k210_pos_y - 160 - 15, 50));
					}
					k210_rcflag = false;
					if (modedelay(&M36_MAVLINK, 6, 25)) {
						Modeindex++;
						if (qr_circle_mode == 1 || qr_circle_mode == 3)
						{
							K210_SendCMD(7);//green
						}else
						{
							K210_SendCMD(8);//red
						}
					}
				}else{
					k210_rcflag = false;
				}
				break;
			}
			case 31:
			{
				if (k210_rcflag) {//找到绿色
					if (abs(k210_pos_x) > 3 && abs(k210_pos_y) > 3)
						Position_Control_set_TargetPositionXYRelative(constrain_float(k210_pos_x - 120 - 5, 50), constrain_float(k210_pos_y - 160 - 15, 50));
					k210_rcflag = false;
				}
				if (modedelay(&M36_MAVLINK, 2, 26)) {
					OrderSetPositionWithCheck(0.f, 0.f, 80.f);
					if(qr_circle_mode == 3 || qr_circle_mode == 4)
					{
					K210_SendCMD(9);
					}
					//					Position_Control_set_TargetPositionXY(landpos_x,landpos_y);
					Modeindex = 50;
				}
				break;
			}
			case 50:
			{
				if (modedelay(&M36_MAVLINK, 1, 40)) {
					OrderLandByUltrasonic();
					Modeindex++;
				}
				break;
			}
			case 51:
			{
				//				VelocityENU = get_VelocityENU();
				//				Position = get_Position();
				//				if(Position.z < Height + 5.f && VelocityENU.z > 0)
				//				{
				//					Position_Control_set_TargetPositionZRelative(-100.f);
				//					//Attitude_Control_set_Throttle( throttle_stick );
				//					
				//					Modeindex ++;
				//				}
				K210_SendCMD(9);
				Modeindex++;
				break;
			}
			case 52:
			{
				//等待降落完成
				
				if(modedelay_s(1)){
					K210_SendCMD(9);
				}
				break;
			}
			}
		}
		else {
			//执行封装好的操作
			static uint8_t div_count = 0;
#define fre_5Hz 10
			switch (OrderMode) {
			case Exit: {
				div_count = 0;
				IsInflightOrder = false;
				break;
			}
			case SetPositionWithCheck: {
				vector3_float pos_now = get_position_t265();
				float dis = sqrtf((TgtPos.x - pos_now.x) * (TgtPos.x - pos_now.x) + (TgtPos.y - pos_now.y) * (TgtPos.y - pos_now.y));

				if (++div_count > fre_5Hz) {
					div_count = 0;
					if (dis > MaxPosLimit) {
						Position_Control_set_TargetPositionXYRelative((TgtPos.x - pos_now.x) * MaxPosLimit / dis, (TgtPos.y - pos_now.y) * MaxPosLimit / dis);
					}
					else {
						Position_Control_set_TargetPositionXYRelative(TgtPos.x - pos_now.x, TgtPos.y - pos_now.y);
					}
				}

				if (dis < EXITThreshold) {
					OrderMode = Exit;
				}
				break;
			}
			case TurnSelfCenter: {
				float currentYaw = Quaternion_getYaw(get_Airframe_attitude());

				//改进
				if (currentYaw >= target_yaw - Pi / 72 && currentYaw <= target_yaw + Pi / 72)
				{
					Attitude_Control_set_Target_Yaw(target_yaw);
					Position_Control_set_TargetPositionXYRelative(
						current_pos.x - turnyaw_errx + cosf(target_yaw) * FlightR - get_position_t265().x,
						current_pos.y - turnyaw_erry + sinf(target_yaw) * FlightR - get_position_t265().y
					);
					OrderMode = Exit;
				}
				else {
					if (++div_count > fre_5Hz) {
						div_count = 0;
						Position_Control_set_TargetPositionXYRelative(
							current_pos.x - turnyaw_errx + cosf(currentYaw) * FlightR - get_position_t265().x,
							current_pos.y - turnyaw_erry + sinf(currentYaw) * FlightR - get_position_t265().y
						);
					}
				}
				break;
			}
			case Circlrmode: {
				float currentYaw = Quaternion_getYaw(get_Airframe_attitude());
				float center_yaw = currentYaw + CirCleYaw_err;
				if (center_yaw > M_PI)center_yaw -= 2 * M_PI;
				else if (center_yaw < -M_PI)center_yaw += 2 * M_PI;


				if (++time_count > 200 && currentYaw >= target_yaw - Pi / 72 && currentYaw <= target_yaw + Pi / 72) {

					//					Position_Control_set_TargetPositionXYRelative(
					//					current_pos.x - cosf(target_yaw+CirCleYaw_err)*CircleRadiu - get_position_t265().x,
					//					current_pos.y - sinf(target_yaw+CirCleYaw_err)*CircleRadiu - get_position_t265().y
					//					);
					time_count = 0;
					Position_Control_set_TargetPositionXYRelative(0, 0);
					Attitude_Control_set_Target_Yaw(target_yaw);
					OrderMode = Exit;
				}
				else {
					if (++div_count > fre_5Hz) {
						div_count = 0;
						Position_Control_set_TargetPositionXYRelative(
							current_pos.x - cosf(center_yaw + CirCleDelayYaw_err) * CircleRadiu - get_position_t265().x,
							current_pos.y - sinf(center_yaw + CirCleDelayYaw_err) * CircleRadiu - get_position_t265().y
						);
					}
				}
				break;
			}
			case LandByUltrasonic: {
				if (ultrasonic < 5.f && get_VelocityENU().z > -10)
				{
					OrderMode = Exit;
					set_inFlight_false();
				}
				break;
			}
			case TakeOffWithInit: {
				static uint8_t takeoffstate = 0;
				switch (takeoffstate) {
				case 0: {
					Buzzer(true);
					if (modedelay_s(2)) {
						Buzzer(false);
						bool mystate = Position_Control_Takeoff_HeightRelative(tgt_height + 30);
						if (!mystate)Led_setStatus(LED_status_error);

						takeoffstate++;
					}
					break;
				}
				case 1: {
					if (get_Altitude_ControlMode() == Position_ControlMode_Position) {
						OrderMode = Exit;
						CtrlLockHeight(tgt_height);
						takeoffstate = 0;
					}
					break;
				}
				}
				break;
			}
			case SetHeightWithCheck: {
				if (IsDirUp) {
					if (get_height_t265() > tgt_height - 5) {
						Position_Control_set_TargetVelocityZ(0);
						CtrlLockHeight(tgt_height);
						OrderMode = Exit;
					}
				}
				else {
					if (get_height_t265() < tgt_height + 5) {
						Position_Control_set_TargetVelocityZ(0);
						CtrlLockHeight(tgt_height);
						OrderMode = Exit;
					}
				}
				break;
			}
			case TakeOff45degree: {
				static uint8_t takeoff45state = 0;
				switch (takeoff45state) {
				case 0: {
					Buzzer(true);
					if (modedelay_s(2)) {
						Buzzer(false);
						bool mystate = Position_Control_Takeoff_HeightRelative(20.f);
						if (!mystate)Led_setStatus(LED_status_error);

						takeoff45state++;
					}
					break;
				}
				case 1: {
					if (get_Altitude_ControlMode() == Position_ControlMode_Position) {
						takeoff45state++;
					}
					break;
				}
				case 2: {
					Position_Control_set_TargetVelocityZ(20.f);
					Position_Control_set_TargetVelocityBodyHeadingXY(0, 20.f);
					takeoff45state++;
					break;
				}
				case 3: {
					if (get_height_t265() > tgt_height) {
						Position_Control_set_ZLock();
						Position_Control_set_XYLock();
						OrderMode = Exit;
						takeoff45state = 0;
					}
					break;
				}
				}
				break;
			}
			default: {
				Led_setStatus(LED_status_error);
				OrderMode = Exit;
				break;
			}
			}
		}
	}
	else
	{
		//MODE按钮在高位或摇杆不在中位
		guided_enabled = false;
		//关闭水平位置控制
		Position_Control_Disable();

		//高度控制输入
		if (in_symmetry_range_offset_float(throttle_stick, 5, 50))
			Position_Control_set_ZLock();
		else
			Position_Control_set_TargetVelocityZ((throttle_stick - 50.0f) * 6);

		//偏航控制输入
		if (in_symmetry_range_offset_float(yaw_stick, 5, 50))
			Attitude_Control_set_YawLock();
		else
			Attitude_Control_set_Target_YawRate((50.0f - yaw_stick) * 0.05f);

		//Roll Pitch控制输入
		Attitude_Control_set_Target_RollPitch(\
			(roll_stick - 50.0f) * 0.015f, \
			(pitch_stick - 50.0f) * 0.015f);
	}
}

bool Get_Guided_Mode_Enabled(void)
{
	return guided_enabled;
}
bool Get_POS_Control_Enabled(void)
{
	return pos_enabled;
}
void Set_POS_Control_Enabled(bool value)
{
	pos_enabled = value;
}
// 按照自身中心旋转
//bool turnyaw_selfcenter(const float yaw_rate, const float target_yaw)
//{
//	static uint8_t state = 0;
//	//自旋相关参数			
//	static vector3_float current_pos;
//	static float currentYaw = 0;
//	static float turnyaw_errx = 0, turnyaw_erry = 0;
//	#define FlightR 11.5 //飞机半径
////自旋相关参数	end
//	bool return_state = false;
//	switch(state){
//		case 0:
//       {
//					Attitude_Control_set_Target_YawRate(yaw_rate);
//					state ++;
//					currentYaw = Quaternion_getYaw( get_Airframe_attitude() );
//					current_pos = get_Position();
//					turnyaw_errx = cosf(currentYaw)*FlightR, turnyaw_erry = sinf(currentYaw)*FlightR;
//		//current_pos = get_Position();
//					break;
//       }
//		case 1:
//			{
//					currentYaw = Quaternion_getYaw( get_Airframe_attitude() );
//					Position_Control_set_TargetPositionXY(
//						current_pos.x - turnyaw_errx + cosf(currentYaw)*FlightR,
//						current_pos.y - turnyaw_erry + sinf(currentYaw)*FlightR
//					);
//		
//					if (currentYaw >= target_yaw - Pi / 36 && currentYaw <= target_yaw + Pi / 36)
//					{
//							Attitude_Control_set_Target_Yaw(target_yaw);
//							return_state = true;
//							state = 0;
//					}
//					break;
//			}
//	}
//	return return_state;
//}



void OrderSetPositionWithCheck(float tgtx, float tgty, uint8_t limit) {
	IsInflightOrder = true;
	OrderMode = SetPositionWithCheck;

	TgtPos.x = tgtx;
	TgtPos.y = tgty;
	MaxPosLimit = limit;
}
void OrderSetPositionWithCheckRelative(float tgtx, float tgty, uint8_t limit) {
	OrderSetPositionWithCheck(get_position_t265().x + tgtx, get_position_t265().y + tgty, limit);
}
void OrderSetPositionWithCheckRelative_BodyHeading(float tgtx, float tgty, uint8_t limit) {
	vector3_float current_pos = get_position_t265();
	float Yaw = Quaternion_getYaw(get_Airframe_attitude());
	float Yaw_sin, Yaw_cos;
	arm_sin_cos_f32(rad2degree(Yaw), &Yaw_sin, &Yaw_cos);
	float posx_ENU = map_BodyHeading2ENU_x(tgtx, tgty, Yaw_sin, Yaw_cos);
	float posy_ENU = map_BodyHeading2ENU_y(tgtx, tgty, Yaw_sin, Yaw_cos);
	OrderSetPositionWithCheck(current_pos.x + posx_ENU, current_pos.y + posy_ENU, limit);
}
void OrderTurnSelfCenter(float tgtyaw, float yaw_rate) {
	IsInflightOrder = true;
	OrderMode = TurnSelfCenter;

	Attitude_Control_set_Target_YawRate(yaw_rate);
	target_yaw = tgtyaw;
	float currentYaw = Quaternion_getYaw(get_Airframe_attitude());
	current_pos = get_position_t265();
	turnyaw_errx = cosf(currentYaw) * FlightR, turnyaw_erry = sinf(currentYaw) * FlightR;
}
void OrderCirclrmode(float tgtyaw, float yaw_rate, float radiu, float yaw_err) {
	IsInflightOrder = true;
	OrderMode = Circlrmode;
	//消除双目安装偏差
	float currentYaw = Quaternion_getYaw(get_Airframe_attitude());
	//	turnyaw_errx = cosf(currentYaw)*FlightR, turnyaw_erry = sinf(currentYaw)*FlightR;
		//根据半径radiu和圆心对飞机的偏移量yaw_err ,求出圆心坐标（yaw_err,radiu相当于极坐标的 斯塔 和 r 。都是bodyheading坐标系）
	float center_yaw = Quaternion_getYaw(get_Airframe_attitude()) + yaw_err;
	if (center_yaw > M_PI)center_yaw -= 2 * M_PI;
	else if (center_yaw < -M_PI)center_yaw += 2 * M_PI;
	//	current_pos.x = get_position_t265().x - turnyaw_errx + cosf(center_yaw)*(radiu);
	//	current_pos.y = get_position_t265().y - turnyaw_erry + sinf(center_yaw)*(radiu);
	current_pos.x = get_position_t265().x + cosf(center_yaw) * (radiu);
	current_pos.y = get_position_t265().y + sinf(center_yaw) * (radiu);
#define DelayYaw_err 2*M_PI/16
	if (yaw_rate > 0)CirCleDelayYaw_err = DelayYaw_err;
	else CirCleDelayYaw_err = -DelayYaw_err;

	Attitude_Control_set_Target_YawRate(yaw_rate);
	target_yaw = tgtyaw;
	CircleRadiu = radiu;
	CirCleYaw_err = yaw_err;
}
void OrderLandByUltrasonic() {
	IsInflightOrder = true;
	OrderMode = LandByUltrasonic;

	Position_Control_set_TargetVelocityZ(-40);
	CtrlunLockHeight();
}
void OrderTakeOffWithInit(float myHeight) {
	IsInflightOrder = true;
	OrderMode = TakeOffWithInit;

	Height = get_Position().z;
	Height_t265 = get_position_t265().z;
	Height_asonic = ultrasonic;
	tgt_height = myHeight;
}
void CtrlLockHeight(float myHeight) {
	IsLockHeight = true;
	Lock_Height = myHeight;
}
void CtrlunLockHeight() {
	IsLockHeight = false;
}
void CtrlSerLockHeightType(uint8_t type) {
	LockHeightType = type;
}
void OrderSetHeightWithCheck(float myheight, float myvelosity) {
	IsInflightOrder = true;
	OrderMode = SetHeightWithCheck;

	//解锁高度锁定
	CtrlunLockHeight();

	if (myvelosity < 0)myvelosity = -myvelosity;
	if (myheight < get_height_t265()) {
		myvelosity = -myvelosity;
		IsDirUp = false;
	}
	else {
		IsDirUp = true;
	}

	Position_Control_set_TargetVelocityZ(myvelosity);
	tgt_height = myheight;
}

void OrderTakeOff45degree(float myHeight) {
	IsInflightOrder = true;
	OrderMode = TakeOff45degree;
	time_count = 0;

	Height = get_Position().z;
	Height_t265 = get_position_t265().z;
	Height_asonic = ultrasonic;
	tgt_height = myHeight;
}
void Order_template() {
	IsInflightOrder = true;
	OrderMode = 0;
	time_count = 0;
}
void ultra_order(){
	static uint8_t count = 0;
	if(count<5)
		count++;
	else
	{
		uint8_t send_array[1] = {85};
		Uart7_Send(send_array, 1);
		count = 0;
	}
}