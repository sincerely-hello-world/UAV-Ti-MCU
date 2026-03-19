#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>
#include "M35_Auto1.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"
#include "drv_Uart2.h"
#include "drv_SDI.h"

static void M35_Auto1_MainFunc();
static void M35_Auto1_enter();
static void M35_Auto1_exit();
const Mode M35_Auto1 = 
{
	50 , //mode frequency
	M35_Auto1_enter , //enter
	M35_Auto1_exit ,	//exit
	M35_Auto1_MainFunc ,	//mode main func
};

typedef struct
{
	//退出模式计数器
	uint16_t exit_mode_counter;
	
	//上次是否自动控制
	bool last_auto;
	
	//自动飞行状态机
	uint8_t auto_step1;	//0-记录按钮位置
											//1-等待按钮按下起飞 
											//2-等待起飞完成 
											//3-等待2秒
											//4-降落
											//5-等待降落完成
	uint16_t auto_counter;
	uint16_t point_ind;
	float last_button_value;
	
	float last_height;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M35_Auto1_enter()
{
	Led_setStatus( LED_status_running1 );
	
	//初始化模式变量
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->exit_mode_counter = 0;
	Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
	Altitude_Control_Enable();
}

static void M35_Auto1_exit()
{
	Altitude_Control_Disable();
	Attitude_Control_Disable();
	
	free( Mode_Inf );
}
#define write_new_byte(buffer,i,byte) buffer[i++]=byte;CK_A+=byte; CK_B+=CK_A

extern float SDI_Data[9];
extern TIME MFin_Time;
static void M35_Auto1_MainFunc()
{
		const Receiver* rc = get_current_Receiver();
		
	if( rc->available == false )
	{
		//接收机不可用
		//降落
		Position_Control_set_XYLock();
		Position_Control_set_TargetVelocityZ( -50 );
		return;
	}
	float throttle_stick = rc->data[0];
	float yaw_stick = rc->data[1];
	float pitch_stick = rc->data[2];
	float roll_stick = rc->data[3];	
	
	/*判断退出模式*/
		if( throttle_stick < 5 && yaw_stick < 5 && pitch_stick < 5 && roll_stick > 95 )
		{
			if( ++Mode_Inf->exit_mode_counter >= 50 )
			{
				change_Mode( 1 );
				return;
			}
		}
		else
			Mode_Inf->exit_mode_counter = 0;
	/*判断退出模式*/
		
	//判断摇杆是否在中间
	bool sticks_in_neutral = 
		in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( pitch_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( roll_stick , 5 , 50 );
	
	extern int32_t SDI_Point[30];
	extern TIME SDI_Time;
	extern TIME MSt_Time;
	
	if( sticks_in_neutral && get_Position_Measurement_System_Status() == Measurement_System_Status_Ready )
	{
		//摇杆在中间
		//执行自动飞行		
		//只有在位置有效时才执行自动飞行
		
		//打开水平位置控制
		Position_Control_Enable();
		switch( Mode_Inf->auto_step1 )
		{
			case 0:
				Mode_Inf->last_button_value = rc->data[5];
				++Mode_Inf->auto_step1;
				Mode_Inf->auto_counter = 0;
				break;
			
			case 1:
			{
				//等待按钮按下起飞			
				extern TIME Takeoff_Time;
				if( (get_is_inFlight() == false && fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15) || get_pass_time(Takeoff_Time)<0.5f )
				{
					Position_Control_Takeoff_HeightRelative( 300.0f );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
					unsigned char sd[] = { 0xa5 , 0x5a , 0x04 , 0xb6 , 0xba , 0xaa };
					Uart2_Send( sd , sizeof(sd) );
				}
				else
					goto ManualControl;
				break;
			}
			case 2:
			{
				//等待起飞完成飞直线
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					if( SDI_Point[2] > 300 )
						Position_Control_set_TargetPositionZRelative( SDI_Point[2]-300 );
					else
						Position_Control_set_TargetPositionZRelative( 1700 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
					Mode_Inf->point_ind = 0;
				}
				break;
			}
				
			case 3:
				//等待执行任务命令
				if( get_pass_time(MFin_Time) < 0.5f )
				{
					Mode_Inf->auto_step1 = 7;
					Mode_Inf->auto_counter = 0;
				}
				else if( get_pass_time(MSt_Time) < 0.5f )
				{
					Mode_Inf->auto_step1 = 3;
					Mode_Inf->auto_counter = 0;
				}
				break;				
				
			case 4:
			{	//转偏航
				static TIME old_tt;
				if( get_Altitude_ControlMode() == Position_ControlMode_Position && get_Position_ControlMode() == Position_ControlMode_Position )
				{
					if( Mode_Inf->point_ind > 0 )
					{
						uint8_t buf[30];
						unsigned char CK_A = 0 , CK_B = 0;
						buf[0] = 'A';	buf[1] = 'C';
						int buffer_index = 2;
						write_new_byte( buf, buffer_index, Mode_Inf->point_ind );
						write_new_byte( buf, buffer_index, 12 );
						
						uint16_t p = Mode_Inf->point_ind - 1;
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+0]))[0] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+0]))[1] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+0]))[2] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+0]))[3] );
						
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+1]))[0] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+1]))[1] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+1]))[2] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+1]))[3] );
						
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+2]))[0] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+2]))[1] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+2]))[2] );
						write_new_byte( buf, buffer_index, ((uint8_t*)&(SDI_Point[p*3+2]))[3] );
						
						buf[ buffer_index++ ] = CK_A;
						buf[ buffer_index++ ] = CK_B;
						Uart3_Send( buf, buffer_index );
					}
					
					if( SDI_Point[Mode_Inf->point_ind*3+1]!=0 && SDI_Point[Mode_Inf->point_ind*3+0]!=0 )
					{	//继续飞下个航点					
						float x , y;
						if( get_Point_From_LatLon( &x , &y , SDI_Point[Mode_Inf->point_ind*3+1]*1e-7 , SDI_Point[Mode_Inf->point_ind*3+0]*1e-7 ) )
						{	//转偏航
							float LineA = get_Position().y - y;
							float LineB = x - get_Position().x;
							Attitude_Control_set_Target_Yaw( atan2f(-LineA,LineB) );
						}
						Mode_Inf->auto_counter = 0;
						++Mode_Inf->auto_step1;
					}
					else
					{	//航点飞行完成
						Mode_Inf->point_ind = 0;
						uint8_t buf[30];
						unsigned char CK_A = 0 , CK_B = 0;
						buf[0] = 'A';	buf[1] = 'C';
						int buffer_index = 2;
						write_new_byte( buf, buffer_index, 120 );
						write_new_byte( buf, buffer_index, 0 );
						
						buf[ buffer_index++ ] = CK_A;
						buf[ buffer_index++ ] = CK_B;
						Uart3_Send( buf, buffer_index );
						
						Mode_Inf->auto_step1 = 2;
						Mode_Inf->auto_counter = 0;		
					}						
				}
				else if( SDI_Time.t!=old_tt.t && get_pass_time(SDI_Time)<0.5f )
				{	//获取到导线位置，将航线水平移动靠近导线
					old_tt.t = SDI_Time.t;
					Position_Control_move_TargetPositionXYRelativeBodyHeading( 0, -SDI_Data[0]*0.05f );
				}
				break;
			}
				
			case 5:
			{	//等待转偏航
				if( ++Mode_Inf->auto_counter >= 150 )
				{
					Mode_Inf->auto_counter = 0;
					++Mode_Inf->auto_step1;
				}
				break;
			}
			
			case 6:				
			{	//等待起飞完成飞直线
				if( SDI_Point[Mode_Inf->point_ind*3+1]!=0 && SDI_Point[Mode_Inf->point_ind*3+0]!=0 )
				{	//继续飞下个航点
					Position_Control_set_TargetPositionXY_LatLon( SDI_Point[Mode_Inf->point_ind*3+1]*1e-7 , SDI_Point[Mode_Inf->point_ind*3+0]*1e-7 );
					if( Mode_Inf->point_ind>0 && SDI_Point[Mode_Inf->point_ind*3+2]>300 )
						Position_Control_set_TargetPositionZRelative( (float)SDI_Point[Mode_Inf->point_ind*3+2] - (float)SDI_Point[Mode_Inf->point_ind*3-3+2] );
					
					++Mode_Inf->point_ind;
					Mode_Inf->auto_step1 = 4;
				}
				else
					++Mode_Inf->auto_step1;
				break;
			}
			
			case 7:
				//等待直线飞行完成下降
				if( get_Position_ControlMode() == Position_ControlMode_Position )
				{
					if( SDI_Point[2] > 400 )
						Position_Control_set_TargetPositionZRelative( (float)400-SDI_Point[2] );
					else
						Position_Control_set_TargetPositionZRelative( -1500 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
					Mode_Inf->last_height = 500;
					OLED_Draw_TickCross8x6( false , 0 , 0 );
				}
				break;
				
			case 8:
				//等待降高度完成
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;

			case 9:			
				Position_Control_set_TargetVelocityZ(-30);
			
				//等待按钮按下
				if( get_is_inFlight() == false )
				{
					unsigned char sd[] = { 0xa5 , 0x5a , 0x04 , 0xb4 , 0xb8 , 0xaa };
					Uart2_Send( sd , sizeof(sd) );
					Position_Control_set_XYLock();
					Mode_Inf->auto_step1 = 0;
					Mode_Inf->auto_counter = 0;
				}
				break;
						
		}
	}
	else
	{
		ManualControl:
		//摇杆不在中间
		//手动控制
		if( Mode_Inf->last_auto==true )
		{
			Mode_Inf->last_auto = false;
			
			uint8_t buf[30];
			unsigned char CK_A = 0 , CK_B = 0;
			buf[0] = 'A';	buf[1] = 'C';
			int buffer_index = 2;
			write_new_byte( buf, buffer_index, 121 );
			write_new_byte( buf, buffer_index, 0 );
			
			buf[ buffer_index++ ] = CK_A;
			buf[ buffer_index++ ] = CK_B;
			Uart3_Send( buf, buffer_index );
			
			Mode_Inf->auto_counter = 0;
		}
		//Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
		
				char str[15];
				double Lat , Lon;
				get_LatLon_From_Point( get_Position().x , get_Position().y , &Lat , &Lon );
				sprintf( str , "%3.9f", Lat );
				OLED_Draw_Str8x6( str , 0 , 0 );
				sprintf( str , "%3.9f", Lon );
				OLED_Draw_Str8x6( str , 1 , 0 );
				sprintf( str , "%3.9f", SDI_Point[1]*1e-7 );
				OLED_Draw_Str8x6( str , 3 , 0 );
				sprintf( str , "%3.9f" , SDI_Point[0]*1e-7 );
				OLED_Draw_Str8x6( str , 4 , 0 );
				sprintf( str , "%3.9f", SDI_Data[0]*1.0 );
				OLED_Draw_Str8x6( str , 6 , 0 );
				sprintf( str , "%3.9f" , SDI_Data[1]*1.0 );
				OLED_Draw_Str8x6( str , 7 , 0 );
				OLED_Update();
		
		//关闭水平位置控制
		Position_Control_Disable();
		
		//高度控制输入
		if( in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) )
			Position_Control_set_ZLock();
		else
			Position_Control_set_TargetVelocityZ( ( throttle_stick - 50.0f ) * 6 );

		//偏航控制输入
		if( in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) )
			Attitude_Control_set_YawLock();
		else
			Attitude_Control_set_Target_YawRate( ( 50.0f - yaw_stick )*0.05f );
		
		//Roll Pitch控制输入
		Attitude_Control_set_Target_RollPitch( \
			( roll_stick 	- 50.0f )*0.015f, \
			( pitch_stick - 50.0f )*0.015f );
	}
}