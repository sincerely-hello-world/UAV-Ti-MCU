#include "Basic.h"
#include "STS.h"
#include "drv_Laser.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"

#include "TM4C123GH6PM.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "pwm.h"
#include "rom.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"

uint16_t crc16tablefast(uint8_t *ptr, uint16_t len);
void Laser_Handler(char rdata);
void K210_Handler(char rdata);
static void Process_K210_Data(char * buffer);
static void Laser_Server( unsigned int Task_ID );
static void Process_Laser_Data(char * buffer);
extern int k210_pos_x,k210_pos_y,k210_state;
int k210_pos_x=0,k210_pos_y=0,k210_state=0;
extern bool k210_rcflag;
bool k210_rcflag = false;
/*×´Ì¬»ú*/

/*CRCËã·¨*/
//void init_drv_Laser()
//{
//    //×¢²á´«¸ÐÆ÷
//    PositionSensorRegister( default_laser_sensor_index, \
//                            Position_Sensor_Type_RangePositioning, \
//                            Position_Sensor_DataType_s_z, \
//                            Position_Sensor_frame_ENU, \
//                            0.05f, \
//                            0, 0 );
//}

void K210_Handler(char rdata)
{
    static int recv_state = 0;
    static int j = 0;
    static char rbuffer[16];
		static int tmpx=0,tmpy=0,tmpstate=0;
    /*×´Ì¬»ú*/
		switch(recv_state){
			case 0:{
				if (rdata == 0x73)recv_state++;
				break;
			}
			case 1:{
				tmpx = (int)rdata;
				recv_state++;
				break;
			}
			case 2:{
				tmpx += (int)rdata<<8;
				recv_state++;
				break;
			}
			case 3:{
				tmpy = (int)rdata;
				recv_state++;
				break;
			}
			case 4:{
				tmpy += (int)rdata<<8;
				recv_state++;
				break;
			}
			case 5:{
				tmpstate = (int)rdata;
				recv_state++;
				break;
			}
			case 6:{
				tmpstate += (int)rdata<<8;
				recv_state++;
				break;
			}
			case 7:{
				if (rdata == 0x3c){
					k210_pos_x = tmpx;
					k210_pos_y = tmpy;
					k210_state = tmpstate;
					k210_rcflag = true;
				}
				recv_state = 0;
				break;
			}
			default :{
				recv_state = 0;
				break;
			}
		}
}



static void Process_K210_Data(char * buffer)
{

}