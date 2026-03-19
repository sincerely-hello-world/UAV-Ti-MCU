#include "Basic.h"
#include "STS.h"
#include "drv_Laser.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"
#include "drv_Uart2.h"
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
#include "mavlink.h"
#include "Commulink.h"
uint16_t crc16tablefast(uint8_t *ptr, uint16_t len);
void Laser_Handler(char rdata);
static void Laser_Server( unsigned int Task_ID );
static void Process_Laser_Data(char * buffer);
/*×´Ì¬»ú*/
float laser_dis[2];
uint8_t current_id=0;
bool received=true;
uint32_t start_time=0;
/*CRCËã·¨*/
void init_drv_Laser()
{
    //×¢²á´«¸ÐÆ÷Position_Sensor_Type_RangePositioning
    PositionSensorRegister( default_laser_sensor_index, \
                            Position_Sensor_Type_RangePositioning, \
                            Position_Sensor_DataType_s_z, \
                            Position_Sensor_frame_ENU, \
                            0.05f, \
                            0,0 );
    STS_Add_Task( STS_Task_Trigger_Mode_RoughTime, 1.0f/50, 0, Laser_Server );
}
void Send_Measure_Start(uint8_t id)
{
    uint8_t frame[8]= {0x57,0x10,0xFF,0xFF,0x00,0xFF,0xFF,0x63};
    frame[4]=id;
    frame[7]+=id;
    received=false;
    Uart2_Send(frame,8);
}

static void Laser_Server( unsigned int Task_ID )
{
    if(received||(get_System_Run_Time() * 1000-start_time>40))
    {
        Send_Measure_Start(current_id);
        start_time=get_System_Run_Time() * 1000;
    }
}

void Laser_Handler(char rdata)
{
    static int recv_state=0;
    static int j=0;
    static char rbuffer[16];
    /*×´Ì¬»ú*/
    if(recv_state==0)
    {
        if(rdata==0x57)
        {
            recv_state=1;
        }
        else
            recv_state=0;
    }
    else  if(recv_state==1)
    {
        if(rdata==0x00)
        {
            recv_state=2;
        }
        else
            recv_state=0;
    }
    else  if(recv_state==2)
    {
        if(rdata==0xFF)
        {
            recv_state=3;
        }
        else
            recv_state=0;
    }
    else if(recv_state==3)
    {
        if(j<12)
        {
            rbuffer[j++]=rdata;
            recv_state=3;
        }
        else if(j==12)
        {
            rbuffer[j]=rdata;
            j=0;
            Process_Laser_Data(rbuffer);
            recv_state=0;
        }
    }
}
static void Process_Laser_Data(char * buffer)
{
    vector3_float position;
    int laser_raw=0;

    laser_raw =  (int32_t)(buffer[5] << 8 | buffer[6] << 16 | buffer[7] << 24) / 256;


    uint8_t dis_status=buffer[8];
    if(	current_id==0)
    {
        position.z = laser_raw/10.0;
        if( position.z > 1 && position.z < 500 &&dis_status!=14)
        {
            float lean_cosin = get_lean_angle_cosin();
            position.z *= lean_cosin;
            PositionSensorUpdatePosition( default_laser_sensor_index, position, true, -1,0,0 );
        }
        else
            PositionSensorSetInavailable( default_laser_sensor_index );
    }
    if(laser_raw > 10 && laser_raw < 5000 &&dis_status!=14)
    {
        laser_dis[current_id]=laser_raw/10.0;
    }
    else
    {
        laser_dis[current_id]=1000;
    }
    if(current_id==1)
    {
        mavlink_message_t msg_sd;
        uint8_t port_id = 1;
        const Port* port = get_Port( port_id );
        mavlink_msg_distance_sensor_pack_chan(
            1,	//system id
            MAV_COMP_ID_AUTOPILOT1,	//component id
            port_id, 	//chan
            &msg_sd,
            get_System_Run_Time() * 1e3, 	//boot ms
            1,
            500,
            laser_dis[current_id],
            MAV_DISTANCE_SENSOR_LASER,
            1,
            MAV_SENSOR_ROTATION_NONE,
            255
        );
        mavlink_msg_to_send_buffer(port->write, &msg_sd);
    }
    current_id++;
    if(current_id>1)current_id=0;
    received=1;
}