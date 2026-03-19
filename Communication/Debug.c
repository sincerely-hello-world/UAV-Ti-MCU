#include "DebugComm.h"
#include "STS.h"
#include "drv_Uart0.h"
#include "Sensors.h"
#include "drv_USB.h"
#include "MeasurementSystem.h"
#include "drv_Uart2.h"
#include "mavlink.h"
#include "Commulink.h"
#include "drv_Uart3.h"
extern float get_hover_throttle();
//extern bool Get_Guided_Mode_Enabled(void);
//extern bool Get_POS_Control_Enabled(void);
static void debug_server( unsigned int Task_ID );
void init_Debug()
{
    STS_Add_Task( STS_Task_Trigger_Mode_RoughTime, 0.05f, 0, debug_server );
}

float debug_test[10];
extern vector3_float vel_vision;
extern vector3_float pos_vision;
static void debug_server( unsigned int Task_ID )
{
    mavlink_message_t msg_sd;

    uint8_t port_id = 1;
    const Port* port = get_Port( port_id );
    if( port->write )
    {
//        mavlink_msg_debug_vect_pack_chan(
//            1,	//system id
//            MAV_COMP_ID_AUTOPILOT1,	//component id
//            port_id, 	//chan
//            &msg_sd,
//            "baro",	//name
//            get_System_Run_Time() * 1e3, 	//boot ms
//            debug_test[0],
//            debug_test[1],
//            GetPositionSensor( internal_baro_sensor_index )->position.z * 0.01f - 86 );
//        mavlink_msg_to_send_buffer(port->write, &msg_sd);

//        mavlink_msg_debug_vect_pack_chan(
//            1,	//system id
//            MAV_COMP_ID_AUTOPILOT1,	//component id
//            port_id, 	//chan
//            &msg_sd,
//            "GuangLiu",	//name
//            get_System_Run_Time() * 1e3, 	//boot ms
//            GetPositionSensor(default_optical_flow_index)->velocity.x,//debug_test[2] ,
//            GetPositionSensor(default_optical_flow_index)->velocity.y,//debug_test[3] ,
//            GetPositionSensor(default_laser_sensor_index)->position.z*0.01f );
//        mavlink_msg_to_send_buffer(port->write, &msg_sd);

        mavlink_msg_debug_vect_pack_chan(
            1,	//system id
            MAV_COMP_ID_AUTOPILOT1,	//component id
            port_id, 	//chan
            &msg_sd,
            "Vision",	//name
            get_System_Run_Time() * 1e3, 	//boot ms
            GetPositionSensor(default_vision_sensor_index)->velocity.x,//debug_test[2] ,
            GetPositionSensor(default_vision_sensor_index)->velocity.y,//GetPositionSensor(default_vision_sensor_index)->position.y,//debug_test[3] ,
            GetPositionSensor(default_laser_sensor_index)->position.z*0.01f);//pos_vision.z/100.0);//GetPositionSensor(default_vision_sensor_index)->position.z );
        mavlink_msg_to_send_buffer(port->write, &msg_sd);

        mavlink_msg_debug_vect_pack_chan(
            1,	//system id
            MAV_COMP_ID_AUTOPILOT1,	//component id
            port_id, 	//chan
            &msg_sd,
            "PWM",	//name
            get_System_Run_Time() * 1e3, 	//boot ms
            get_hover_throttle(),//debug_test[2] ,
            false,//GetPositionSensor(default_vision_sensor_index)->position.y,//debug_test[3] ,
            true);//pos_vision.z/100.0);//GetPositionSensor(default_vision_sensor_index)->position.z );
        mavlink_msg_to_send_buffer(port->write, &msg_sd);
				
//        mavlink_msg_debug_vect_pack_chan(
//            1,	//system id
//            MAV_COMP_ID_AUTOPILOT1,	//component id
//            port_id, 	//chan
//            &msg_sd,
//            "Mag",	//name
//            get_System_Run_Time() * 1e3, 	//boot ms
//            GetMagnetometer(2)->data.x,
//            GetMagnetometer(2)->data.y,
//            GetMagnetometer(2)->data.z );
//        mavlink_msg_to_send_buffer(port->write, &msg_sd);

//        mavlink_msg_debug_vect_pack_chan(
//            1,	//system id
//            MAV_COMP_ID_AUTOPILOT1,	//component id
//            port_id, 	//chan
//            &msg_sd,
//            "GPS",	//name
//            get_System_Run_Time() * 1e3, 	//boot ms
//            GethAcc(),
//            0,
//            0 );
//        mavlink_msg_to_send_buffer(port->write, &msg_sd);
    }
}