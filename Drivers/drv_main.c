#include "Basic.h"
#include "drv_main.h"

#include "drv_uDMA.h"
#include "drv_LED.h"
#include "drv_Sensors.h"
#include "drv_USB.h"
#include "drv_OLED.h"
#include "drv_Receiver.h"
#include "drv_PWMOut.h"
#include "drv_Ultrasonic.h"
#include "drv_OpticalFlow.h"
#include "drv_Uart0.h"
#include "drv_Uart2.h"
#include "drv_Uart3.h"
#include "drv_Uart7.h"
#include "drv_adc.h"
#include "drv_Laser.h"
#include "MavlinkRCProcess.h"
#include "drv_I2C1.h"

void init_Drivers()
{
    init_drv_uDMA();
    init_drv_LED();
//    init_drv_OLED();
    init_drv_PWMOut();

		PWM_PullUpAll();
		delay( 5.0f );			
		PWM_PullDownAll();//Ð£×¼¿ªÆô
 
    
    init_drv_Sensors();
    init_drv_USB();
    init_drv_Receiver();
    
    //init_drv_Ultrasonic();
    //init_drv_OpticalFlow();
    //init_drv_GPS();
	
    init_drv_Uart2();
	
    //init_drv_Laser();
		//init_drv_I2C1();
		//init_drv_Uart0();
		//init_drv_Uart3();
		//init_drv_Ground();
		//init_drv_ADC();
		
		init_drv_Uart7();
     
}