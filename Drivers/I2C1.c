#include "Basic.h"
#include "drv_I2C1.h"

#include "TM4C123GH6PM.h"
#include "I2C.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"

#include "STS.h"


uint8_t shit;

static void I2C1_Transfered_Handler()
{
	I2CSlaveIntClear(I2C1_BASE);
	//I2CSlaveDataPut(I2C1_BASE, 0x32);
	shit = I2CSlaveDataGet(I2C1_BASE);
}

void init_drv_I2C1()
{
	/*配置i2c1拉取外置罗盘数据*/
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
	delay(1e-6f);
	SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
	delay(1e-6f);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //开启GPIOA电源
	delay(1e-6f);

	GPIOPinTypeI2CSCL(GPIOA_BASE, GPIO_PIN_6); // Use pin with I2C SCL peripheral
	GPIOPinTypeI2C(GPIOA_BASE, GPIO_PIN_7);	   // Use pin with I2C peripheral

	// Use alternate function
	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	delay(1e-6f);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1); //开启I2C1外设电源
	delay(1e-6f);
	I2CSlaveInit(I2C1_BASE, 0x32);
	//I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true); // Enable and set frequency to 400 kHz
	delay(1e-6f);

	//I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_FIFO_BURST_SEND_FINISH);
	delay(1e-3f);
	/*配置i2c1拉取外置罗盘数据*/

	//开启I2C1接收中断
	I2CIntRegister(I2C1_BASE, I2C1_Transfered_Handler); //中断函数注册
	//I2CMasterIntEnable(I2C1_BASE);
	I2CSlaveIntEnable(I2C1_BASE);
	IntEnable(INT_I2C1);
	IntPrioritySet(INT_I2C1, INT_PRIO_2);
}
void init_drv_Ground()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //开启GPIOA电源
	
//	GPIODirModeSet(GPIOA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);//软解设置为读入
	
	GPIOPinTypeGPIOInput(GPIOA_BASE, GPIO_PIN_6);//设置为输入模式
	
	GPIOPadConfigSet(GPIOA_BASE, GPIO_PIN_6, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);//配置为8mA弱上拉
	
}

int Read_Ground(){
	int8_t ground = -5;
	if(GPIOPinRead(GPIOA_BASE, GPIO_PIN_6) == 0){
		ground = 0;
	}
	
	return ground;
}
