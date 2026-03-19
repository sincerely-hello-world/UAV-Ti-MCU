#include "Basic.h"
#include "drv_Uart0.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"

#include "Commulink.h"
#include "STS.h"
#include "Sensors_Backend.h"
#include "RingBuf.h"
#include "drv_Laser.h"
#include "TM4C123GH6PM.h"
#include "uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "Timer.h"
#include "udma.h"

//串口中断
static void UART0_Handler();
void K210_Handler2(char rdata);

/*发送缓冲区*/
#define TX_BUFFER_SIZE 12
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static RingBuf_uint8_t Tx_RingBuf;
/*发送缓冲区*/

/*接收缓冲区*/
#define RX_BUFFER_SIZE 24
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static RingBuf_uint8_t Rx_RingBuf;
/*接收缓冲区*/

uint8_t k210_2_rc1 = 0;
bool k210_2_rcflag = false;
uint8_t mnist_num[4];
uint8_t mnist_count = 0;


void init_drv_Uart0()
{
    //使能Uart0引脚（Rx:PA0 Tx:PA1）
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //使能UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //配置GPIO
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置

    //配置Uart
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
                        (UART_CONFIG_WLEN_8
                         | UART_CONFIG_STOP_ONE
                         |	UART_CONFIG_PAR_NONE));
    UARTFIFOEnable( UART0_BASE );
	
		//初始化缓冲区
    RingBuf_uint8_t_init( &Tx_RingBuf, tx_buffer, TX_BUFFER_SIZE );
    RingBuf_uint8_t_init( &Rx_RingBuf, rx_buffer, RX_BUFFER_SIZE );
	
		//配置串口接收中断
    UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);//使能UART0发送接收中断
    UARTIntRegister(UART0_BASE,UART0_Handler);//UART中断地址注册
		
		//配置DMA发送
    uDMAChannelControlSet( UDMA_PRI_SELECT | UDMA_CH9_UART0TX, \
                           UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 );
    UARTDMAEnable( UART0_BASE, UART_DMA_TX );
    UARTIntRegister( UART0_BASE, UART0_Handler );
    uDMAChannelAssign(UDMA_CH9_UART0TX  );

    //打开中断
    IntPrioritySet(INT_UART0, INT_PRIO_7);
		IntEnable( INT_UART0 );

}

void Uart0_Send( const uint8_t* data, uint16_t length )
{
    IntDisable( INT_UART0 );

    //获取剩余的缓冲区空间
    int16_t buffer_space = RingBuf_uint8_t_get_Freesize( &Tx_RingBuf );
    //获取DMA中待发送的字节数
    int16_t DMA_Remain = uDMAChannelSizeGet( UDMA_CH9_UART0TX );

    //计算要发送的字节数
    int16_t max_send_count = buffer_space - DMA_Remain;
    if( max_send_count < 0 )
        max_send_count = 0;
    uint16_t send_count = ( length < max_send_count ) ? length : max_send_count;

    //将待发送字节压入缓冲区
    RingBuf_uint8_t_push_length( &Tx_RingBuf, data, send_count );
//	for( uint8_t i = 0 ; i < send_count ; ++i )
//		RingBuf_uint8_t_push( &Tx_RingBuf , data[i] );

    //获取DMA发送是否完成
    if( uDMAChannelIsEnabled( UDMA_CH9_UART0TX ) == false )
    {
        //DMA已完成
        //可以继续发送
        uint16_t length;
        uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf, &length );
        if( length )
        {
            uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH9_UART0TX, \
                                    UDMA_MODE_BASIC, p, (void*)&UART0->DR, length );
            uDMAChannelEnable( UDMA_CH9_UART0TX );
        }
    }
    IntEnable( INT_UART0 );
}

static void UART0_Handler()
{
    UARTIntClear( UART0_BASE, UART_INT_OE | UART_INT_RT );
    UARTRxErrorClear( UART0_BASE );
		while( ( UART0->FR & (1<<4) ) == false	)
    {
        //接收
        uint8_t rdata = UART0->DR & 0xff;
				K210_Handler2(rdata);
				
				//k210_2_rc1=rdata;
        //RingBuf_uint8_t_push( &Rx_RingBuf , rdata );
    }

    if( uDMAChannelIsEnabled( UDMA_CH9_UART0TX ) == false )
    {
        uint16_t length;
        uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf, &length );
        if( length )
        {
            uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH9_UART0TX, \
                                    UDMA_MODE_BASIC, p, (void*)&UART0->DR, length );
            uDMAChannelEnable( UDMA_CH9_UART0TX );
        }
    }
}

uint16_t read_UART0( uint8_t* data, uint16_t length )
{
    IntDisable( INT_UART0 );
    uint8_t read_bytes = RingBuf_uint8_t_pop_length( &Rx_RingBuf, data, length );
    IntEnable( INT_UART0 );
    return read_bytes;
}

uint16_t UART0_DataAvailable()
{
    IntDisable( INT_UART0 );
    uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Rx_RingBuf );
    IntEnable( INT_UART0 );
    return bytes2read;
}

void K210_Handler2(char rdata)
{
    static int recv_state = 0;
		static int tmpx=0;
    /*状态机*/
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
				if (rdata == 0x3c){
					k210_2_rc1 = tmpx;
					k210_2_rcflag = true;
					if(k210_2_rc1>=1 && k210_2_rc1<=4){
						mnist_num[k210_2_rc1-1]++;
						mnist_count++;
					}
//					uint8_t send_array[3] = { 0x73, k210_2_rc1, 0x3c };
//					Uart0_Send(send_array, 3);
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