
#include "drv_Uart7.h"

#include "Basic.h"
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
#include "Sensors.h"
#include "InteractiveInterface.h"
#include "nlink_utils.h"
#include "nlink_tofsense_frame0.h"
#include "drv_Uart3.h"
 
// 串口中断
static void UART7_Handler();
//static void UART7_Listen(char data);
void T265_Handler(char rdata);

/*发送缓冲区*/
#define TX_BUFFER_SIZE 20
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static RingBuf_uint8_t Tx_RingBuf;
/*发送缓冲区*/

///*接收缓冲区*/
#define RX_BUFFER_SIZE 24
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static RingBuf_uint8_t Rx_RingBuf;
///*接收缓冲区*/

void init_drv_Uart7()
{
    // 使能Uart7引脚（Rx:PE0 Tx:PE1）
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // 使能UART7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

    // PD7解锁
    //    GPIOD->LOCK = GPIO_LOCK_KEY;
    //    GPIOD->CR |= (1 << 7);
    //    GPIOD->LOCK = 0;

    // 配置GPIO
    GPIOPinConfigure(GPIO_PE0_U7RX);
    GPIOPinConfigure(GPIO_PE1_U7TX);
    GPIOPinTypeUART(GPIOE_BASE, GPIO_PIN_0 | GPIO_PIN_1); // GPIO的UART模式配置

    // 配置Uart
    UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 57600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART7_BASE);

    // 初始化缓冲区
    RingBuf_uint8_t_init(&Tx_RingBuf, tx_buffer, TX_BUFFER_SIZE);
    RingBuf_uint8_t_init(&Rx_RingBuf, rx_buffer, RX_BUFFER_SIZE);

    // 配置串口接收中断
    UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX); // 使能UART7发送接收中断
    UARTIntRegister(UART7_BASE, UART7_Handler);                         // UART中断地址注册

    // 配置DMA发送
    uDMAChannelControlSet(UDMA_PRI_SELECT | UDMA_CH21_UART7TX,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
    UARTDMAEnable(UART7_BASE, UART_DMA_TX);
    UARTIntRegister(UART7_BASE, UART7_Handler);
    uDMAChannelAssign(UDMA_CH21_UART7TX);

    // 打开中断
    IntPrioritySet(INT_UART7, INT_PRIO_7);
    IntEnable(INT_UART7);
    //		uint8_t send_array[1] = {85};
    //		Uart7_Send(send_array, 1);
    PositionSensorRegister(default_vision_position_index,
                           Position_Sensor_Type_RelativePositioning,
                           Position_Sensor_DataType_s_xyz,
                           Position_Sensor_frame_ENU,
                           0.1f,
                           0, 0);
}

void Uart7_Send(const uint8_t *data, uint16_t length)
{
    IntDisable(INT_UART7);

    // 获取剩余的缓冲区空间
    int16_t buffer_space = RingBuf_uint8_t_get_Freesize(&Tx_RingBuf);
    // 获取DMA中待发送的字节数
    int16_t DMA_Remain = uDMAChannelSizeGet(UDMA_CH21_UART7TX);

    // 计算要发送的字节数
    int16_t max_send_count = buffer_space - DMA_Remain;
    if (max_send_count < 0)
        max_send_count = 0;
    uint16_t send_count = (length < max_send_count) ? length : max_send_count;

    // 将待发送字节压入缓冲区
    RingBuf_uint8_t_push_length(&Tx_RingBuf, data, send_count);
    //	for( uint8_t i = 0 ; i < send_count ; ++i )
    //		RingBuf_uint8_t_push( &Tx_RingBuf , data[i] );

    // 获取DMA发送是否完成
    if (uDMAChannelIsEnabled(UDMA_CH21_UART7TX) == false)
    {
        // DMA已完成
        // 可以继续发送
        uint16_t length;
        uint8_t *p = RingBuf_uint8_t_pop_DMABuf(&Tx_RingBuf, &length);
        if (length)
        {
            uDMAChannelTransferSet(UDMA_PRI_SELECT | UDMA_CH21_UART7TX,
                                   UDMA_MODE_BASIC, p, (void *)&UART7->DR, length);
            uDMAChannelEnable(UDMA_CH21_UART7TX);
        }
    }
    IntEnable(INT_UART7);
}
uint8_t rdata_test;
static void UART7_Handler()
{
    UARTIntClear(UART7_BASE, UART_INT_OE | UART_INT_RT);
    UARTRxErrorClear(UART7_BASE);

    while ((UART7->FR & (1 << 4)) == false)
    {
        // 接收
        // uint32_t getone = UARTCharGet(UART7_BASE);
        uint8_t rdata = UART7->DR & 0xff;
        rdata_test = rdata;
        T265_Handler(rdata);
		
       
    }

    if (uDMAChannelIsEnabled(UDMA_CH21_UART7TX) == false)
    {
        uint16_t length;
        uint8_t *p = RingBuf_uint8_t_pop_DMABuf(&Tx_RingBuf, &length);
        if (length)
        {
            uDMAChannelTransferSet(UDMA_PRI_SELECT | UDMA_CH21_UART7TX,
                                   UDMA_MODE_BASIC, p, (void *)&UART7->DR, length);
            uDMAChannelEnable(UDMA_CH21_UART7TX);
        }
    }
}

uint16_t read_UART7(uint8_t *data, uint16_t length)
{
    IntDisable(INT_UART7);
    uint8_t read_bytes = RingBuf_uint8_t_pop_length(&Rx_RingBuf, data, length);
    IntEnable(INT_UART7);
    return read_bytes;
}

uint16_t UART7_DataAvailable()
{
    IntDisable(INT_UART7);
    uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read(&Rx_RingBuf);
    IntEnable(INT_UART7);
    return bytes2read;
}

#define T265_HEAD 0x54 // T265头字母
#define T265_LEN 18
// new 5/12
#define T265_READY 0x52 // T265 already标志

static uint8_t info[T265_LEN]; // 20

static int T265_long = 0;
static int T265_Lock = 1; // 锁定T265词条
unsigned char think_look = 0;
float t265_x = 0, t265_y = 0, t265_z = 0;
unsigned char T265_is_ready = 0;
// 54                     首字符T
// 2B 31 35 37 37 39      +02862 028.62
// 2B 30 35 35 30 30      +02490 024.90
// 2B 30 35 35 30 30      +00089 000.89

static int cnt_C = 0;

volatile vector3_float Position_data;

void T265_Handler(char rdata)  //uart7 接linux uart3, 接受实时的t265传感器数据
{
    think_look = rdata;
    if (T265_Lock == 0)
    {
        if (T265_long < T265_LEN)
        {
            info[T265_long] = think_look;
        }
        if (T265_long == T265_LEN-1)
        {    
							cnt_C++;
							if(cnt_C == 100)
							{
								cnt_C=0;
								Uart7_Send("6",1);
							}					
					// 0-17 共18位数据 //计算的时候用的是非十六进制，比如31转化为49进行计算
					// t265的数据帧- 置信度[3],坐标(x:0.2862366968765855,y:0.24901144337642472),高度[0.008999999999999994]
					// 要发送数据 hex_str:58 2B 30 32 38 36 32 2B 30 32 34 39 30 2B 30 30 30 38 39 35  即 data_bytes:X+02862+02490+000895  
					// X为数据头， 然后接三串数字，单位cm, 
					// 如+02862， 转化为：028.62 |  +02490 转化为：024.90 | +00089 转化为 000.89  
            t265_x = (44 - info[0]) * 1.0 * ((info[1] - 48) * 100 + (info[2] - 48) * 10 + (info[3] - 48) * 1 + (info[4] - 48) * 0.1 + (info[5] - 48) * 0.01);       // 带符号的数据字符串与字符转换
            t265_y = (44 - info[6]) * 1.0 * ((info[7] - 48) * 100 + (info[8] - 48) * 10 + (info[9] - 48) * 1 + (info[10] - 48) * 0.1 + (info[11] - 48) * 0.01);     // 带符号的数据字符与字符串转换
            t265_z = (44 - info[12]) * 1.0 * ((info[13] - 48) * 100 + (info[14] - 48) * 10 + (info[15] - 48) * 1 + (info[16] - 48) * 0.1 + (info[17] - 48) * 0.01); // 带符号的数据字符与字符串转换
 
            Position_data.x = t265_x;
            Position_data.y = t265_y;
            Position_data.z = t265_z;

            PositionSensorUpdatePosition(default_vision_position_index, Position_data, true,0, 0, 0);
//					bool PositionSensorUpdatePosition( unsigned char index, vector3_float position, bool available, float delay, float xy_trustD, float z_trustD );
				}
        T265_long++;
        if (T265_long == T265_LEN)
        {
            T265_long = 0;
            T265_Lock = 1;
						T265_is_ready = 1;
        }
    }
    else if (think_look == T265_HEAD)
    {
        T265_Lock = 0;
    }
    else if (think_look == T265_READY)
    {
        T265_is_ready = 1;
    }
}


