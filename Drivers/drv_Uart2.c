#include "Basic.h"
#include "drv_Uart2.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"

#include "STS.h"
#include "Sensors_Backend.h"
#include "RingBuf.h"

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

// 串口中断
static void UART2_Handler();

/*发送缓冲区*/
#define TX_BUFFER_SIZE 32
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static RingBuf_uint8_t Tx_RingBuf;
/*发送缓冲区*/

///*接收缓冲区*/
#define RX_BUFFER_SIZE 24
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static RingBuf_uint8_t Rx_RingBuf;
///*接收缓冲区*/

void init_drv_Uart2()
{
    // 使能UART2引脚（Rx:PD6 Tx:PD7）
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // 使能UART2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    // PD7解锁
    GPIOD->LOCK = GPIO_LOCK_KEY;
    GPIOD->CR |= (1 << 7);
    GPIOD->LOCK = 0;

    // 配置GPIO
    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIOD_BASE, GPIO_PIN_6 | GPIO_PIN_7); // GPIO的UART模式配置

    // 配置Uart
    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 57600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART2_BASE);

    // 初始化缓冲区
    RingBuf_uint8_t_init(&Tx_RingBuf, tx_buffer, TX_BUFFER_SIZE);
    RingBuf_uint8_t_init(&Rx_RingBuf, rx_buffer, RX_BUFFER_SIZE);

    // 配置串口接收中断
    UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART2_BASE, UART2_Handler);

    // 配置DMA发送
    uDMAChannelControlSet(UDMA_PRI_SELECT | UDMA_CH13_UART2TX,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
    UARTDMAEnable(UART2_BASE, UART_DMA_TX);
    UARTIntRegister(UART2_BASE, UART2_Handler);
    uDMAChannelAssign(UDMA_CH13_UART2TX);

    // 打开中断
    IntPrioritySet(INT_UART2, INT_PRIO_5);
    IntEnable(INT_UART2);
}

void Uart2_Send(const uint8_t *data, uint16_t length)
{
    IntDisable(INT_UART2);

    // 获取剩余的缓冲区空间
    int16_t buffer_space = RingBuf_uint8_t_get_Freesize(&Tx_RingBuf);
    // 获取DMA中待发送的字节数
    int16_t DMA_Remain = uDMAChannelSizeGet(UDMA_CH13_UART2TX);

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
    if (uDMAChannelIsEnabled(UDMA_CH13_UART2TX) == false)
    {
        // DMA已完成
        // 可以继续发送
        uint16_t length;
        uint8_t *p = RingBuf_uint8_t_pop_DMABuf(&Tx_RingBuf, &length);
        if (length)
        {
            uDMAChannelTransferSet(UDMA_PRI_SELECT | UDMA_CH13_UART2TX,
                                   UDMA_MODE_BASIC, p, (void *)&UART2->DR, length);
            uDMAChannelEnable(UDMA_CH13_UART2TX);
        }
    }
    IntEnable(INT_UART2);
}

static void UART2_Listen(char data);

static void UART2_Handler()
{
    UARTIntClear(UART2_BASE, UART_INT_OE | UART_INT_RT);
    UARTRxErrorClear(UART2_BASE);
    while ((UART2->FR & (1 << 4)) == false)
    {
        // 接收
        uint8_t rdata = UART2->DR & 0xff;
        UART2_Listen(rdata);
    }

    if (uDMAChannelIsEnabled(UDMA_CH13_UART2TX) == false)
    {
        uint16_t length;
        uint8_t *p = RingBuf_uint8_t_pop_DMABuf(&Tx_RingBuf, &length);
        if (length)
        {
            uDMAChannelTransferSet(UDMA_PRI_SELECT | UDMA_CH13_UART2TX,
                                   UDMA_MODE_BASIC, p, (void *)&UART2->DR, length);
            uDMAChannelEnable(UDMA_CH13_UART2TX);
        }
    }
}



#define ListFlag 0x96
#define DownFLAG 0x97
//#define GoFlag 0x98
#define BackFlag 0x99
#define FlybackFlag 0x95

#define LISTEN_LEN 18
#define YAW_LEN 8
// 3.14 314 0.99 99     
/*
这个是根据数据量来的，之前穿杆是23
之后读取三点是33
*/



static int listen_long = 0;
static int listen_lock = 1;

static int flyagain_lock = 1;
static int flyback_lock =1;
static int down_lock = 1;

static int yaw_lock = 1;
static int yaw_long = 0;
float yaw_fix=0;
float yaw_y=0;
static int num_lock=1;

int shu_taskid = 0;

bool qifeixuke=false;
bool xiajiangxuke=false;
bool target_liscence=false;
bool yaw_liscence=false;
bool chongfeixuke=false;
bool huoyuan=false;
bool chuxianhuoyuan=false;
//bool landflag=false;

float zx_yaw=0;
float zx_x = 0;
float zx_y = 0;

//int think_listen = 0;
//int listen_ready = 0;
//float detect_x,detect_y,detect_z;

int firepoint=0;
int gofly=0;
int backfly=0;
int downfly=0;
int taskid = 0;

static int at1_state = 0; // 0:未开始, 1:'@', 2:'@1', 3:'@1:'
static char at1_buffer[1000];
static int at1_len = 0;
int point_count = 0; // 点的数量
static int point_index = 0; // 当前点的索引
int points[50][2]; // 假设最多50个点，每个点有x和y坐标

static uint8_t task_info[LISTEN_LEN]; // 20
uint8_t task_yaw[YAW_LEN]; // 20
float detect_x,detect_y,detect_z;
extern int mark;


//-------------------------
int think_listen = 0;
int listen_ready = 0;
static uint8_t task_info[LISTEN_LEN]; // 20
float detect_x=0,detect_y=0,detect_z=0;
// 47                     首字符G
// 2B 31 35 37 37 39      +02862 028.62
// 2B 30 35 35 30 30      +02490 024.90
// 2B 30 35 35 30 30      +00089 000.89
#define GoFlag 0x47
#define TakeoffFlag 0x52
#define LandFlag 0x53
#define HoverFlag 0x54
unsigned char landflag=0;
unsigned char hoverflag=0;
unsigned char takeoffflag=0;

unsigned char  cnt_2=0;
static void UART2_Listen(char data)
{
    think_listen = data;
    if (listen_lock == 0)
    {
        if (listen_long < LISTEN_LEN)
        {
            task_info[listen_long] = think_listen;
        }
        if (listen_long == LISTEN_LEN-1)                         //列表遍历完飞回原点
        { 
             detect_x = (44 - task_info[0]) * 1.0 * ((task_info[1] - 48) * 100 + (task_info[2] - 48) * 10 + (task_info[3] - 48) * 1 + (task_info[4] - 48) * 0.1 + (task_info[5] - 48) * 0.01);       // 带符号的数据字符串与字符转换
             detect_y = (44 - task_info[6]) * 1.0 * ((task_info[7] - 48) * 100 + (task_info[8] - 48) * 10 + (task_info[9] - 48) * 1 + (task_info[10] - 48) * 0.1 + (task_info[11] - 48) * 0.01);     // 带符号的数据字符与字符串转换
             detect_z = (44 - task_info[12]) * 1.0 * ((task_info[13] - 48) * 100 + (task_info[14] - 48) * 10 + (task_info[15] - 48) * 1 + (task_info[16] - 48) * 0.1 + (task_info[17] - 48) * 0.01); // 带符号的数据字符与字符串转换
						 gofly=1;
        } 
//				cnt_2++;
//				if(cnt_2 == 5)
//				{
//					cnt_2=0;
//					Uart2_Send(task_info,18); //验真成功过
//				}
        listen_long++;
        if (listen_long == LISTEN_LEN)
        {
            listen_long = 0;
            listen_lock = 1;
        }
    }
	
	if (think_listen == GoFlag)
	{
			listen_lock=0;
			listen_long = 0;
	}
	
	else if(think_listen == TakeoffFlag){
		  takeoffflag =1;
			Uart2_Send("Takeoffok",9);
	}
	else if(think_listen == LandFlag){ // MCU uart2 - Linux uart4
			landflag =1;
			Uart2_Send("Landok",6);
	}
	else if(think_listen == HoverFlag){ // MCU uart2 - Linux uart4
			hoverflag =1;
	}
}
