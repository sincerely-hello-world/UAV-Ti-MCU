#pragma once


 

#include <stdint.h>
#include "vector_3.h"

extern float t265_x;
extern float t265_y;
extern float t265_z;
//extern unsigned char think_look;
extern unsigned char T265_is_ready;

// 当前位置信息
extern volatile vector3_float Position_data;

void init_drv_Uart7();

void Uart7_Send( const uint8_t* data, uint16_t length );
uint16_t read_UART7( uint8_t* data, uint16_t length );
uint16_t UART7_DataAvailable();