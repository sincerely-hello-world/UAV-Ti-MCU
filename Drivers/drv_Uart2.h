#pragma once

#include <stdint.h>

extern float quan[4];

extern float chuangan[4];

extern float zx_x;
extern float zx_y;

extern float help_fire[6];


//-------------
extern unsigned char landflag;
extern unsigned char hoverflag;
extern unsigned char takeoffflag;

extern int gofly;
extern float detect_x,detect_y,detect_z;


void init_drv_Uart2();

void Uart2_Send( const uint8_t* data, uint16_t length );
uint16_t read_UART2( uint8_t* data, uint16_t length );
uint16_t UART2_DataAvailable();