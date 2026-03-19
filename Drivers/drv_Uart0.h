#pragma once

#include <stdint.h>



void init_drv_Uart0();

void Uart0_Send( const uint8_t* data, uint16_t length );
uint16_t read_UART0( uint8_t* data, uint16_t length );
uint16_t UART0_DataAvailable();