#pragma once

#include "Receiver.h"

//获取接收机
Receiver* get_Receiver_NC( RC_Type rc );

//更新接收机数据
void Receiver_Update( RC_Type _rc, bool connected, float raw_data[16], uint8_t channel_count );