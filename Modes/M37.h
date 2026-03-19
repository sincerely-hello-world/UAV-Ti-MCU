#pragma once

#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>

#include "AC_Math.h"
#include "Receiver.h"
#include "drv_Uart2.h"
#include "drv_Uart7.h"
#include "drv_Uart0.h"
#include "drv_PWMOut.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"

#include "sysctl.h"
#include "hw_memmap.h"
#include "pin_map.h"
#include "gpio.h"
#include "pwm.h"

#include "drv_Uart2.h"

#include "drv_LED.h"

#define Pi 3.1416

#define SIZE_UAV_X 30.0
#define SIZE_UAV_Y 30.0

extern const Mode M37_Liu;
