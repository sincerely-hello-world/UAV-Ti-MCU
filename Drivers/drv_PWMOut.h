#pragma once

void init_drv_PWMOut();

void PWM_Out(float out[8]);
void PWM_PullDownAll();
void PWM_PullUpAll();

void PWM_PulseWidthSet_All(unsigned int width);
void PWM_PulseWidthReduce_All(unsigned int value);

/* 停掉所有的pwm输出，飞机瞬间失去动力！！！*/
void Disable_Motor();

/* 如果要二次起飞, 并且之前 Disable_PWMout， 就必须重置标志位，让飞机能输出动力 */
void Enable_Motor();

unsigned char get_Motor_status();


 
