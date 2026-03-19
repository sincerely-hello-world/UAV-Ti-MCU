#pragma once

void init_drv_PWMOut();

void PWM_Out(float out[8]);
void PWM_PullDownAll();
void PWM_PullUpAll();

void PWM_PulseWidthSet_All(unsigned int width);
void PWM_PulseWidthReduce_All(unsigned int value);



void pwm_disable();
