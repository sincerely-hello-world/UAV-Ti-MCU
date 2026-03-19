#include "pid.h"

#define NULL ((void *)0)

void limited(int max, int *num)   //限幅
{
	if(*num>max)  *num = max;
	if(*num<-max) *num =-max;
}

// 初始化 PID 控制器
void pid_init(PID_t *pid, float kp, float ki, float kd, int output_max)
{
    if (pid == NULL) return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_max = output_max;//输出最大值
		pid->err_sum_max = pid->output_max ; // 积分最大值
    // 清零状态变量
    pid->target = 0;
    pid->actual = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->err_sum = 0;
    pid->output = 0;
}

int pid_calculate(PID_t *pid)
{
    if (pid == NULL) return 0;

    // 1. 计算当前误差
    pid->err = pid->target - pid->actual;

    // 2. 更新积分项
    pid->err_sum += pid->err;

    // 3. 积分限幅 
    limited(pid->err_sum_max, &pid->err_sum);

    // 4. 计算 PID 输出（整数运算）
    int p_out = (pid->Kp * pid->err);   
    int i_out = (pid->Ki * pid->err_sum);
    int d_out = (pid->Kd * (pid->err - pid->err_last));

    int output_val = p_out + i_out + d_out;

    // 5. 输出限制幅度 limited 函数限制
    limited((short)pid->output_max, &output_val);
    pid->output = output_val;

    // 6. 保存历史
    pid->err_last = pid->err;

    return pid->output;
}



