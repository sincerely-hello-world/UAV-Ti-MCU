#ifndef __PID_H
#define __PID_H



typedef struct Pid_my {  // PID结构体声明
		float  Kp,Ki,Kd;   // 系数
		int target;  //in:目标值
		int actual;  //in:实际值
	
		int err;//当前偏差
	  int err_last;//上次偏差
	  int err_sum;//积分
		int err_sum_max;
		
		int output;			 //out:输出值  //可以给输出限制幅度
		int output_max;
} PID_t;

extern PID_t pidX,pidY,pidZ;
//声明函数
void pid_init(PID_t *pid, float kp, float ki, float kd, int output_max);

int pid_calculate(PID_t *pid);




#endif

