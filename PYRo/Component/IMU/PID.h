#include<stdint.h>
#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float feedback,expect;
	float last_feedback;
	//反馈值，设定值
	float error,last_error,last_last_error;
	//误差，上次误差，上上次误差
	float Kp,Ki,Kd;
	//比例，积分，微分常数
	float integral;
	//积分值
	float differential;
	//微分值
	float Pout,Iout,Dout;
	//比例，积分，微分输出
	float Last_Pout,Last_Iout,Last_Dout;
	//上次比例，积分，微分输出
	float Max_Integral;
	//最大积分值
	float Max_Out;
	//最大输出值
	float Out;
	//输出值
	
	uint8_t integral_separation_flag;
	//积分分离标志位
	uint8_t integral_shift_flag;
	//变速积分标志位
	uint8_t trapezoidal_integral_flag;
	//梯形积分标志位
	
	float Beta;
	//积分控制系数
	
	float integral_gate;
	//积分分离阈值
	float integral_gate_proportion_gain;
	
	float integral_upGate,integral_downGate;
	//变速积分上下限
	
	uint8_t incomplete_derivate_flag;
	//不完全积分标志位
	float Alpha;
	//Dout滤波器系数
	
}PID_t;

void PID_Init(PID_t* Init,float K_p,float K_i,float K_d,float MaxI,float MaxOut,	//Base
	uint8_t integral_separation,float integral_gate,float propotion_gain, //积分分离
	uint8_t integral_shift,float integral_upGate,float integral_downGate, //变速积分
	uint8_t trapezoidal_integral, //梯形积分
	uint8_t incomplete_derivate,float Alpha // 不完全微分
	);

void PID_Set_Feedback(PID_t*Pid,float Feedback);
void PID_Set_Expect(PID_t*Pid,float Expect);
void PID_Predict(PID_t*Pid);

#endif