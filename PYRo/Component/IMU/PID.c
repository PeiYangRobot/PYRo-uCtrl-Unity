#include"PID.h"
#include<math.h>

void PID_Init(PID_t* Init,float K_p,float K_i,float K_d,float MaxI,float MaxOut,	//Base
	uint8_t integral_separation,float integral_gate,float propotion_gain, //积分分离
	uint8_t integral_shift,float integral_upGate,float integral_downGate, //变速积分
	uint8_t trapezoidal_integral, //梯形积分
	uint8_t incomplete_derivate,float Alpha // 不完全微分
	)
	//**************************************Baseform PID_Init(pid_struc,K_array,MaxIOut,Max_Out,	0,0,	0,0,0		0,		0,0);
{
	Init->error=0.0f;
	Init->last_error=0.0f;
	Init->last_last_error=0.0f;
	Init->Pout=0.0f;
	Init->Iout=0.0f;
	Init->Dout=0.0f;
	Init->Out=0.0f;
	Init->Kp=K_p;
	Init->Ki=K_i;
	Init->Kd=K_d;
	Init->Max_Integral=MaxI;
	Init->Max_Out=MaxOut;
	
	Init->integral_separation_flag=integral_separation;
	Init->integral_gate=integral_gate;
	Init->integral_gate_proportion_gain=propotion_gain;
	
	Init->integral_shift_flag=integral_shift;
	Init->integral_upGate=integral_upGate;
	Init->integral_downGate=integral_downGate;
	
	Init->trapezoidal_integral_flag=trapezoidal_integral;
	
	Init->incomplete_derivate_flag=incomplete_derivate;
	Init->Alpha=Alpha;
}

void PID_Set_Feedback(PID_t*Pid,float Feedback)
{
	Pid->feedback=Feedback;
}
void PID_Set_Expect(PID_t*Pid,float Expect)
{
	Pid->expect=Expect;
}

void PID_Predict(PID_t*Pid)
{
	
	//误差继承及计算
	Pid->last_last_error=Pid->last_error;
	Pid->last_error=Pid->error;
	//Pid->last_error=-Pid->feedback;

	Pid->error=Pid->expect-Pid->feedback;
	
	//各个Out值继承
	Pid->Last_Pout=Pid->Pout;
	Pid->Last_Iout=Pid->Iout;
	Pid->Last_Dout=Pid->Dout;
	
	//计算Pout
	Pid->Pout=Pid->Kp*Pid->error;
	
	//积分分离
	if(Pid->integral_separation_flag==1)
	{
		
		if(fabs(Pid->error)>Pid->integral_gate)	
		{
			Pid->Beta=1.0f;
			Pid->Pout*=Pid->integral_gate_proportion_gain;
		}
		else	
		{
			Pid->Beta=1.0f;
			Pid->Pout/=Pid->integral_gate_proportion_gain;
		}
		
	Pid->integral+=Pid->error;
	
	}
	
	//变速积分
	else if(Pid->integral_shift_flag==1)
	{
		if(fabs(Pid->error)>Pid->integral_upGate)
		{
			Pid->Beta=0;
		}
		else if(fabs(Pid->error)<Pid->integral_downGate)
		{
			Pid->Beta=1;
		}
		else
		{
			Pid->Beta=(Pid->integral_upGate-fabs(Pid->error))/(Pid->integral_upGate-Pid->integral_downGate);
		}
	}
	
	//梯形积分
	else if(Pid->trapezoidal_integral_flag==1)
	{
		Pid->integral+=(Pid->error+Pid->last_error)/2;
		Pid->Beta=1.0f;
	}
	
	//普通积分
	else		
	{
		Pid->integral+=Pid->error;
		Pid->Beta=1.0f;
	}
	
	//积分限幅

	if(Pid->integral>Pid->Max_Integral)
	{
		Pid->integral=Pid->Max_Integral;
	}
	else if(Pid->integral<-Pid->Max_Integral)
	{
		Pid->integral=-Pid->Max_Integral;
	}
	
	Pid->Iout=Pid->Ki*Pid->integral*Pid->Beta;
	
	//微分计算
	Pid->differential=Pid->error-Pid->last_error;
	Pid->Dout=Pid->Kd*Pid->differential;
	
	//不完全微分
	if(Pid->incomplete_derivate_flag==1)
	{
		Pid->Dout=(1-Pid->Alpha)*Pid->Dout+Pid->Alpha*Pid->Last_Dout;
		//对Dout一阶低通滤波
	}
	
	
	
	//计算输出
	Pid->Out=Pid->Pout+Pid->Iout+Pid->Dout;
	
	//输出限幅
	if(Pid->Out>Pid->Max_Out)
	{
		Pid->Out=Pid->Max_Out;
	}
	else if(Pid->Out<-Pid->Max_Out)
	{
		Pid->Out=-Pid->Max_Out;
	}
	
}

