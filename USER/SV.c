#include "SV.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "gait.h"
#include "uart.h"
#define INTEGRALNUM 6
u8 target_p;
float P[4] = {0};
float p;
float set_p;
float set_speed;
float speed;
float actual_p;
float actual_p_minus;
float err;//偏差值
float err_last;//上一个偏差值
float integral_list[INTEGRALNUM] = {0};
float integral = 0;//积分值
float Kp, Ki, Kd;
float T;
// static gasControls gasControl;
u8 gas_control[4] = {1};  //LF RF LH RH
void gasControl(void)  // 2-plus 1-seal  0-minus
{
if(gas_control[0] == 2){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//PB0置0		
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//PB0置0
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 
}
if(gas_control[0] == 1){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	//PB1置1 
		
}
if(gas_control[0] == 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//PB1置1 
}
if(gas_control[1] == 2){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0置0
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//PB0置0
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 
}

if(gas_control[1] == 1){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1  		
}

if(gas_control[1] == 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//PB1置1 
}

if(gas_control[2] == 2){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//PB0置0
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 
}

if(gas_control[2] == 1){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	//PB1置1 
		
}

if(gas_control[2] == 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//PB1置1 
}

if(gas_control[3] == 2){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 
}

if(gas_control[3] == 1){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	//PB1置1 	
}

if(gas_control[3] == 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//PB1置1 
}
}
u8 sv_flag[4] = {0}; // lf, rf, lh, rh; 0: close, 1: open
u8 sv_plus_flag[5] = {0};
u8 sv_minus_flag[5] = {0};
void update_integral_list(float update_value)
{
	for(int i = 0; i<INTEGRALNUM; i++)
	{
		if(i != INTEGRALNUM-1)
		integral_list[i] = integral_list[i+1];
		else
		integral_list[i] = update_value;
	}
}

void SV_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOC_CLK_ENABLE();					//开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|
										 GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|
	                   GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12;			//PB0，1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;         			//上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;  	//高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);     		//初始化GPIOB.0和GPIOB.1

    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_Initure.Pin=GPIO_PIN_8;
		GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;         			//上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW;  	//高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);

}
void SV_test1(void)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//PB1置1 
		HAL_Delay(500);
}
void SV_test2(void)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	//PB1置1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	//PB1置1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//PB0置0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 		
		HAL_Delay(500);
}
void sv_switch(void)
{	
	for(int i=0; i<4; i++)
	{
		if(sv_plus_flag[0] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);			
		if(sv_plus_flag[0] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
		if(sv_plus_flag[1] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);			
		if(sv_plus_flag[1] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);			
		if(sv_plus_flag[2] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);			
		if(sv_plus_flag[2] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);			
		if(sv_plus_flag[3] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);			
		if(sv_plus_flag[3] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	
		if(sv_plus_flag[4] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);			
		if(sv_plus_flag[4] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	
		
		if(sv_minus_flag[0] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);			
		if(sv_minus_flag[0] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);			
		if(sv_minus_flag[1] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);			
		if(sv_minus_flag[1] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);			
		if(sv_minus_flag[2] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);			
		if(sv_minus_flag[2] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);			
		if(sv_minus_flag[3] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);			
		if(sv_minus_flag[3] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
		if(sv_minus_flag[4] == 1)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);			
		if(sv_minus_flag[4] == 0)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);				
	}
}
void pid_value_init(void)
{
  Kp = 2;
  Ki = 0.2;
  Kd = 2;
  err = 0;
	err_last = 0;
	integral = 0;
	target_p = 50;
	actual_p_minus=-50;
}

void SV_ESTIMATE(void)
{
	for(int i=0; i<4; i++)
	{
		if(sv_flag[i]==1)
		{			
			P[3] = 80*(3.3*adc1/65536-1.45);
			P[1] = 80*(3.3*adc2/65536-1.45);
			P[0] = 80*(3.3*adc3/65536-1.45);
			P[2] = 80*(3.3*adc4/65536-1.45);
			actual_p = P[i];
			
			err = target_p - actual_p;
			update_integral_list(err);
			for(int i=0; i<INTEGRALNUM; i++)
			{
				integral += integral_list[i];
			}
			p = Kp * err + Ki*integral + Kd*(err - err_last);
			err_last = err;
			actual_p = p + actual_p;
			speed = 1.5*actual_p-5;
			TIM_SetCompare1(&TIM16_Handler,speed);
			integral = 0;
			break;
		}
	}
}
int time = 0;
void SV_ESTIMATE_sv(void)
{
	for(int i=0; i<4; i++)
	{
		if(sv_flag[i]==1)
		{
			P[3] = 80*(3.3*adc1/65536-1.45);
			P[1] = 80*(3.3*adc2/65536-1.45);
			P[0] = 80*(3.3*adc3/65536-1.45);
			P[2] = 80*(3.3*adc4/65536-1.45);
			actual_p = P[i];			
			if(actual_p > target_p)times++;
			if(times>=1)sv_plus_flag[i] = 0;
			sv_switch();
		}
		if(sv_minus_flag[i]==1)
		{
			P[3] = 80*(3.3*adc1/65536-1.45);
			P[1] = 80*(3.3*adc2/65536-1.45);
			P[0] = 80*(3.3*adc3/65536-1.45);
			P[2] = 80*(3.3*adc4/65536-1.45);
			actual_p = P[i];			
			if(actual_p > target_p)times++;
			if(times>=1)sv_minus_flag[i] = 0;
			sv_switch();
		}
	}
}
void SV_ESTIMATE_minus(void)
{
	for(int i=0; i<4; i++)
	{
		if(sv_flag[i]==1)
		{			
			P[3] = 80*(3.3*adc1/65536-1.45);
			P[1] = 80*(3.3*adc2/65536-1.45);
			P[0] = 80*(3.3*adc3/65536-1.45);
			P[2] = 80*(3.3*adc4/65536-1.45);
			actual_p = P[i];
			
			err = actual_p_minus - actual_p;
			update_integral_list(err);
			for(int i=0; i<INTEGRALNUM; i++)
			{
				integral += integral_list[i];
			}
			p = Kp * err + Ki*integral + Kd*(err - err_last);
			err_last = err;
			actual_p = p + actual_p;
			speed = -1.5*actual_p+5;
			TIM_SetCompare1(&TIM16_Handler,speed);
			integral = 0;
			break;
		}
	}
}

void air_control_trot(void)//rf rh lf lh
{
	if(s<-0.2)
		{
			sv_flag[0]=1;
			SV_ESTIMATE();		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//rf0和别的是反的		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//rh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//lf0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//lh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//all0	
		}
	if(s>=-0.2&&s<0)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//rf0和别的是反的		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//rh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//lf0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//lh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//all0
		sv_flag[1]=1;
		SV_ESTIMATE();		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	
	}
	if(s>=0&&s<=T/8)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//保险
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			Read_All_Ad();
			sv_flag[1]=1;
			SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);//rf lh up
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	}
	if(3*s>T/16&&s<=5*T/16)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);		
	}	
	if(s>8*T/32&&s<=15*T/32)
	{
		Read_All_Ad();
		sv_flag[1]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	}
	if(s>15*T/32&&s<=17*T/32)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	}
	if(s>19*T/32&&s<=21*T/32)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();
	}
	if(s>24*T/32&&s<=26*T/32)
	{
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	}
	if(s>14*T/16&&s<=T)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	
		Read_All_Ad();
		sv_flag[1]=1;
		SV_ESTIMATE();
	}
}
	

void air_control_trot_zlh(void)
{
if(s<-0.2)
		{
			sv_flag[0]=1;
			SV_ESTIMATE();		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//rf0和别的是反的		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//rh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//lf0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//lh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//all0	
		}
	if(s>=-0.2&&s<0)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//rf0和别的是反的		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//rh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//lf0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//lh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//all0
		sv_flag[1]=1;
		SV_ESTIMATE();		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	
	}
	if(s>=0&&s<=T/8)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//保险
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			Read_All_Ad();
			sv_flag[1]=1;
			SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);//rf lh up
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	}
	if(3*s>T/16&&s<=5*T/16)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();

	}	
	if(s>8*T/32&&s<=15*T/32)
	{
		Read_All_Ad();
		sv_flag[1]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	}
	if(s>16*T/32&&s<=18*T/32)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	}
	if(s>19*T/32&&s<=21*T/32)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();
	}
	if(s>27*T/32&&s<=28*T/32)
	{
		Read_All_Ad();
		sv_flag[0]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	}
	if(s>14*T/16&&s<=T)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	
		Read_All_Ad();
		sv_flag[1]=1;
		SV_ESTIMATE();
	}


}
void sv_control_trot(void)
{
	if(s<-0.2)
	{
		sv_flag[0]=1;
		SV_ESTIMATE();
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//rf0和别的是反的		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//rh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//lf0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//lh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//all0	
	}
	if(s>=-0.2&&s<0)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//rf0和别的是反的		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//rh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//lf0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//lh0		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//all0
		sv_minus_flag[1] = 1;  // rf
		sv_minus_flag[2] = 1;  // lh
		sv_minus_flag[4] = 1;  // main
		SV_ESTIMATE_sv();
	}
}
void air_control_tripod(void)
{
	if(s<-0.2)
		{
		
			sv_flag[1]=1;
			SV_ESTIMATE();
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//rf0和别的是反的		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//rh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//lf0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//lh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//all0	
		}
	if(s>=-0.2&&s<0)
	{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//rf0和别的是反的		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//rh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//lf0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//lh0		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//all0				
	}
	if(s>=0&&s<=T/16)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
		sv_flag[2]=0;

	}
	if(s>3*T/32&&s<=4*T/32)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	}
	if(s>3*T/16&&s<=5*T/16)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		sv_flag[1]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	if(s>5*T/16&&s<=7*T/16)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
		sv_flag[1]=0;
	}
	if(s>7*T/16&&s<=17*T/32)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		sv_flag[3]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
	}
	if(s>17*T/32&&s<=9*T/16)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);

	sv_flag[3]=0;
	}
	if(s>11*T/16&&s<=25*T/32)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		sv_flag[0]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
	}
	if(s>25*T/32&&s<=13*T/16)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			sv_flag[0]=0;
	}
	if(s>15*T/16&&s<=T)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		sv_flag[2]=1;
		SV_ESTIMATE();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);

	}

}
