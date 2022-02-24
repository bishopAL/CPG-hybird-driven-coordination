#include "sys.h"
#include <stdlib.h>
#include "string.h"
#include "delay.h"
#include "uart.h" 
#include "pwm.h"
#include "timer.h"
#include "adc.h"
#include "can.h"
#include "gait.h"
#include "SV.h"
#include "main.h"
#include "gait.h"

struct Basic_cpg_controller cpg;
struct Pcpg_controller pcpg_1;
struct Pcpg_controller pcpg_2;
struct Pcpg_controller pcpg_pneumatic_pos;
struct Pcpg_controller pcpg_pneumatic_neg;
struct Delay_line pneumatic_neg_delay;
struct Delay_line pneumatic_pos_delay;
struct Delay_line joint23_delay;
struct Delay_line joint1_delay;
struct Delay_line joint0_delay;

int main(void)
{	
	u8 c=0;	
	u8 d=255;
	Cache_Enable();                		//打开L1-Cache	
	HAL_Init();				        	//初始化HAL库
	Stm32_Clock_Init(160,5,2,4);  	    //设置时钟,400Mhz 
	delay_init(400);		//延时初始化
	FDCAN1_Mode_Init(5,8,31,8,FDCAN_MODE_NORMAL);  //普通测试
	HAL_Delay(100);	
	MY_UART7_Init();
	SV_Init();//电磁阀与泵初始化
	pid_value_init();//泵的PID
	TIM1_PWM_Init(10000-1,200-1);//4分频 100M/200=500k的计数频率，ARR自动重装载为10000，PSC那么PWM频率为500k/10k=50HZ
	TIM3_PWM_Init(10000-1,200-1); 
	TIM4_PWM_Init(10000-1,200-1);
	TIM12_PWM_Init(10000-1,200-1); 
	TIM15_PWM_Init(10000-1,200-1); 	
	TIM16_PWM_Init(200-1,10000-1);//pump
	MX_ADC1_Init();
	TIM2_Init(10000-1,5000-1);	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	StartAngleInit1();
	InitRobotPosion();
	HAL_Delay(1000);
	SV_test1();
	HAL_Delay(500);
	SV_test2();
	// CPG init
	
	CLASS_BCC_Basic_cpg_controller_Ini(&cpg);
	CLASS_BCC_setParameter(&cpg, 0.885, -0.59268, 0, 1.4, 0.18, 0.025); // 20 Hz 50 ms for 1 period

	CLASS_PC_Pcpg_controller_Ini(&pcpg_1);
	CLASS_PC_setParameter(&pcpg_1, PCPG_1_SLOPE,PCPG_LIFT_THRES);  // lift motor        CPG_1_SLOPE,PCPG_LIFT_THRES
	
	CLASS_PC_Pcpg_controller_Ini(&pcpg_2);
	CLASS_PC_setParameter(&pcpg_2, PCPG_2_SLOPE,PCPG_THRES);  // swing phase & stance phase     PCPG_2_SLOPE,PCPG_THRES
	
	CLASS_PC_Pcpg_controller_Ini(&pcpg_pneumatic_pos);
	CLASS_PC_setParameter(&pcpg_pneumatic_pos, PCPG_PP_SLOPE,PCPG_PP_THRES);  // PCPG_PP_SLOPE,PCPG_PP_THRES
	CLASS_PC_Pcpg_controller_Ini(&pcpg_pneumatic_neg);
	CLASS_PC_setParameter(&pcpg_pneumatic_neg, PCPG_PN_SLOPE,PCPG_PN_THRES);  // PCPG_PP_SLOPE,PCPG_PN_THRES

	
	CLASS_DL_Delay_line_Ini(&pneumatic_neg_delay);
	CLASS_DL_setParameter(&pneumatic_neg_delay, DELAYSIZE);
	
	CLASS_DL_Delay_line_Ini(&pneumatic_pos_delay);
	CLASS_DL_setParameter(&pneumatic_pos_delay, DELAYSIZE);
	
	CLASS_DL_Delay_line_Ini(&joint23_delay);
	CLASS_DL_setParameter(&joint23_delay, DELAYSIZE);

	CLASS_DL_Delay_line_Ini(&joint1_delay);
	CLASS_DL_setParameter(&joint1_delay, DELAYSIZE);
	
	CLASS_DL_Delay_line_Ini(&joint0_delay);
	CLASS_DL_setParameter(&joint0_delay, DELAYSIZE);
	
	for(int ti=0; ti<200; ti++)
	{
		cpg_run(0);
	}
	
	TIM5_Init(10000-1,500-1);
	while(1)
	{	
//		 float t;
//		 t=0;
//		 while(t<100)
//		 {
//		Read_All_Ad(); 
//		sv_flag[2]=1;
//		SV_ESTIMATE();
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//		HAL_Delay(20);
		HAL_UART_Transmit(&huart7, &adc_3_H,sizeof(adc_1_H),0xFFFF);		
		HAL_UART_Transmit(&huart7, &adc_3_L,sizeof(adc_1_H),0xFFFF);
////		 HAL_UART_Transmit(&huart7, &d,1,0xFFFF);
////		 HAL_UART_Transmit(&huart7, &adc_3_H,sizeof(adc_1_H),0xFFFF);		
////		 HAL_UART_Transmit(&huart7, &adc_3_L,sizeof(adc_1_H),0xFFFF);
//		HAL_UART_Transmit(&huart7, &c,1,0xFFFF);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//		 t+=1;
//		 }
//			 HAL_Delay(1000);
//		 while(t>=100&&t<200)
//		 {
//			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	
//			 TIM_SetCompare1(&TIM16_Handler,199);
//			HAL_Delay(20);
//			t+=1;
//		 }
//		HAL_Delay(2000); 
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//rf0和别的是反的
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);	//rf1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//rh0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	//rh1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//lf0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);	//lf1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//lh0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	//lh1 
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//all0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//all1	
//		HAL_Delay(2000);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	//PB1置1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	//PB1置1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	//PB1置1 
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 		 
	}
}
