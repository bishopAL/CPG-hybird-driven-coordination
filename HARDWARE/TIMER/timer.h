#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "Basic_cpg_controller.h"
#include "Pcpg_controller.h"
#include "Delay_line.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//定时器驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/8/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
extern TIM_HandleTypeDef TIM5_Handler;      //定时器3PWM句柄 
extern TIM_HandleTypeDef TIM2_Handler;      //定时器3PWM句柄 
extern TIM_HandleTypeDef TIM7_Handler;      //定时器3PWM句柄 
extern TIM_OC_InitTypeDef TIM2_CH2Handler;

extern float T;
extern float s;
extern float delta_t;
extern float times;
extern u8 movement_flag;
extern u8 temp_movement_flag;
void TIM5_Init(u16 arr,u16 psc);    //定时器初始化
void TIM2_Init(u16 arr,u16 psc);    //定时器初始化
void TIM7_Init(u16 arr,u16 psc);    //定时器初始化
#endif

