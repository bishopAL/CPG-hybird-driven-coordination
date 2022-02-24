#ifndef _CAN_H
#define _CAN_H
#include "sys.h"
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//FDCAN��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2018/6/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//FDCAN1����RX0�ж�ʹ��
#define FDCAN1_RX0_INT_ENABLE	0		//0,��ʹ��;1,ʹ��.
extern u8 mesg[4];
extern u8 canbuf1[8];
extern u8 canbuf2[8];
extern u8 canbuf3[4];
u8 FDCAN1_Mode_Init(u16 presc,u8 ntsjw,u16 ntsg1,u8 ntsg2,u32 mode);
u8 FDCAN1_Send_Msg(u8* msg,u32 len);
u8 FDCAN1_Receive_Msg(u8 *buf);
#endif
