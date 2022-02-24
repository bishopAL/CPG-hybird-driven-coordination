/**
  ******************************************************************************
  * @file    Templates/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    21-April-2017
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "Basic_cpg_controller.h"
#include "Pcpg_controller.h"
#include "Delay_line.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#define PCPG_THRES 0.848 // control swingcd, phase pcpg signal 16 period
#define PCPG_LIFT_THRES 0.87 // control swing phase pcpg signal 16 period
#define PCPG_PP_THRES 0.87 // control pneumatic positive pcpg signal 8 period
#define PCPG_PN_THRES 0.865 // control pneumatic negetive pcpg signal 12 period
#define PCPG_1_SLOPE 20. // control the downward slope of pcpg signal joint 1
#define PCPG_2_SLOPE 2.  // joint 23
#define PCPG_PP_SLOPE 80.
#define PCPG_PN_SLOPE 80.

#define DELAYSIZE 96
#define DELAYRF 0
#define DELAYRH 24
#define DELAYLF 48
#define DELAYLH 72
#define DELAYPN 0
#define DELAYPP 14
#define DELAYSS 2

extern struct Basic_cpg_controller cpg;
extern struct Pcpg_controller pcpg_1;
extern struct Pcpg_controller pcpg_2;
extern struct Pcpg_controller pcpg_pneumatic_pos;
extern struct Pcpg_controller pcpg_pneumatic_neg;
extern struct Delay_line pneumatic_neg_delay;
extern struct Delay_line pneumatic_pos_delay;
extern struct Delay_line joint23_delay;
extern struct Delay_line joint1_delay;
extern struct Delay_line joint0_delay;

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
