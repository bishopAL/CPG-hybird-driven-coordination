#ifndef _SV_H
#define _SV_H
#include "sys.h"

void SV_Init(void); //LED初始化函数
void SV_ESTIMATE(void);// 判断函数
void SV_ESTIMATE_minus(void);
void SV_ESTIMATE_sv(void);
void sv_stich(void);
void SV_test1(void);
void SV_test2(void);
extern u8 sv_flag[4];
extern u8 sv_plus_flag[5];
extern u8 sv_minus_flag[5];
void pid_value_init(void);
void update_integral_list(float update_value);  // 积分项更新函数
void air_control_trot(void);
void air_control_trot_zlh(void);
  // typedef enum{LF0, LF1, LF2, RF0, RF1, RF2, LH0, LH1, LH2, RH0, RH1, RH2} gasControls;
extern u8 gas_control[4];  // LF RF LH RH
void gasControl(void);
void sv_control_trot(void);
void air_control_tripod(void);
#endif