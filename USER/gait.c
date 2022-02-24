#include <math.h>
#include "pwm.h"	
#include "capture.h"
#include "adc.h"
#include "SV.h"
#include "gait.h"
#include <sys.h>
#include "uart.h"
#include "timer.h"
#include "main.h"
const short L1=38;//38
const short L2=47;//47
const short L3=22;//22
#define DE 1
KinematicsArm KMGecko;
int i;
float F1=38;
float F2=47;
float F3=22;


float x[4]={L2, L2, -L2, -L2}; // present position
float y[4]={L1, -L1, L1, -L1};
float z[4]={-L3, -L3, -L3, -L3};
float x_b=62;//62
float y_b=30.24;//30.24
float D;
float B;	

float D_onestep;
float L_onestep=20;
float W_onestep=0;
float delta=0;
float H_onestep=15;
float stime=0;
float Time=4;
float cx[4]={0};
float cy[4]={0};
float cz[4]={0};
float delta;

void cpg_run(bool runFlag)
{
	float a[4][3], theta[4][3];
	CLASS_BCC_run(&cpg);
	float op1, op2;
	op1 = CLASS_BCC_getSignal(&cpg, 1);
	op2 = CLASS_BCC_getSignal(&cpg, 2); 

	CLASS_PC_run(&pcpg_1, op1, op2);
	CLASS_PC_run(&pcpg_2, op1, op2);
	CLASS_PC_run(&pcpg_pneumatic_pos, op1, op2);
	CLASS_PC_run(&pcpg_pneumatic_neg, op1, op2);
	
	CLASS_DL_writeIn(&pneumatic_neg_delay, CLASS_PC_getSignal(&pcpg_pneumatic_neg, 2));
	CLASS_DL_writeIn(&pneumatic_pos_delay, CLASS_PC_getSignal(&pcpg_pneumatic_pos, 2));
	
	CLASS_DL_writeIn(&joint1_delay, CLASS_PC_getSignal(&pcpg_1, 2));
	CLASS_DL_writeIn(&joint23_delay, CLASS_PC_getSignal(&pcpg_2, 2));

	float pcpg_signal[16];
	pcpg_signal[0] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYRF + DELAYPN); // RF RH LF LH
	pcpg_signal[1] = CLASS_DL_readFr(&joint1_delay, DELAYRF + DELAYSS);
	pcpg_signal[2] = CLASS_DL_readFr(&joint23_delay, DELAYRF + DELAYSS);
	pcpg_signal[3] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYRF + DELAYPP);
	pcpg_signal[4] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYRH + DELAYPN);
	pcpg_signal[5] = CLASS_DL_readFr(&joint1_delay, DELAYRH + DELAYSS);
	pcpg_signal[6] = CLASS_DL_readFr(&joint23_delay, DELAYRH + DELAYSS);
	pcpg_signal[7] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYRH + DELAYPP);
	pcpg_signal[8] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYLF + DELAYPN);
	pcpg_signal[9] = CLASS_DL_readFr(&joint1_delay, DELAYLF + DELAYSS);
	pcpg_signal[10] = CLASS_DL_readFr(&joint23_delay, DELAYLF + DELAYSS);
	pcpg_signal[11] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYLF + DELAYPP);
	pcpg_signal[12] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYLH + DELAYPN);
	pcpg_signal[13] = CLASS_DL_readFr(&joint1_delay, DELAYLH + DELAYSS);
	pcpg_signal[14] = CLASS_DL_readFr(&joint23_delay, DELAYLH + DELAYSS);
	pcpg_signal[15] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYLH + DELAYPP);

	CLASS_DL_step_one(&pneumatic_neg_delay);
	CLASS_DL_step_one(&pneumatic_pos_delay);
	CLASS_DL_step_one(&joint0_delay);
	CLASS_DL_step_one(&joint1_delay);
	CLASS_DL_step_one(&joint23_delay);
	
//			a[0][0] = -asin(L3 / sqrt(z[0] * z[0] + y[0] * y[0]) ) - atan2(z[0],y[0]);
//			a[0][1] = (asin((y[0] * y[0] + x[0] * x[0] + z[0] * z[0] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[0] * y[0] +  x[0] * x[0] + z[0] * z[0] - L3 * L3)) )
//											- atan2(sqrt(y[0] * y[0] + z[0] * z[0] - L3 * L3) , x[0]));
//			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[0] * y[0] - x[0] * x[0] - z[0] * z[0]) / (2 * L1 * L2));
//	
//			a[1][0] = asin(L3 / sqrt(y[1] * y[1] + z[1] * z[1]) ) + atan2(z[1],-y[1]);
//			a[1][1] = -asin((y[1] * y[1] + x[1] * x[1] + z[1] * z[1] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[1] * y[1] +  x[1] * x[1] + z[1] * z[1] - L3 * L3)) )
//											+ atan2(sqrt(y[1] * y[1] + z[1] * z[1] - L3 * L3) , x[1]);
//			a[1][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[1] * y[1] - x[1] * x[1] - z[1] * z[1]) / (2 * L1 * L2));
//	
//			a[2][0] = -asin(L3 / sqrt(y[2] * y[2] + z[2] * z[2]) ) - atan2(z[2],y[2]);
//			a[2][1] = -(asin((y[2] * y[2] + x[2] * x[2] + z[2] * z[2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[2] * y[2] +  x[2] * x[2] + z[2] * z[2] - L3 * L3)) 
//											 - atan2(sqrt(y[2] * y[2] + z[2] * z[2] - L3 * L3) , -x[2])));
//			a[2][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[2] * y[2] - x[2] * x[2] - z[2] * z[2]) / (2 * L1 * L2));
//	
//			a[3][0] = asin(L3 / sqrt(y[3] * y[3] + z[3] * z[3]) ) + atan2(z[3],-y[3]);
//			a[3][1] = asin((y[3] * y[3] + x[3] * x[3] + z[3] * z[3] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[3] * y[3] +  x[3] * x[3] + z[3] * z[3] - L3 * L3)) )
//											- atan2(sqrt(y[3] * y[3] + z[3] * z[3] - L3 * L3) , -x[3]);
//			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[3] * y[3] - x[3] * x[3] - z[3] * z[3]) / (2 * L1 * L2));

	a[0][0] = -(pcpg_signal[9] + 1) / 2 * (-0.6);  // LF RF LH RH;  lift, swing1, , swing2
	a[0][1] = -(pcpg_signal[10] + 1) / 2 * 0.52;
	a[0][2] = -(pcpg_signal[10] + 1) / 2 * -0.64;
	a[1][0] = (pcpg_signal[1] + 1) / 2 * (-0.6);
	a[1][1] = (pcpg_signal[2] + 1) / 2 * 0.52;
	a[1][2] = (pcpg_signal[2] + 1) / 2 * -0.64;
	a[2][0] = -(pcpg_signal[13] + 1) / 2 * (-0.6);
	a[2][1] = -(pcpg_signal[14] + 1) / 2 * 0.52;
	a[2][2] = -(pcpg_signal[14] + 1) / 2 * -0.64;
	a[3][0] = (pcpg_signal[5] + 1) / 2 * (-0.6);
	a[3][1] = (pcpg_signal[6] + 1) / 2 * 0.52;
	a[3][2] = (pcpg_signal[6] + 1) / 2 * -0.64;
				
	theta[0][0]=(a[0][0]+a[0][1]);
	theta[0][1]=(a[0][1]-a[0][0]);
	
	theta[0][0]=-(int) (theta[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
	theta[0][1]=-(int) (theta[0][1]/3.1415926*180*100);
	a[0][2]=-(int) (a[0][2]/3.1415926*180*100);

	theta[0][0]= (float) (theta[0][0]/100);
	theta[0][1]= (float) (theta[0][1]/100);
	a[0][2]= (float) (a[0][2]/100);
	
	theta[1][0]=(a[1][0]+a[1][1]);
	theta[1][1]=(a[1][1]-a[1][0]);
	
	theta[1][0]=-(int) (theta[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
	theta[1][1]=-(int) (theta[1][1]/3.1415926*180*100);
	a[1][2]=-(int) (a[1][2]/3.1415926*180*100);

	theta[1][0]= (float) (theta[1][0]/100);
	theta[1][1]= (float) (theta[1][1]/100);
	a[1][2]= (float) (a[1][2]/100);

	theta[2][0]=(a[2][1]-a[2][0]);
	theta[2][1]=(a[2][1]+a[2][0]);
	
	theta[2][0]=-(int) (theta[2][0]/3.1415926*180*100);//rf alpha 角，最内侧
	theta[2][1]=-(int) (theta[2][1]/3.1415926*180*100);
	a[2][2]=-(int) (a[2][2]/3.1415926*180*100);

	theta[2][0]= (float) (theta[2][0]/100);
	theta[2][1]= (float) (theta[2][1]/100);
	a[2][2]= (float) (a[2][2]/100);
	
	theta[3][0]=(a[3][1]-a[3][0]);
	theta[3][1]=(a[3][1]+a[3][0]);
	
	theta[3][0]=-(int) (theta[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
	theta[3][1]=-(int) (theta[3][1]/3.1415926*180*100);
	a[3][2]=-(int) (a[3][2]/3.1415926*180*100);

	theta[3][0]= (float) (theta[3][0]/100);
	theta[3][1]= (float) (theta[3][1]/100);
	a[3][2]= (float) (a[3][2]/100);
	
	if((pcpg_signal[3]-pcpg_signal[0])>0) // RF
	gas_control[1] = 2;    // LF RF LH RH
	else if((pcpg_signal[3]-pcpg_signal[0])==0)
	gas_control[1] = 1;
	else
	gas_control[1] = 0;
	
	if((pcpg_signal[7]-pcpg_signal[4])>0) // RH
	gas_control[3] = 2;    // LF RF LH RH
	else if((pcpg_signal[7]-pcpg_signal[4])==0)
	gas_control[3] = 1;
	else
	gas_control[3] = 0;
	
	if((pcpg_signal[11]-pcpg_signal[8])>0) // LF
	gas_control[0] = 2;    // LF RF LH RH
	else if((pcpg_signal[11]-pcpg_signal[8])==0)
	gas_control[0] = 1;
	else
	gas_control[0] = 0;
	
	if((pcpg_signal[15]-pcpg_signal[12])>0) // LH
	gas_control[2] = 2;    // LF RF LH RH
	else if((pcpg_signal[15]-pcpg_signal[12])==0)
	gas_control[2] = 1;
	else
	gas_control[2] = 0;
	
	gasControl();
	
	
	if(runFlag)
	{
	Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
	Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
	Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);

	Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
	Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
	Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
		
	Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
	Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
	Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);

	Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
	Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
	Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		
		
	
	}
}

void StartAngleInit1(void)//darius's robot
{
	//关节1、2、3分别为小腿关节、肩关节
	/*backup*/
	KMGecko.StartAngle[RF_J3] = 0; //52
	KMGecko.StartAngle[RF_J2] = -9; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 2; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = 2;	//-10//-25;	
	KMGecko.StartAngle[RH_J2] = 2;	//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = 0 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = 3;	//-10
	KMGecko.StartAngle[LF_J2] = 3;	//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = 0;	//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = 2;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = 4;///0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = -3;//;small figure turning to the inside		
/*second*/
//	KMGecko.StartAngle[RF_J3] = 45; //52
//	KMGecko.StartAngle[RF_J2] = -45; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = 5; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = -45;	//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = 45;	//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = 0 ;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = -45;	//-10
//	KMGecko.StartAngle[LF_J2] = 45;	//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = 0;	//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = 45;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = -45;///0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = 0;//;small figure turning to the inside		
		/*third*/
/*second hold*/
//	KMGecko.StartAngle[RF_J3] = 45-102; //52
//	KMGecko.StartAngle[RF_J2] = -45+91; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = 10; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = -45+88;	//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = 45-103;	//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = -20 ;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = -45+92;	//-10
//	KMGecko.StartAngle[LF_J2] = 45-92;	//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = -25;	//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = 45-95 ;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = -45+90;//0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = 15;//;small figure turning to the inside		
/*third hold*/
//	KMGecko.StartAngle[RF_J3] = 45-10-82; //52
//	KMGecko.StartAngle[RF_J2] = -45+10+82; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = 30; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = -45+16+85;	//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = 45-16-80;	//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = -40 ;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = -45+11+82;//-10
//	KMGecko.StartAngle[LF_J2] = 45-6-82;	//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = -30;	//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = 45-10-85;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = -45+10+82;//0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = 35;//;small figure turning to the inside	

	
	/*航天三院*/
//	KMGecko.StartAngle[RF_J3] = -1; //52
//	KMGecko.StartAngle[RF_J2] = -25; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = -60; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = 1;//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = 30;//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = 10;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = 1;//-10
//	KMGecko.StartAngle[LF_J2] = 34;//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = 55;//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = -1;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = -35;//0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = -5;//;small figure turning to the inside
}
void StartAngleInit2(void)//darius's robot
{
/**version1**/
	//关节1、2、3分别为踝关节、膝关节、髋关节
	//number 3
	KMGecko.StartAngle[RF_J3] = 52; //52
	KMGecko.StartAngle[RF_J2] = 26; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 7; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = -25;//-10//-25;	
	KMGecko.StartAngle[RH_J2] = 50;//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = -5;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = -10;//-10
	KMGecko.StartAngle[LF_J2] = -13;//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -2;//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = -30;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = 2;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 0;//;small figure turning to the inside
}
void StartAngleInit3(void)//darius's robot
{
	KMGecko.StartAngle[RF_J3] = 45-102; //52
	KMGecko.StartAngle[RF_J2] = -45+95; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 15; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = -45+88;	//-10//-25;	
	KMGecko.StartAngle[RH_J2] = 45-103;	//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = -30 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = -45+92;	//-10
	KMGecko.StartAngle[LF_J2] = 45-92;	//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -30;	//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = 45-95 ;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = -45+90;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 25;//;small figure turning to the inside		
}
void InitRobotPosion(void)
{
//	if(((Time8Channel3HighTime<=1600)&&(Time8Channel3HighTime>=1400))||(Time8Channel3HighTime<=500))
//	{	
	Angle(KMGecko.StartAngle[LF_J3],LF_J3);
	Angle(KMGecko.StartAngle[LF_J1],LF_J1);
	Angle(KMGecko.StartAngle[LF_J2],LF_J2);
	
	Angle(KMGecko.StartAngle[RF_J1],RF_J1);
	Angle(KMGecko.StartAngle[RF_J2],RF_J2);		
	Angle(KMGecko.StartAngle[RF_J3],RF_J3);
	
	Angle(KMGecko.StartAngle[RH_J1],RH_J1);
	Angle(KMGecko.StartAngle[RH_J2],RH_J2);
	Angle(KMGecko.StartAngle[RH_J3],RH_J3);

	Angle(KMGecko.StartAngle[LH_J1],LH_J1);
	Angle(KMGecko.StartAngle[LH_J2],LH_J2);
	Angle(KMGecko.StartAngle[LH_J3],LH_J3);

//	setcurrentposition(RF,L1,L2,-L3);
//	setcurrentposition(RH,L1,L2,-L3);
//	setcurrentposition(LF,L1,L2,-L3);
//	setcurrentposition(LH,L1,L2,-L3);
//	HAL_Delay(30);	
}
void Angle(float angle,int8_t footnumber)
{
	int16_t ArrValue = 0;
	//ArrValue = 250 + (int32_t) (angle *0.18);
	//ArrValue = angle * 5 + 750;
	ArrValue = (int32_t) (angle * 5.56 + 750)	;
	if(footnumber == LF_J1)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare1(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LF_J2)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare2(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LF_J3)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare3(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LH_J1)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare1(&TIM12_Handler,ArrValue);
	}	
	else if(footnumber == LH_J2)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare2(&TIM12_Handler,ArrValue);
	}	
	else if(footnumber == LH_J3)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare1(&TIM15_Handler,ArrValue);
	}	
	else if(footnumber == RF_J1)
	{
	//	User_PWM_SetValue(&htim3, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare1(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RF_J2)
	{
	//	User_PWM_SetValue(&htim3, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare2(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RF_J3)
	{
	//	User_PWM_SetValue(&htim1, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare3(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RH_J1)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare1(&TIM3_Handler,ArrValue);
	}
	else if(footnumber == RH_J2)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare2(&TIM3_Handler,ArrValue);
	}
	else if(footnumber == RH_J3)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare3(&TIM3_Handler,ArrValue);
	}
}