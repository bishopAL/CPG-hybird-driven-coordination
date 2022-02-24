#ifndef __MOTION_H__
#define __MOTION_H__
#include <sys.h>
#include <timer.h>

#define JonintNum 12
#define RF_J1 0
#define LF_J1 1
#define RH_J1 2
#define LH_J1 3

#define RF_J2 4
#define LF_J2 5
#define RH_J2 6
#define LH_J2 7

#define RF_J3 8
#define LF_J3 9
#define RH_J3 10
#define LH_J3 11

#define RF 100
#define RH 200
#define LF 300
#define LH 400	
#define LFFLAG 1
#define RFFLAG 2
#define LHFLAG 3
#define RHFLAG 4	

extern float L_onestep;
extern float W_onestep;
extern float D_onestep;
extern float H_onestep;
extern const short L1;
extern const short L2;
extern const short L3;
extern float D; 
extern float B; 
extern float F1;
extern float F2;
extern float F3;
extern float stime;
extern float Time;
extern float cx[4];
extern float cy[4];
extern float cz[4];
extern float temp1; 
void StartAngleInit1(void);
void StartAngleInit2(void);
void StartAngleInit3(void);
void StartAngle_rectify(void);
void InitRobotPosion(void);                                
void Angle(float,int8_t);//output angle
void cpg_run(bool runFlag);

typedef struct Kinematics
{	
	float StartAngle[JonintNum];
	uint32_t StepNum;//Êý¾Ý²½¾à
}KinematicsArm;
	
extern KinematicsArm KMGecko;

#endif
