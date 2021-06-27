#ifndef __DataStorage_H
#define __DataStorage_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Datatypedef.h"
	 
#define 	ChairBalanceAngleMax_Add     							0
#define 	ChairBalanceAngleMin_Add         					4
#define 	StepMotVibrationAngle1_Add		   					8
#define 	StepMotVibrationAngle2_Add								12
#define 	ChaoShengBoData_SetData_0_Add							16
#define 	ChaoShengBoData_SetData_1_Add							20
#define 	ChaoShengBoData_SetData_2_Add							24
#define 	ChaoShengBoData_SetData_3_Add							28
#define 	ChaoShengBoData_SetData1_0_Add						32
#define 	ChaoShengBoData_SetData1_1_Add						36
#define 	ChaoShengBoData_SetData1_2_Add						40
#define 	ChaoShengBoData_SetData1_3_Add						44
#define 	ChaoShengBoData_SetData2_0_Add						48
#define 	ChaoShengBoData_SetData2_1_Add						52
#define 	ChaoShengBoData_SetData2_2_Add						56
#define 	ChaoShengBoData_SetData2_3_Add						60
#define 	TrackMotRightData_SetSpeed_Add 						64
#define 	TrackMotRightData_SetSpeed1_Add						68
#define 	TrackMotLeftData_SetSpeed_Add 						72
#define 	TrackMotLeftData_SetSpeed1_Add						76
#define 	StepMotData_CurrentAngle_Add							80
#define 	StepMotData_Speed_Add											84
#define 	SteeringEngine1dataa_SetAngle_Add					88
#define 	SteeringEngine2dataa_SetAngle_Add					92
#define 	SteeringEngine3dataa_SetAngle_Add					96
#define 	SteeringEngine4dataa_SetAngle_Add					100
#define 	SteeringEngine1dataa_SetAngle1_Add				104
#define 	SteeringEngine2dataa_SetAngle1_Add				108
#define 	SteeringEngine3dataa_SetAngle1_Add				112
#define 	SteeringEngine4dataa_SetAngle1_Add				116
#define 	SteeringEngine1dataa_SetAngle2_Add				120
#define 	SteeringEngine2dataa_SetAngle2_Add				124
#define 	SteeringEngine3dataa_SetAngle2_Add				128
#define 	SteeringEngine4dataa_SetAngle2_Add				132

#define 	JY61_1_Roll_X_Add													4000
#define 	JY61_1_Pitch_Y_Add												4004
#define 	JY61_1_Yaw_Z_Add													4008
#define 	JY61_1_T_Add															4012
#define 	JY61_2_Roll_X_Add													4016
#define 	JY61_2_Pitch_Y_Add												4020
#define 	JY61_2_Yaw_Z_Add													4024
#define 	JY61_2_T_Add															4028
#define 	ChaoShengBoData_floatData_0_Add						4032
#define 	ChaoShengBoData_floatData_1_Add						4036
#define 	ChaoShengBoData_floatData_2_Add						4040
#define 	ChaoShengBoData_floatData_3_Add						4044
#define 	TrackMotRightData_CurrentSpeed_Add				4048
#define 	TrackMotLeftData_CurrentSpeed_Add 				4052
#define 	SteeringEngine1data_CurrentAngle_Add			4056
#define 	SteeringEngine2data_CurrentAngle_Add			4060
#define 	SteeringEngine3data_CurrentAngle_Add			4064
#define 	SteeringEngine4data_CurrentAngle_Add			4068
#define 	SteeringEngine1data_OffsetAngle_Add				4072
#define 	StepMotOffsetAngle_Add										4076
#define 	ChaoShengBoData_floatData_4_Add						4080
#define 	ChaoShengBoData_floatData_5_Add						4084


#define 	CMD_SteeringEngine1Add_Add	     					0
#define 	CMD_SteeringEngine1Sub_Add	     					1
#define 	CMD_SteeringEngine2Add_Add	     					2
#define 	CMD_SteeringEngine2Sub_Add	     					3
#define 	CMD_SteeringEngine3Add_Add	     					4
#define 	CMD_SteeringEngine3Sub_Add	     					5
#define 	CMD_SteeringEngine4Add_Add	     					6
#define 	CMD_SteeringEngine4Sub_Add	     					7
#define 	CMD_TrackMotRightForward_Add	     				8
#define 	CMD_TrackMotRightBack_Add	     						9
#define 	CMD_TrackMotRightStop_Add	     						10
#define 	CMD_TrackMotLeftForward_Add	     					11
#define 	CMD_TrackMotLeftBack_Add	     						12
#define 	CMD_TrackMotLeftStop_Add	     						13
#define 	CMD_StepMotUpContinued_Add	     					14
#define 	CMD_StepMotDownContinued_Add	     				15
#define 	CMD_StepMotStopContinued_Add	     				16
#define 	CMD_StepMotUpOneStep_Add	    						17
#define 	CMD_StepMotDownOneStep_Add	     					18
#define 	CMD_PushRodFrontUp_Add	     						  19
#define 	CMD_PushRodFrontDown_Add	     						20
#define 	CMD_PushRodFrontStop_Add	     						21
#define 	CMD_PushRodBehindUp_Add	     						  22
#define 	CMD_PushRodBehindDown_Add	     						23
#define 	CMD_PushRodBehindStop_Add	     						24


#define CMD_OPEN 'O'
#define CMD_CLOSE 'C'
#define CMD_OTHER 'Z'



extern ManualCMD CMD_SteeringEngine1Add;
extern ManualCMD CMD_SteeringEngine1Sub;
extern ManualCMD CMD_SteeringEngine2Add;
extern ManualCMD CMD_SteeringEngine2Sub;
extern ManualCMD CMD_SteeringEngine3Add;
extern ManualCMD CMD_SteeringEngine3Sub;
extern ManualCMD CMD_SteeringEngine4Add;
extern ManualCMD CMD_SteeringEngine4Sub;
extern ManualCMD CMD_TrackMotRightForward;
extern ManualCMD CMD_TrackMotRightBack;
extern ManualCMD CMD_TrackMotRightStop;
extern ManualCMD CMD_TrackMotLeftForward;
extern ManualCMD CMD_TrackMotLeftBack;
extern ManualCMD CMD_TrackMotLeftStop;
extern ManualCMD CMD_StepMotUpContinued;
extern ManualCMD CMD_StepMotDownContinued;
extern ManualCMD CMD_StepMotStopContinued;
extern ManualCMD CMD_StepMotUpOneStep;
extern ManualCMD CMD_StepMotDownOneStep;
extern ManualCMD CMD_PushRodFrontUp;
extern ManualCMD CMD_PushRodFrontDown;
extern ManualCMD CMD_PushRodFrontStop;
extern ManualCMD CMD_PushRodBehindUp;
extern ManualCMD CMD_PushRodBehindDown;
extern ManualCMD CMD_PushRodBehindStop;	 
	 
	 
void FloatData_AddInit(void);
void FloatData_GetFormEEPROM_ONE(uint16_t add);
void FloatData_GetFormEEPROM();
void FloatData_GetFormEEPROM_TWO(uint16_t add,uint8_t* data);

double Chair_add(double a,double b);
double Chair_sub(double a,double b);
double Chair_mul(double a,double b);
double Chair_div(double a,double b);
double Chair_cos(double a);
double Chair_sin(double a);
double Chair_sqrt(double a);
//float Chair_add(float a,float b);
//float Chair_sub(float a,float b);
//float Chair_mul(float a,float b);
//float Chair_div(float a,float b);
//float Chair_cos(float a);
//float Chair_sin(float a);
//float Chair_sqrt(float a);

float Data_Get_Min(float a,float b); 
float Data_Get_Max(float a,float b);
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
#endif