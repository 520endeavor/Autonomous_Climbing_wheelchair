#include "DataStorage.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "chaoshengbo.h"
#include "24cxx.h"
#include "arm_math.h"

ManualCMD CMD_SteeringEngine1Add;
ManualCMD CMD_SteeringEngine1Sub;
ManualCMD CMD_SteeringEngine2Add;
ManualCMD CMD_SteeringEngine2Sub;
ManualCMD CMD_SteeringEngine3Add;
ManualCMD CMD_SteeringEngine3Sub;
ManualCMD CMD_SteeringEngine4Add;
ManualCMD CMD_SteeringEngine4Sub;
ManualCMD CMD_TrackMotRightForward;
ManualCMD CMD_TrackMotRightBack;
ManualCMD CMD_TrackMotRightStop;
ManualCMD CMD_TrackMotLeftForward;
ManualCMD CMD_TrackMotLeftBack;
ManualCMD CMD_TrackMotLeftStop;
ManualCMD CMD_StepMotUpContinued;
ManualCMD CMD_StepMotDownContinued;
ManualCMD CMD_StepMotStopContinued;
ManualCMD CMD_StepMotUpOneStep;
ManualCMD CMD_StepMotDownOneStep;
ManualCMD CMD_PushRodFrontUp;
ManualCMD CMD_PushRodFrontDown;
ManualCMD CMD_PushRodFrontStop;
ManualCMD CMD_PushRodBehindUp;
ManualCMD CMD_PushRodBehindDown;
ManualCMD CMD_PushRodBehindStop;


double Chair_add(double a,double b){
	double aa,bb,cc;
	aa=a;
	bb=b;
	cc=aa+bb;
	return cc;
}

double Chair_sub(double a,double b){
	double aa,bb,cc;
	aa=a;
	bb=b;
	cc=aa-bb;
	return cc;

}
double Chair_mul(double a,double b){
	double aa,bb,cc;
	aa=a;
	bb=b;
	cc=aa*bb;
	return cc;
}
double Chair_div(double a,double b){
	double aa;
	aa=a/b;
	return aa;
}
double Chair_cos(double a){	
	double aa;
	aa=cos(a);
	return aa;
}
double Chair_sin(double a){
	double aa;
	aa=sin(a);
	return aa;
}
double Chair_sqrt(double a){
	double aa,dd;
	aa=a;
  dd=sqrt(aa);
	return dd;
	
}
//float Chair_add(float a,float b){
//	float aa,bb,cc;
//	aa=a;
//	bb=b;
//	arm_add_f32(&a,&b,&cc,1);
//	return cc;
//}

//float Chair_sub(float a,float b){
//	float aa,bb,cc;
//	aa=a;
//	bb=b;
//	arm_sub_f32(&a,&b,&cc,1);
//	return cc;

//}
//float Chair_mul(float a,float b){
//	float aa,bb,cc;
//	aa=a;
//	bb=b;
//	arm_mult_f32(&a,&b,&cc,1);
//	return cc;
//}
//float Chair_div(float a,float b){
//	float aa;

//	aa=a/b;
//	return aa;
//}
//float Chair_cos(float a){	
//	float aa;
//	aa=arm_cos_f32(a);
//	return aa;
//}
//float Chair_sin(float a){
//	float aa;
//	aa=arm_sin_f32(a);
//	return aa;
//}
//float Chair_sqrt(float a){
//	float aa,dd;
//	aa=a;
//  arm_sqrt_f32(aa,&dd);
//	return dd;
//	
//}
void FloatData_AddInit(void){
	
JY61_1.ChairBalanceAngleMax.Add= 	ChairBalanceAngleMax_Add  ;   							
JY61_1.ChairBalanceAngleMin.Add=  	ChairBalanceAngleMin_Add  ;       					
JY61_2.StepMotVibrationAngle1.Add= 	StepMotVibrationAngle1_Add	;	   					
JY61_2.StepMotVibrationAngle2.Add =	StepMotVibrationAngle2_Add		;						
ChaoShengBodata.SetData[0].Add= 	ChaoShengBoData_SetData_0_Add	;						
ChaoShengBodata.SetData[1].Add=  	ChaoShengBoData_SetData_1_Add		;					
ChaoShengBodata.SetData[2].Add=  	ChaoShengBoData_SetData_2_Add		;					
ChaoShengBodata.SetData[3].Add=  	ChaoShengBoData_SetData_3_Add		;					
ChaoShengBodata.SetData1[0].Add=  	ChaoShengBoData_SetData1_0_Add	;					
ChaoShengBodata.SetData1[1].Add= 	ChaoShengBoData_SetData1_1_Add;			
ChaoShengBodata.SetData1[2].Add=	ChaoShengBoData_SetData1_2_Add;		 
ChaoShengBodata.SetData1[3].Add= 	ChaoShengBoData_SetData1_3_Add;	
ChaoShengBodata.SetData2[0].Add= 	ChaoShengBoData_SetData2_0_Add; 
ChaoShengBodata.SetData2[1].Add=  	ChaoShengBoData_SetData2_1_Add;
ChaoShengBodata.SetData2[2].Add=  	ChaoShengBoData_SetData2_2_Add;
ChaoShengBodata.SetData2[3].Add=  	ChaoShengBoData_SetData2_3_Add;
TrackMotRightData.SetSpeed.Add= 	TrackMotRightData_SetSpeed_Add 						; 
TrackMotRightData.SetSpeed1.Add=  	TrackMotRightData_SetSpeed1_Add						 ;
TrackMotLeftData.SetSpeed.Add=  	TrackMotLeftData_SetSpeed_Add 						 ;
TrackMotLeftData.SetSpeed1.Add=  	TrackMotLeftData_SetSpeed1_Add						 ;
StepMotdata.CurrentAngle.Add=   StepMotData_CurrentAngle_Add							 ;
StepMotdata.Speed.Add=	StepMotData_Speed_Add											 ;
SteeringEngine1data.SetAngle.Add= 	SteeringEngine1dataa_SetAngle_Add					 ;
SteeringEngine2data.SetAngle.Add=  	SteeringEngine2dataa_SetAngle_Add					 ;
SteeringEngine3data.SetAngle.Add=  	SteeringEngine3dataa_SetAngle_Add					 ;
SteeringEngine4data.SetAngle.Add=  	SteeringEngine4dataa_SetAngle_Add					 ;
SteeringEngine1data.SetAngle1.Add=  	SteeringEngine1dataa_SetAngle1_Add				 ;
SteeringEngine2data.SetAngle1.Add= 	SteeringEngine2dataa_SetAngle1_Add				 ;
SteeringEngine3data.SetAngle1.Add= 	SteeringEngine3dataa_SetAngle1_Add				 ;
SteeringEngine4data.SetAngle1.Add= 	SteeringEngine4dataa_SetAngle1_Add				 ;
SteeringEngine1data.SetAngle2.Add= 	SteeringEngine1dataa_SetAngle2_Add				 ;
SteeringEngine2data.SetAngle2.Add= 	SteeringEngine2dataa_SetAngle2_Add				 ;
SteeringEngine3data.SetAngle2.Add= 	SteeringEngine3dataa_SetAngle2_Add				 ;
SteeringEngine4data.SetAngle2.Add= 	SteeringEngine4dataa_SetAngle2_Add				 ;

JY61_1.Roll_Pitch_Yaw_Data.Roll_X.Add= 	JY61_1_Roll_X_Add													 ;
JY61_1.Roll_Pitch_Yaw_Data.Pitch_Y.Add= 	JY61_1_Pitch_Y_Add												 ;
JY61_1.Roll_Pitch_Yaw_Data.Yaw_Z.Add= 	JY61_1_Yaw_Z_Add													 ;
JY61_1.Roll_Pitch_Yaw_Data.T.Add= 	JY61_1_T_Add															 ;
JY61_2.Roll_Pitch_Yaw_Data.Roll_X.Add= 	 	JY61_2_Roll_X_Add													 ;
JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Add=  	JY61_2_Pitch_Y_Add												 ;
JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Add= 	JY61_2_Yaw_Z_Add													 ;
JY61_2.Roll_Pitch_Yaw_Data.T.Add=  	JY61_2_T_Add															 ;
ChaoShengBodata.floatData[0].Add= 	ChaoShengBoData_floatData_0_Add						 ;
ChaoShengBodata.floatData[1].Add= 	ChaoShengBoData_floatData_1_Add						 ;
ChaoShengBodata.floatData[2].Add= 	ChaoShengBoData_floatData_2_Add						 ;
ChaoShengBodata.floatData[3].Add= 	ChaoShengBoData_floatData_3_Add						 ;
TrackMotRightData.CurrentSpeed.Add= 	TrackMotRightData_CurrentSpeed_Add				;
TrackMotLeftData.CurrentSpeed.Add= 	TrackMotLeftData_CurrentSpeed_Add 				;
SteeringEngine1data.CurrentAngle.Add= 	SteeringEngine1data_CurrentAngle_Add			;
SteeringEngine2data.CurrentAngle.Add=  	SteeringEngine2data_CurrentAngle_Add			;
SteeringEngine3data.CurrentAngle.Add=  	SteeringEngine3data_CurrentAngle_Add			;
SteeringEngine4data.CurrentAngle.Add=  	SteeringEngine4data_CurrentAngle_Add			;
SteeringEngine1data.OffsetAngle.Add=  	SteeringEngine1data_OffsetAngle_Add			;
StepMotdata.OffsetAngle.Add= 	StepMotOffsetAngle_Add										;

ChaoShengBodata.floatData[4].Add= 	ChaoShengBoData_floatData_4_Add						 ;
ChaoShengBodata.floatData[5].Add= 	ChaoShengBoData_floatData_5_Add						 ;



CMD_SteeringEngine1Add.Add= CMD_SteeringEngine1Add_Add  ;
CMD_SteeringEngine1Sub.Add=  CMD_SteeringEngine1Sub_Add ;

CMD_SteeringEngine2Add.Add= CMD_SteeringEngine2Add_Add ;
CMD_SteeringEngine2Sub.Add= CMD_SteeringEngine2Sub_Add;
CMD_SteeringEngine3Add.Add= CMD_SteeringEngine3Add_Add ;
CMD_SteeringEngine3Sub.Add= CMD_SteeringEngine3Sub_Add;
CMD_SteeringEngine4Add.Add= CMD_SteeringEngine4Add_Add;
CMD_SteeringEngine4Sub.Add= CMD_SteeringEngine4Sub_Add ;
CMD_TrackMotRightForward.Add= CMD_TrackMotRightForward_Add ;
CMD_TrackMotRightBack.Add= CMD_TrackMotRightBack_Add ;
CMD_TrackMotRightStop.Add= CMD_TrackMotRightStop_Add ;
CMD_TrackMotLeftForward.Add= CMD_TrackMotLeftForward_Add ;
CMD_TrackMotLeftBack.Add= CMD_TrackMotLeftBack_Add ;
CMD_TrackMotLeftStop.Add= CMD_TrackMotLeftStop_Add ;
CMD_StepMotUpContinued.Add= CMD_StepMotUpContinued_Add ;
CMD_StepMotDownContinued.Add= CMD_StepMotDownContinued_Add; 
CMD_StepMotStopContinued.Add= CMD_StepMotStopContinued_Add ;
CMD_StepMotUpOneStep.Add= CMD_StepMotUpOneStep_Add ;
CMD_StepMotDownOneStep.Add= CMD_StepMotDownOneStep_Add; 
CMD_PushRodFrontUp.Add= CMD_PushRodFrontUp_Add;
CMD_PushRodFrontDown.Add= CMD_PushRodFrontDown_Add ;
CMD_PushRodFrontStop.Add= CMD_PushRodFrontStop_Add ;
CMD_PushRodBehindUp.Add= CMD_PushRodBehindUp_Add ;
CMD_PushRodBehindDown.Add= CMD_PushRodBehindDown_Add ;
CMD_PushRodBehindStop.Add= CMD_PushRodBehindStop_Add;
}
void FloatData_GetFormEEPROM_TWO(uint16_t add,uint8_t* data){
	if(add==SteeringEngine1data.OffsetAngle.Add){
		SteeringEngine1data.OffsetAngle.Data.chardata[0]=data[0];
		SteeringEngine1data.OffsetAngle.Data.chardata[1]=data[1];
		SteeringEngine1data.OffsetAngle.Data.chardata[2]=data[2];
		SteeringEngine1data.OffsetAngle.Data.chardata[3]=data[3];
	}
	if(add==StepMotdata.OffsetAngle.Add){
		StepMotdata.OffsetAngle.Data.chardata[0]=data[0];
		StepMotdata.OffsetAngle.Data.chardata[1]=data[1];
		StepMotdata.OffsetAngle.Data.chardata[2]=data[2];
		StepMotdata.OffsetAngle.Data.chardata[3]=data[3];
	}
}
void FloatData_GetFormEEPROM_ONE(uint16_t add){
			
if(add==JY61_1.ChairBalanceAngleMax.Add){
		 AT24CXX_Read(JY61_1.ChairBalanceAngleMax.Add,JY61_1.ChairBalanceAngleMax.Data.chardata,4);
}else if(add==JY61_1.ChairBalanceAngleMin.Add){
		AT24CXX_Read(JY61_1.ChairBalanceAngleMin.Add,JY61_1.ChairBalanceAngleMin.Data.chardata,4);
}else if(add==JY61_2.StepMotVibrationAngle1.Add){
		AT24CXX_Read(JY61_2.StepMotVibrationAngle1.Add,JY61_2.StepMotVibrationAngle1.Data.chardata,4);
}else if(add==JY61_2.StepMotVibrationAngle2.Add){	
	AT24CXX_Read(JY61_2.StepMotVibrationAngle2.Add,JY61_2.StepMotVibrationAngle2.Data.chardata,4);	
}else if(add==ChaoShengBodata.SetData[0].Add){
		AT24CXX_Read(ChaoShengBodata.SetData[0].Add,ChaoShengBodata.SetData[0].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData[1].Add){
	AT24CXX_Read(ChaoShengBodata.SetData[1].Add,ChaoShengBodata.SetData[1].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData[2].Add){	
	AT24CXX_Read(ChaoShengBodata.SetData[2].Add,ChaoShengBodata.SetData[2].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData[3].Add){
	AT24CXX_Read(ChaoShengBodata.SetData[3].Add,ChaoShengBodata.SetData[3].Data.chardata,4);	
}else if(add==ChaoShengBodata.SetData1[0].Add){
	AT24CXX_Read(ChaoShengBodata.SetData1[0].Add,ChaoShengBodata.SetData1[0].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData1[1].Add){
	AT24CXX_Read(ChaoShengBodata.SetData1[1].Add,ChaoShengBodata.SetData1[1].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData1[2].Add){	
	AT24CXX_Read(ChaoShengBodata.SetData1[2].Add,ChaoShengBodata.SetData1[2].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData1[3].Add){
	AT24CXX_Read(ChaoShengBodata.SetData1[3].Add,ChaoShengBodata.SetData1[3].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData2[0].Add){	
	AT24CXX_Read(ChaoShengBodata.SetData2[0].Add,ChaoShengBodata.SetData2[0].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData2[1].Add){
	AT24CXX_Read(ChaoShengBodata.SetData2[1].Add,ChaoShengBodata.SetData2[1].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData2[2].Add){	
	AT24CXX_Read(ChaoShengBodata.SetData2[2].Add,ChaoShengBodata.SetData2[2].Data.chardata,4);
}else if(add==ChaoShengBodata.SetData2[3].Add){
	AT24CXX_Read(ChaoShengBodata.SetData2[3].Add,ChaoShengBodata.SetData2[3].Data.chardata,4);	
}else if(add==TrackMotRightData.SetSpeed.Add){
	
	AT24CXX_Read(TrackMotRightData.SetSpeed.Add,TrackMotRightData.SetSpeed.Data.chardata,4);
}else if(add==TrackMotRightData.SetSpeed1.Add){
	AT24CXX_Read(TrackMotRightData.SetSpeed1.Add,TrackMotRightData.SetSpeed1.Data.chardata,4);
}else if(add==TrackMotLeftData.SetSpeed.Add){

	AT24CXX_Read(TrackMotLeftData.SetSpeed.Add,TrackMotLeftData.SetSpeed.Data.chardata,4);
}else if(add==TrackMotLeftData.SetSpeed1.Add){
	AT24CXX_Read(TrackMotLeftData.SetSpeed1.Add,TrackMotLeftData.SetSpeed1.Data.chardata,4);
}else if(add==StepMotdata.CurrentAngle.Add){	
	AT24CXX_Read(StepMotdata.CurrentAngle.Add,StepMotdata.CurrentAngle.Data.chardata,4);
}else if(add==StepMotdata.Speed.Add){
	AT24CXX_Read(StepMotdata.Speed.Add,StepMotdata.Speed.Data.chardata,4);
}else if(add==SteeringEngine1data.SetAngle.Add){	
	AT24CXX_Read(SteeringEngine1data.SetAngle.Add,SteeringEngine1data.SetAngle.Data.chardata,4);
}else if(add==SteeringEngine2data.SetAngle.Add){
	AT24CXX_Read(SteeringEngine2data.SetAngle.Add,SteeringEngine2data.SetAngle.Data.chardata,4);
}else if(add==SteeringEngine3data.SetAngle.Add){	
	AT24CXX_Read(SteeringEngine3data.SetAngle.Add,SteeringEngine3data.SetAngle.Data.chardata,4);
}else if(add==SteeringEngine4data.SetAngle.Add){	
	AT24CXX_Read(SteeringEngine4data.SetAngle.Add,SteeringEngine4data.SetAngle.Data.chardata,4);
}else if(add==SteeringEngine1data.SetAngle1.Add){
	AT24CXX_Read(SteeringEngine1data.SetAngle1.Add,SteeringEngine1data.SetAngle1.Data.chardata,4);
}else if(add==SteeringEngine2data.SetAngle1.Add){
	AT24CXX_Read(SteeringEngine2data.SetAngle1.Add,SteeringEngine2data.SetAngle1.Data.chardata,4);
}else if(add==SteeringEngine3data.SetAngle1.Add){	
	AT24CXX_Read(SteeringEngine3data.SetAngle1.Add,SteeringEngine3data.SetAngle1.Data.chardata,4);
}else if(add==SteeringEngine4data.SetAngle1.Add){	
	AT24CXX_Read(SteeringEngine4data.SetAngle1.Add,SteeringEngine4data.SetAngle1.Data.chardata,4);
}else if(add==SteeringEngine1data.SetAngle2.Add){
	AT24CXX_Read(SteeringEngine1data.SetAngle2.Add,SteeringEngine1data.SetAngle2.Data.chardata,4);
}else if(add==SteeringEngine2data.SetAngle2.Add){
	AT24CXX_Read(SteeringEngine2data.SetAngle2.Add,SteeringEngine2data.SetAngle2.Data.chardata,4);
}else if(add==SteeringEngine3data.SetAngle2.Add){	
	AT24CXX_Read(SteeringEngine3data.SetAngle2.Add,SteeringEngine3data.SetAngle2.Data.chardata,4);
}else if(add==SteeringEngine4data.SetAngle2.Add){	
	AT24CXX_Read(SteeringEngine4data.SetAngle2.Add,SteeringEngine4data.SetAngle2.Data.chardata,4);
}
}

void FloatData_GetFormEEPROM(){
	/*超声波地址*/
	ChaoShengBodata.Add[0]=0xd0;	
	ChaoShengBodata.Add[1]=0xd2;	
	ChaoShengBodata.Add[2]=0xe0;	
	ChaoShengBodata.Add[3]=0xe2;	
	ChaoShengBodata.Add[4]=0xd8;	
	ChaoShengBodata.Add[5]=0xdc;	
	/*地址大于4000 未保存数据*/
	TrackMotRightData.CurrentSpeed.Data.floatdata=0;
	TrackMotLeftData.CurrentSpeed.Data.floatdata=0;
	SteeringEngine1data.OffsetAngle.Data.floatdata=0;
	SteeringEngine1data.CurrentAngle.Data.floatdata=0;
	SteeringEngine2data.CurrentAngle.Data.floatdata=0;
	SteeringEngine3data.CurrentAngle.Data.floatdata=0;
	SteeringEngine4data.CurrentAngle.Data.floatdata=0;
	StepMotdata.OffsetAngle.Data.floatdata=0;
	
	/*地址小于4000 保存数据*/
	AT24CXX_Read(JY61_1.ChairBalanceAngleMax.Add,JY61_1.ChairBalanceAngleMax.Data.chardata,4);
	AT24CXX_Read(JY61_1.ChairBalanceAngleMin.Add,JY61_1.ChairBalanceAngleMin.Data.chardata,4);
	AT24CXX_Read(JY61_2.StepMotVibrationAngle1.Add,JY61_2.StepMotVibrationAngle1.Data.chardata,4);
	AT24CXX_Read(JY61_2.StepMotVibrationAngle2.Add,JY61_2.StepMotVibrationAngle2.Data.chardata,4);	

	AT24CXX_Read(ChaoShengBodata.SetData[0].Add,ChaoShengBodata.SetData[0].Data.chardata,4);
	AT24CXX_Read(ChaoShengBodata.SetData[1].Add,ChaoShengBodata.SetData[1].Data.chardata,4);	
	AT24CXX_Read(ChaoShengBodata.SetData[2].Add,ChaoShengBodata.SetData[2].Data.chardata,4);
	AT24CXX_Read(ChaoShengBodata.SetData[3].Add,ChaoShengBodata.SetData[3].Data.chardata,4);	
	AT24CXX_Read(ChaoShengBodata.SetData1[0].Add,ChaoShengBodata.SetData1[0].Data.chardata,4);
	AT24CXX_Read(ChaoShengBodata.SetData1[1].Add,ChaoShengBodata.SetData1[1].Data.chardata,4);	
	AT24CXX_Read(ChaoShengBodata.SetData1[2].Add,ChaoShengBodata.SetData1[2].Data.chardata,4);
	AT24CXX_Read(ChaoShengBodata.SetData1[3].Add,ChaoShengBodata.SetData1[3].Data.chardata,4);	
	AT24CXX_Read(ChaoShengBodata.SetData2[0].Add,ChaoShengBodata.SetData2[0].Data.chardata,4);
	AT24CXX_Read(ChaoShengBodata.SetData2[1].Add,ChaoShengBodata.SetData2[1].Data.chardata,4);	
	AT24CXX_Read(ChaoShengBodata.SetData2[2].Add,ChaoShengBodata.SetData2[2].Data.chardata,4);
	AT24CXX_Read(ChaoShengBodata.SetData2[3].Add,ChaoShengBodata.SetData2[4].Data.chardata,4);	
	
	AT24CXX_Read(TrackMotRightData.SetSpeed.Add,TrackMotRightData.SetSpeed.Data.chardata,4);
	AT24CXX_Read(TrackMotRightData.SetSpeed1.Add,TrackMotRightData.SetSpeed1.Data.chardata,4);

	AT24CXX_Read(TrackMotLeftData.SetSpeed.Add,TrackMotLeftData.SetSpeed.Data.chardata,4);
	AT24CXX_Read(TrackMotLeftData.SetSpeed1.Add,TrackMotLeftData.SetSpeed1.Data.chardata,4);
	
	AT24CXX_Read(StepMotdata.CurrentAngle.Add,StepMotdata.CurrentAngle.Data.chardata,4);
	AT24CXX_Read(StepMotdata.Speed.Add,StepMotdata.Speed.Data.chardata,4);
	
	AT24CXX_Read(SteeringEngine1data.SetAngle.Add,SteeringEngine1data.SetAngle.Data.chardata,4);
	AT24CXX_Read(SteeringEngine2data.SetAngle.Add,SteeringEngine2data.SetAngle.Data.chardata,4);	
	AT24CXX_Read(SteeringEngine3data.SetAngle.Add,SteeringEngine3data.SetAngle.Data.chardata,4);	
	AT24CXX_Read(SteeringEngine4data.SetAngle.Add,SteeringEngine4data.SetAngle.Data.chardata,4);

	AT24CXX_Read(SteeringEngine1data.SetAngle1.Add,SteeringEngine1data.SetAngle1.Data.chardata,4);
	AT24CXX_Read(SteeringEngine2data.SetAngle1.Add,SteeringEngine2data.SetAngle1.Data.chardata,4);	
	AT24CXX_Read(SteeringEngine3data.SetAngle1.Add,SteeringEngine3data.SetAngle1.Data.chardata,4);	
	AT24CXX_Read(SteeringEngine4data.SetAngle1.Add,SteeringEngine4data.SetAngle1.Data.chardata,4);

	AT24CXX_Read(SteeringEngine1data.SetAngle2.Add,SteeringEngine1data.SetAngle2.Data.chardata,4);
	AT24CXX_Read(SteeringEngine2data.SetAngle2.Add,SteeringEngine2data.SetAngle2.Data.chardata,4);	
	AT24CXX_Read(SteeringEngine3data.SetAngle2.Add,SteeringEngine3data.SetAngle2.Data.chardata,4);	
	AT24CXX_Read(SteeringEngine4data.SetAngle2.Add,SteeringEngine4data.SetAngle2.Data.chardata,4);
	


 }
float Data_Get_Min(float a,float b){
	float c;
	if(a>=b){
		c=b;
	}else{
		c=a;
	}
	return c;
}
float Data_Get_Max(float a,float b){
	float c;
	if(a>=b){
		c=a;
	}else{
		c=b;
	}
	return c;
}