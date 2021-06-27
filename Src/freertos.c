/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "chaoshengbo.h"
#include "24cxx.h"
#include "Datatypedef.h"
#include "tim.h"
#include "usart.h"
#include "DataStorage.h"
#include "arm_math.h"
#include "gpio.h"
#include "adc.h"


/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */
	int ChaoShengBoOffSetData_Vertical[4]={0,0,0,0};
	int ChaoShengBoOffSetData_Horizontal[4]={0,0,0,0};
	uint8_t AutoModeTaskNumber;
///*ChaoShengBodata.SetData2[0].Data.floatdata:超声波1 2 水平距离*/
///*ChaoShengBodata.SetData2[1].Data.floatdata:超声波2 3 水平距离*/
///*ChaoShengBodata.SetData2[2].Data.floatdata:超声波2 3 对地最低的距离*/
/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTaskChaoShengBoHandle;
osThreadId myTaskUart1Handle;
osThreadId myTaskSendToVBHandle;
osThreadId myTaskLED_DS2Handle;
osThreadId myTaskReceiveFromVBHandle;
osThreadId myTaskCMDHandle;
osThreadId myTaskChairBalanceJudgeHandle;
osThreadId myTaskChairBalanceActHandle;
osThreadId myTaskJY61_2DataHandle;
osThreadId myTaskPerceptionHandle;
osThreadId myTaskModeSwitch_ManualHandle;
osThreadId myTaskModeSwitch_AutoHandle;

osThreadId myTaskDownStairs_ReadyHandle;
osThreadId myTaskDownStairs_Ready1Handle;
osThreadId myTaskDownStairs_RunningHandle;
osThreadId myTaskDownStairs_LastStairHandle;

osThreadId myTaskUpStairs_ReadyHandle;
osThreadId myTaskUpStairs_Ready1Handle;
osThreadId myTaskUpStairs_RunningHandle;
osThreadId myTaskUpStairs_LastStairHandle;
osThreadId myTaskUp_DownStairsDirConnectJudgeHandle;
osThreadId myTaskUp_DownStairsDirConnectActHandle;

osThreadId myTaskAutoModeLedTwinkleHandle;
osThreadId myTaskManualModeLedTwinkleHandle;

osThreadId myTaskChaoShengBo_CalibrateHandle;
osThreadId myTaskManualModeActHandle;

	

/* USER CODE BEGIN Variables */
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void myTaskChaoShengBoTask(void const * argument);
void myTaskUart1(void const * argument);
void myTaskSendToVB(void const * argument);
void myTaskReceiveFromVB(void const * argument);
void myTaskLED_Ds2(void const *argument);
void myTaskCMD(void const *argument);
void myTaskChairBalanceJudge(void const *argument);
void myTaskChairBalanceAct(void const *argument);
void myTaskJY61_2Data(void const *argument);
void myTaskPerception(void const *argument);
void myTaskModeSwitch_Manual(void const *argument);

void myTaskModeSwitch_Auto(void const *argument);


void myTaskDownStairs_Ready(void const *argument);
void myTaskDownStairs_Ready1(void const *argument);
void myTaskDownStairs_Running(void const *argument);
void myTaskDownStairs_LastStair(void const *argument);

void myTaskUpStairs_Ready(void const *argument);
void myTaskUpStairs_Ready1(void const *argument);
void myTaskUpStairs_Running(void const *argument);
void myTaskUpStairs_LastStair(void const *argument);
void myTaskUp_DownStairsDirConnectJudge(void const *argument);
void myTaskUp_DownStairsDirConnectAct(void const *argument);
void myTaskAutoModeLedTwinkle(void const *argument);
void myTaskManualModeLedTwinkle(void const *argument);
void myTaskChaoShengBo_Calibrate(void const *argument);
void myTaskManualModeAct(void const *argument);

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
	QueueHandle_t xQueue_UART1;
	QueueHandle_t xQueue_AutoModeLedWinkle;
	QueueHandle_t xQueue_ManualModeLedWinkle;

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

	xQueue_UART1 = xQueueCreate( 200, sizeof(UART1Message_Typedef) );
	xQueue_AutoModeLedWinkle = xQueueCreate( 200, sizeof(uint8_t) );
	xQueue_ManualModeLedWinkle = xQueueCreate( 200, sizeof(uint8_t) );

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	osThreadDef(myTaskChaoShengBoTask, myTaskChaoShengBoTask, osPriorityNormal, 6, 128);
  myTaskChaoShengBoHandle = osThreadCreate(osThread(myTaskChaoShengBoTask), NULL);	

	osThreadDef(myTaskLED_DS2, myTaskLED_Ds2, osPriorityIdle, 3, 128);
  myTaskLED_DS2Handle = osThreadCreate(osThread(myTaskLED_DS2), NULL);
	
	osThreadDef(myTaskReceiveFromVB, myTaskReceiveFromVB, osPriorityNormal, 4, 128);
  myTaskReceiveFromVBHandle = osThreadCreate(osThread(myTaskReceiveFromVB), NULL);

	osThreadDef(myTaskCMD, myTaskCMD, osPriorityNormal, 4, 128);
  myTaskCMDHandle = osThreadCreate(osThread(myTaskCMD), NULL);


//	osThreadDef(myTaskUp_DownStairsDirConnectAct, myTaskUp_DownStairsDirConnectAct, osPriorityNormal, 4, 128);
//	myTaskUp_DownStairsDirConnectActHandle = osThreadCreate(osThread(myTaskUp_DownStairsDirConnectAct), NULL);
//	
//	osThreadDef(myTaskUp_DownStairsDirConnectJudge, myTaskUp_DownStairsDirConnectJudge, osPriorityNormal, 4, 128);
//   myTaskUp_DownStairsDirConnectJudgeHandle = osThreadCreate(osThread(myTaskUp_DownStairsDirConnectJudge), NULL);


	osThreadDef(myTaskJY61_2Data, myTaskJY61_2Data, osPriorityNormal, 4, 128);
  myTaskJY61_2DataHandle = osThreadCreate(osThread(myTaskJY61_2Data), NULL);

	/*自动灯控制任务*/
	osThreadDef(myTaskAutoModeLedTwinkle, myTaskAutoModeLedTwinkle, osPriorityNormal, 4, 128);
  myTaskAutoModeLedTwinkleHandle = osThreadCreate(osThread(myTaskAutoModeLedTwinkle), NULL);
	/*手动灯控制任务*/
	osThreadDef(myTaskManualModeLedTwinkle, myTaskManualModeLedTwinkle, osPriorityNormal, 4, 128);
  myTaskManualModeLedTwinkleHandle = osThreadCreate(osThread(myTaskManualModeLedTwinkle), NULL);	
	/*超声波校准任务*/
	osThreadDef(myTaskChaoShengBo_Calibrate, myTaskChaoShengBo_Calibrate, osPriorityNormal, 4, 128);
  myTaskChaoShengBo_CalibrateHandle = osThreadCreate(osThread(myTaskChaoShengBo_Calibrate), NULL);	
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
	MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  for(;;)
  {
		FloatData_AddInit();
		FloatData_GetFormEEPROM();
		/*超声波传感器归一校准数值*/
		ChaoShengBoOffSetData_Vertical[0]=0;
		ChaoShengBoOffSetData_Vertical[1]=-18;
		ChaoShengBoOffSetData_Vertical[2]=0;
		ChaoShengBoOffSetData_Vertical[3]=-135;
		ChaoShengBoOffSetData_Horizontal[0]=-64;
		ChaoShengBoOffSetData_Horizontal[1]=-26;
		ChaoShengBoOffSetData_Horizontal[2]=0;
		ChaoShengBoOffSetData_Horizontal[3]=0;
		ChaoShengBodata.CalibrateState=CMD_OPEN;

		SteeringEngineSetAngle(&SteeringEngine1data,SteeringEngine1data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine4data,SteeringEngine4data.SetAngle1.Data.floatdata);

		/*发送调试软件信息任务*/
		osThreadDef(myTaskSendToVB, myTaskSendToVB, osPriorityNormal, 4, 128);
		myTaskSendToVBHandle = osThreadCreate(osThread(myTaskSendToVB), NULL);
		/*接收调试软件信息任务*/
		osThreadDef(myTaskUart1, myTaskUart1, osPriorityNormal, 5, 128);
		myTaskUart1Handle = osThreadCreate(osThread(myTaskUart1), NULL);

		/*模式切换任务 手动任务*/
		osThreadDef(myTaskModeSwitch_Manual, myTaskModeSwitch_Manual, osPriorityNormal, 4, 128);
	   myTaskModeSwitch_ManualHandle = osThreadCreate(osThread(myTaskModeSwitch_Manual), NULL);

			osThreadDef(myTaskChairBalanceJudge, myTaskChairBalanceJudge, osPriorityNormal, 1, 128);
		myTaskChairBalanceJudgeHandle = osThreadCreate(osThread(myTaskChairBalanceJudge), NULL);

		xTaskNotifyGive( myTaskSendToVBHandle );	
		
		vTaskDelete(NULL);
		
  }
}

/* myTaskModeSwitch function */
void myTaskModeSwitch_Auto(void const * argument)
{
	while(ChaoShengBodata.CalibrateState!=CMD_OPEN||JY61_2.ReceiveData_OK!=CMD_OPEN){
		osDelay( 20);
	};
		/*判断自动模式运行*/
		if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_RESET){
			osDelay( 10 );
			if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_RESET){
				TrackMotLeftStop();
				TrackMotRightStop();
				/*创建感知任务*/
				osThreadDef(myTaskPerception, myTaskPerception, osPriorityNormal, 10, 128);
				myTaskPerceptionHandle = osThreadCreate(osThread(myTaskPerception), NULL);
				/*手动 自动 灯*/
				__IO uint8_t TxData=1;
				if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
				TxData=0;
				if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
		}
	}
  for(;;)
  {
	if(ChaoShengBodata.CalibrateState==CMD_OPEN&&JY61_2.ReceiveData_OK==CMD_OPEN){
		/*判断手动模式运行*/
		 if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_SET){
				osDelay( 20 );
				if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_SET){
					switch (AutoModeTaskNumber){
						case 0 :
							break;
						case 1 :
							vTaskDelete(myTaskPerceptionHandle);
							break;
						case 2 :
							vTaskDelete(myTaskUpStairs_ReadyHandle);
							break;
						case 3 :
							vTaskDelete(myTaskUpStairs_Ready1Handle);
							break;
						case 4 :
							vTaskDelete(myTaskUpStairs_RunningHandle);
							break;
						case 5 :
							vTaskDelete(myTaskUpStairs_LastStairHandle);
							break;
						case 6 :
							vTaskDelete(myTaskDownStairs_ReadyHandle);
							break;
						case 7 :
							vTaskDelete(myTaskDownStairs_Ready1Handle);
							break;
						case 8 :
							vTaskDelete(myTaskDownStairs_RunningHandle);
							break;
						case 9 :
							vTaskDelete(myTaskDownStairs_LastStairHandle);
							break;
						default:
							break;
					}
					AutoModeTaskNumber=0;
					osDelay( 100);

					TrackMotLeftStop();
					TrackMotRightStop();
					STEPMOTOR_STOP();
					osDelay( 20);

					/*模式切换任务 手动任务*/
					osThreadDef(myTaskModeSwitch_Manual, myTaskModeSwitch_Manual, osPriorityNormal, 4, 128);
					myTaskModeSwitch_ManualHandle = osThreadCreate(osThread(myTaskModeSwitch_Manual), NULL);


					vTaskDelete(NULL);	
			  }
			}	
		}	
		osDelay( 20);
  }
}

/* myTaskModeSwitch function */
void myTaskModeSwitch_Manual(void const * argument)
{		
	while(ChaoShengBodata.CalibrateState!=CMD_OPEN||JY61_2.ReceiveData_OK!=CMD_OPEN){
		osDelay( 20);
	};
		/*判断手动模式运行*/
	 if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_SET){
		osDelay( 20 );
		if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_SET){
		/*创建手动任务*/
		osThreadDef(myTaskManualModeAct, myTaskManualModeAct, osPriorityNormal, 4, 128);
		myTaskManualModeActHandle = osThreadCreate(osThread(myTaskManualModeAct), NULL);
		/*手动 自动 灯*/
		__IO uint8_t TxData=0;
		if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
		TxData=1;
		if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
		}
	}
  for(;;)
  {
	/*判断数据准备好*/
	if(ChaoShengBodata.CalibrateState==CMD_OPEN&&JY61_2.ReceiveData_OK==CMD_OPEN){
		/*判断自动模式运行*/
		if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_RESET){
			osDelay( 10 );
			if(HAL_GPIO_ReadPin(GPIOE,ModeSwitchKey_Pin)==GPIO_PIN_RESET){

				/*删除手动任务*/
				vTaskDelete(myTaskManualModeActHandle);
				osDelay( 20);
				TrackMotLeftStop();
				TrackMotRightStop();
				STEPMOTOR_STOP();

				osDelay( 20);
					
				/*创建感知任务*/
				osThreadDef(myTaskPerception, myTaskPerception, osPriorityNormal, 10, 128);
				myTaskPerceptionHandle = osThreadCreate(osThread(myTaskPerception), NULL);				
				/*模式切换任务 自动任务*/
				osThreadDef(myTaskModeSwitch_Auto, myTaskModeSwitch_Auto, osPriorityNormal, 4, 128);
				myTaskModeSwitch_AutoHandle = osThreadCreate(osThread(myTaskModeSwitch_Auto), NULL);
				vTaskDelete(NULL);

			}	
		}
	}
		osDelay( 20);
  }
}
/*手动任务*/
void myTaskManualModeAct(void const * argument)
{
float a_Max,a_Min;
uint16_t x_max=2430,x_min=1630,y_max=2630,y_min=1430;
  for(;;)
  {
		/*前进*/
	volatile	int LeftData,RightData;

		/*判断需要转弯*/	
			if(ADC_Value[0]-x_max>0){
				/*后退*/

				/*计算转弯油门*/
				if(ADC_Value[1]-y_max>0){
					/*往右拐*/
					LeftData=(ADC_Value[0]-x_max)*999/(4095-x_max+4095-y_max);
					RightData=((ADC_Value[0]-x_max)*999/(4095-x_max+4095-y_max));
					RightData+=((ADC_Value[1]-y_max)*999/(4095-y_max+4095-x_max));
					LeftData+=((ADC_Value[1]-y_max)*999/(4095-y_max+4095-x_max));
					RightData=-RightData;
				}else if(ADC_Value[1]-y_min<0){
					/*往左拐*/
					LeftData=(ADC_Value[0]-x_max)*999/(4095-x_max+y_min);
					RightData=((ADC_Value[0]-x_max)*999/(4095-x_max+y_min));
					RightData+=((y_min-ADC_Value[1])*999/(4095-x_max+y_min));
					LeftData+=((y_min-ADC_Value[1])*999/(4095-x_max+y_min));
					LeftData=-LeftData;
				}else{

					/*直行*/
					LeftData=-(ADC_Value[0]-x_max)*999/(4095-x_max);
					RightData=-((ADC_Value[0]-x_max)*999/(4095-x_max));
				}	
			}else if(ADC_Value[0]-x_min<0){
				/*前进*/
	
				/*计算转弯油门*/
				if(ADC_Value[1]-y_max>0){
					/*往右拐*/
					LeftData=(x_min-ADC_Value[0])*999/(x_min+4095-y_max);
					RightData=((x_min-ADC_Value[0])*999/(x_min+4095-y_max));
					RightData+=((ADC_Value[1]-y_max)*999/(x_min+4095-y_max));
					LeftData+=((ADC_Value[1]-y_max)*999/(x_min+4095-y_max));
					LeftData=-LeftData;
				}else if(ADC_Value[1]-y_min<0){
					/*往左拐*/
					LeftData=(x_min-ADC_Value[0])*999/(x_min+y_min);
					RightData=((x_min-ADC_Value[0])*999/(x_min+y_min));
					RightData+=((y_min-ADC_Value[1])*999/(x_min+y_min));
					LeftData+=((y_min-ADC_Value[1])*999/(x_min+y_min));
					RightData=-RightData;
				}else{
					/*直行*/
					LeftData=(x_min-ADC_Value[0])*999/(x_min);
					RightData=((x_min-ADC_Value[0])*999/(x_min));	
				}	
			}else{
					LeftData=0;
					RightData=0;
			}
		
				HubMotRight_Move(LeftData);
				HubMotLeft_Move(RightData);
			
		if(HAL_GPIO_ReadPin(SETP_Up_Manual_GPIO_Port,SETP_Up_Manual_Pin)==GPIO_PIN_RESET){
			osDelay( 10 );
			if(HAL_GPIO_ReadPin(SETP_Up_Manual_GPIO_Port,SETP_Up_Manual_Pin)==GPIO_PIN_RESET){
				if(StepMotdata.State!=MOTOSTATE_BUSY){
					STEPMOTOR_MoveContinuted(1);
			}		
			}
		}
		else if(HAL_GPIO_ReadPin(SETP_Down_Manual_GPIO_Port,SETP_Down_Manual_Pin)==GPIO_PIN_RESET){
			osDelay( 10 );
			if(HAL_GPIO_ReadPin(SETP_Down_Manual_GPIO_Port,SETP_Down_Manual_Pin)==GPIO_PIN_RESET){
				if(StepMotdata.State!=MOTOSTATE_BUSY){
					STEPMOTOR_MoveContinuted(-1);
			}		
			}
		}else{
				STEPMOTOR_STOP();
		}
		osDelay(5);
		
  }
}
/* myTaskPerception function */
void myTaskPerception(void const * argument)
{
	AutoModeTaskNumber=1;
	__IO float a_Min,a_Max,b,c;

float ChaoshengBoData[4];

uint32_t TimeOut=0,TimeOut1=0,TimeOut2=0;

  for(;;)
  {
		for(uint8_t n=0;n<4;n++){
			if(n<3){
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Horizontal[n];
			}else{
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Vertical[n];
			}
		}
		a_Min=Data_Get_Min(ChaoshengBoData[1],ChaoshengBoData[2]);
		a_Max=Data_Get_Min(ChaoshengBoData[1],ChaoshengBoData[2]);
		if(ChaoshengBoData[2]>=ChaoshengBoData[1]){
			b=(ChaoshengBoData[0]-(a_Max-a_Min)*ChaoShengBodata.SetData2[0].Data.floatdata/ChaoShengBodata.SetData2[1].Data.floatdata-a_Min)*Chair_sqrt(1/(Chair_add(1,Chair_mul((a_Max-a_Min)/ChaoShengBodata.SetData2[1].Data.floatdata,(a_Max-a_Min)/ChaoShengBodata.SetData2[1].Data.floatdata))));
		}else{
			b=(ChaoshengBoData[0]-(a_Max-a_Min)*(ChaoShengBodata.SetData2[1].Data.floatdata-ChaoShengBodata.SetData2[0].Data.floatdata)/ChaoShengBodata.SetData2[1].Data.floatdata-a_Min)*Chair_sqrt(1/(Chair_add(1,Chair_mul((a_Max-a_Min)/ChaoShengBodata.SetData2[1].Data.floatdata,(a_Max-a_Min)/ChaoShengBodata.SetData2[1].Data.floatdata))));
		}
		if(a_Min<ChaoShengBodata.SetData[0].Data.floatdata){
			TrackMotLeftStop();
			TrackMotRightStop();
			if(a_Max-a_Min<ChaoShengBodata.SetData[1].Data.floatdata){
				if(b>ChaoShengBodata.SetData[2].Data.floatdata*0.5&&b<1.5*ChaoShengBodata.SetData[2].Data.floatdata){
					
					/*检测到上楼梯*/
					/*创建上楼准备任务*/
					osThreadDef(myTaskUpStairs_Ready, myTaskUpStairs_Ready, osPriorityNormal, 5, 128);
					myTaskUpStairs_ReadyHandle = osThreadCreate(osThread(myTaskUpStairs_Ready), NULL);
					AutoModeTaskNumber=2;								
					vTaskDelete(NULL);
				}else{
					/*阶梯宽度不符合*/
					TrackMotLeftStop();
					TrackMotRightStop();
					/*手动 自动 灯闪烁*/
					uint8_t TxData=50;
					if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
					if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
					AutoModeTaskNumber=0;
					vTaskDelete(NULL);
			}
			}else{
					/*角度不符合*/
				TrackMotLeftStop();
				TrackMotRightStop();
				/*手动 自动 灯闪烁*/
				uint8_t TxData=50;
				if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
				if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
				AutoModeTaskNumber=0;
				vTaskDelete(NULL);
			}
			
		}
		else if(ChaoshengBoData[3]-ChaoShengBodata.SetData2[2].Data.floatdata>0.15*ChaoShengBodata.SetData[3].Data.floatdata){
			osDelay( 500 );
			TrackMotLeftStop();
			TrackMotRightStop();
			if(ChaoshengBoData[3]-ChaoShengBodata.SetData2[2].Data.floatdata>0.25*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[3]-ChaoShengBodata.SetData2[2].Data.floatdata<1.25*ChaoShengBodata.SetData[3].Data.floatdata){
				/*把超声波2 3 调竖直*/
				SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle1.Data.floatdata);
				SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle1.Data.floatdata);
				/*创建下楼准备任务 */
				osThreadDef(myTaskDownStairs_Ready, myTaskDownStairs_Ready, osPriorityNormal, 5, 128);
				myTaskDownStairs_ReadyHandle = osThreadCreate(osThread(myTaskDownStairs_Ready), NULL);
				AutoModeTaskNumber=6;
				vTaskDelete(NULL);
			}else{
				/*楼梯高度不符合*/
				TrackMotLeftStop();
				TrackMotRightStop();
				/*手动 自动 灯闪烁*/
				uint8_t TxData=50;
				if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
				if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
				AutoModeTaskNumber=0;
				vTaskDelete(NULL);
			}
		}
		else{
			TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
			TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);	
		}
//		/*超时判断*/
//		if(TimeOut++==5000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);
//			
//		}
		osDelay( 2 );
  }
}
/*调节轮椅上下楼方向*/
void myTaskUp_DownStairsDirConnectAct(void const * argument)
{
	float a,b;
	a=TrackMotRightData.SetSpeed.Data.floatdata;
	b=TrackMotLeftData.SetSpeed.Data.floatdata;
  for(;;)
  {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);  
		if(AutoModeTaskNumber==4||AutoModeTaskNumber==8){
		/*调节方向*/
		/*方向往右偏*/
		if(JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata<JY61_2.StepMotVibrationAngle1.Data.floatdata-1){
			/*向左调*/

			TrackMotRightData.SetSpeed.Data.floatdata=a*0.2;
			TrackMotLeftData.SetSpeed.Data.floatdata=b*1.8;
		}
		/*方向往左偏 */
		else if(JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata>JY61_2.StepMotVibrationAngle1.Data.floatdata+1){
			/*向右调*/
			TrackMotRightData.SetSpeed.Data.floatdata=a*1.8;
			TrackMotLeftData.SetSpeed.Data.floatdata=b*0.2;
			
		}else{
			TrackMotRightData.SetSpeed.Data.floatdata=a;
			TrackMotLeftData.SetSpeed.Data.floatdata=b;
		}
		}
  }
}
/*判断轮椅上下楼方向*/
void myTaskUp_DownStairsDirConnectJudge(void const * argument)
{
float a_Max,a_Min;
  for(;;)
  {
		/*判断方向*/
	if(AutoModeTaskNumber==4||AutoModeTaskNumber==8){
		if(JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata>JY61_2.StepMotVibrationAngle1.Data.floatdata+2.5){
			xTaskNotifyGive(myTaskUp_DownStairsDirConnectActHandle );				
		}
		else if(JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata<JY61_2.StepMotVibrationAngle1.Data.floatdata-2.5){
			xTaskNotifyGive(myTaskUp_DownStairsDirConnectActHandle );				
		}else{
		}
	}
		osDelay(10);
		
  }
}
/*爬最后一个楼梯任务 */
void myTaskUpStairs_LastStair(void const * argument)
{
	AutoModeTaskNumber=5;
	uint8_t a=0,d=0;
uint32_t TimeOut=0,TimeOut1=0,TimeOut2=0;
	float ChaoshengBoData[4];

	__IO float b,c;
			/*记录Z轴数据*/		
	b=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;	
	STEPMOTOR_MoveRel(-65);
	/*方向根据实际来*/
	while(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata+1>b&&StepMotdata.State!=MOTOSTATE_FREE){
		osDelay(2);
//		/*超时判断*/
//		if(TimeOut++==10000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}		
	}
	STEPMOTOR_STOP();
	/*步进电机持续往上走*/
//	STEPMOTOR_MoveContinuted(1);
	STEPMOTOR_MoveHoriizontal();
	TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
	TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
			SteeringEngineSetAngle(&SteeringEngine1data,SteeringEngine1data.SetAngle.Data.floatdata);
			SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle.Data.floatdata);
			SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle.Data.floatdata);

__IO uint16_t e=0,f=0;
  for(;;)
  {

//		if(d++==30){
//			d=0;
//			TrackMotRightStop();
//			TrackMotLeftStop();
//		}
		for(uint8_t n=0;n<4;n++){
			if(n<3){
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Horizontal[n];
			}
		}
		
	if((Data_Get_Max(Data_Get_Max(ChaoshengBoData[0],ChaoshengBoData[1]),ChaoshengBoData[2])<600||e++==2000)&&f==0){
			TrackMotRightStop();
			TrackMotLeftStop();	
			f=1;
	}
	if(f==1&&StepMotdata.State==MOTOSTATE_FREE){
			STEPMOTOR_STOP();
			TrackMotRightStop();
			TrackMotLeftStop();			
			
			uint8_t TxData=200;
			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
			/*完成上楼任务*/
			AutoModeTaskNumber=0;
			vTaskDelete(NULL);
	}

//		while(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata+6<JY61_2.StepMotVibrationAngle2.Data.floatdata ){
//				osDelay(2);
//		}
//		if(e++==300){
//			e=0;
//			TrackMotRightStop();
//			TrackMotLeftStop();
//		}
//		if(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata+3>JY61_2.StepMotVibrationAngle2.Data.floatdata){

//			STEPMOTOR_STOP();
//			TrackMotRightStop();
//			TrackMotLeftStop();			
//			
//			uint8_t TxData=200;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			/*完成上楼任务*/
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);
//		}
//		/*超时判断*/
//		if(TimeOut1++==20){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}	
		osDelay(20);
	}
}
/*爬楼任务 */
void myTaskUpStairs_Running(void const * argument)
{

	AutoModeTaskNumber=4;
	float ChaoshengBoData[4];
	uint8_t ChaoShengBo_Ready400=0,ChaoShengBo2_ReadyOK=0,ChaoShengBo3_ReadyOK=0;
	uint32_t TimeOut,TimeOut1;
__IO uint8_t DIR_Judge=0,DIR_Act=0;

//	float pid_chaoshengbodata=0,ek=0;
//	uint8_t n=0;
//	if(ChaoShengBodata.floatData[4].Data.floatdata<ChaoShengBodata.floatData[5].Data.floatdata){
//		pid_chaoshengbodata=ChaoShengBodata.floatData[4].Data.floatdata;
//		n=4;
//	}else{
//		pid_chaoshengbodata=ChaoShengBodata.floatData[5].Data.floatdata;
//		n=5;
//	}
//	arm_pid_instance_f32 pid_data;
//	pid_data.A0=0.5;
//	pid_data.A1=-0.5;
//	pid_data.A2=0;
//	pid_data.Kp=0.5;
//	pid_data.Ki=0;
//	pid_data.Kd=0;
//	pid_data.state[0]=0;
//	pid_data.state[1]=0;
//	pid_data.state[2]=0;
  for(;;)
  {
		for(uint8_t n=0;n<4;n++){
			if(n<3){
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Horizontal[n];
			}
		}
		
		if(Data_Get_Min(Data_Get_Min(ChaoshengBoData[0],ChaoshengBoData[1]),ChaoshengBoData[2])<ChaoShengBodata.SetData1[2].Data.floatdata){
			
			/*PID 控制超声波*/		
//			ek=pid_chaoshengbodata-ChaoShengBodata.floatData[n].Data.floatdata;
//			volatile float out;
//			out=arm_pid_f32(&pid_data,ek);
//			if(n==5){
//				out=-out;
//			}
//			if(out>0){
//				if(out>30){
//					out=30;
//				}
//				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*(1+out/120));
//				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*(1-out/120));
//				
//			}else if(out<0){
//				out=-out;
//				if(out>30){
//					out=30;
//				}
//				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*(1-out/120));
//				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*(1+out/120));
//				
//			}else{
//				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
//				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
//			}
			
			/*往左偏*/
			if(JY61_2.StepMotVibrationAngle1.Data.floatdata>JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata+4&&DIR_Judge==0){
				DIR_Judge=1;
				DIR_Act=1;
			/*往右偏*/
			}else if(JY61_2.StepMotVibrationAngle1.Data.floatdata<JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata-4&&DIR_Judge==0){
				DIR_Judge=1;
				DIR_Act=1;
			}else{
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
			}
			/*往左偏*/
			if(JY61_2.StepMotVibrationAngle1.Data.floatdata>JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata+0.2&&DIR_Act==1){
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*1.5);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*0.5);
			/*往右偏*/
			}else if(JY61_2.StepMotVibrationAngle1.Data.floatdata<JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata-0.2&&DIR_Act==1){
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*0.5);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*1.5);
			}else{
				DIR_Act=0;
				DIR_Judge=0;
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
			}
	
		}else{
			TrackMotRightStop();
			TrackMotLeftStop();
			/*启动最后一个阶梯任务*/
			osThreadDef(myTaskUpStairs_LastStair, myTaskUpStairs_LastStair, osPriorityNormal, 4, 128);
			myTaskUpStairs_LastStairHandle = osThreadCreate(osThread(myTaskUpStairs_LastStair), NULL);
			
//		TrackMotRightData.SetSpeed.Data.floatdata=500;
//		TrackMotLeftData.SetSpeed.Data.floatdata=500;

			AutoModeTaskNumber=5;
			vTaskDelete(NULL);
			
		}
//		/*超时判断*/
//		if(TimeOut++==60000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}

//			vTaskDelete(myTaskUp_DownStairsDirConnectJudgeHandle);
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}		
		
		osDelay(600);
	}
}
/*爬楼准备1任务 */
void myTaskUpStairs_Ready1(void const * argument)
{
	AutoModeTaskNumber=3;
	float ChaoshengBoData[4];
	uint8_t ChaoShengBo_Ready400=0,ChaoShengBo2_ReadyOK=0,ChaoShengBo3_ReadyOK=0;
	uint32_t TimeOut=0,TimeOut1=0,TimeOut2=0;
	TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
	TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
  for(;;)
  {
		for(uint8_t n=0;n<4;n++){
			if(n<3){
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Horizontal[n];
			}
		}

		if(Data_Get_Min(ChaoshengBoData[1],ChaoshengBoData[2])<=ChaoShengBodata.SetData1[1].Data.floatdata){
			TrackMotLeftStop();
			TrackMotRightStop();
			STEPMOTOR_MoveRel(-35);
			while(StepMotdata.State!=MOTOSTATE_FREE){

					osDelay(2);
//				/*超时判断*/
//				if(TimeOut1++==5000){
//					/*超时错误处理*/
//					TrackMotLeftStop();
//					TrackMotRightStop();
//					STEPMOTOR_STOP();
//					/*手动 自动 灯闪烁*/
//					uint8_t TxData=50;
//					if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//					if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//					AutoModeTaskNumber=0;
//					vTaskDelete(NULL);			
//				}		
			}
			/*调整角度*/
			float a;
			if(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata>JY61_2.StepMotVibrationAngle2.Data.floatdata){
				a=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata-JY61_2.StepMotVibrationAngle2.Data.floatdata;
			}else{
				a=JY61_2.StepMotVibrationAngle2.Data.floatdata-JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
			}
			SteeringEngineSetAngle(&SteeringEngine1data,SteeringEngine1data.SetAngle.Data.floatdata-30);
			SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle.Data.floatdata-30);
			SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle.Data.floatdata+30);
			osDelay(1500);
			/*开始爬楼任务*/
			osThreadDef(myTaskUpStairs_Running, myTaskUpStairs_Running, osPriorityNormal, 5, 128);
			myTaskUpStairs_RunningHandle = osThreadCreate(osThread(myTaskUpStairs_Running), NULL);
			AutoModeTaskNumber=4;
			vTaskDelete(NULL);
		}
//		if(ChaoshengBoData[1]<=ChaoShengBodata.SetData1[1].Data.floatdata){
//			TrackMotRightStop();
//		}	else{
//			TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
//		}
//		if(ChaoshengBoData[2]<=ChaoShengBodata.SetData1[1].Data.floatdata){
//			TrackMotLeftStop();
//		}	else{
//			TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
//		}
//		/*超时判断*/
//		if(TimeOut++==5000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}		
		osDelay(2);
	}
}
/*爬楼准备任务 */
void myTaskUpStairs_Ready(void const * argument)
{
	/*摆臂 向上 摆到指定角度*/	
	STEPMOTOR_MoveRel(35);
	AutoModeTaskNumber=2;
	float ChaoshengBoData[4];
	uint8_t ChaoShengBo_Ready400=0,ChaoShengBo2_ReadyOK=0,ChaoShengBo3_ReadyOK=0;
	uint32_t TimeOut=0,TimeOut1=0,TimeOut2=0;
  for(;;)
  {
		for(uint8_t n=0;n<4;n++){
			if(n<3){
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Horizontal[n];
			}
		}
	
		if(ChaoshengBoData[1]<=ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==1){
			TrackMotRightStop();
			ChaoShengBo2_ReadyOK=1;
			/*超声波2 调整好*/
			if(ChaoShengBo3_ReadyOK==1){
				
				/*超声波2 3 调整好*/
				/*步进电机停止运动*/
				while(StepMotdata.State!=MOTOSTATE_FREE){
					
						osDelay(2);
//				/*超时判断*/
//				if(TimeOut1++==5000){
//					/*超时错误处理*/
//					TrackMotLeftStop();
//					TrackMotRightStop();
//					STEPMOTOR_STOP();
//					/*手动 自动 灯闪烁*/
//					uint8_t TxData=50;
//					if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//					if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//					AutoModeTaskNumber=0;
//					vTaskDelete(NULL);			
//				}	
				}
				/*记录Z轴数据*/		
				JY61_2.StepMotVibrationAngle1.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata;	
				JY61_2.StepMotVibrationAngle2.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;				
				/*进行下个任务*/
				osThreadDef(myTaskUpStairs_Ready1, myTaskUpStairs_Ready1, osPriorityNormal, 5, 128);
				myTaskUpStairs_Ready1Handle = osThreadCreate(osThread(myTaskUpStairs_Ready1), NULL);
				AutoModeTaskNumber=3;
				vTaskDelete(NULL);
			
			}
		}else if(ChaoshengBoData[1]>ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==1){
			TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
		}
		if(ChaoshengBoData[2]<=ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==1){
			TrackMotLeftStop();		
			ChaoShengBo3_ReadyOK=1;
			/*超声波2 调整好*/
			if(ChaoShengBo2_ReadyOK==1){
				
			/*超声波2 3 调整好*/
			/*步进电机停止运动*/
			while(StepMotdata.State!=MOTOSTATE_FREE){
				/*超时判断*/
//				if(TimeOut1++==5000){
//					/*超时错误处理*/
//					TrackMotLeftStop();
//					TrackMotRightStop();
//					STEPMOTOR_STOP();
//					/*手动 自动 灯闪烁*/
//					uint8_t TxData=50;
//					if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//					if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//					AutoModeTaskNumber=0;
//					vTaskDelete(NULL);			
//				}	
					osDelay(2);
			}
				/*记录Z轴数据*/		
				JY61_2.StepMotVibrationAngle1.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata;	
				JY61_2.StepMotVibrationAngle2.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;				
			
				/*进行下个任务*/
				osThreadDef(myTaskUpStairs_Ready1, myTaskUpStairs_Ready1, osPriorityNormal, 5, 128);
				myTaskUpStairs_Ready1Handle = osThreadCreate(osThread(myTaskUpStairs_Ready1), NULL);
				AutoModeTaskNumber=3;
				vTaskDelete(NULL);
			
			}	
		}else if(ChaoshengBoData[2]>ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==1){
			TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
		}
		
		/*超声波同时 400 外*/
		if(ChaoshengBoData[1]>ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoshengBoData[2]>ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==0){
			TrackMotLeftStop();
			TrackMotRightStop();
			ChaoShengBo_Ready400=1;
		}else if(ChaoshengBoData[1]>ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoshengBoData[2]<ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==0){
			TrackMotRightBack(TrackMotRightData.SetSpeed.Data.floatdata);
			TrackMotLeftStop();
		}else if(ChaoshengBoData[1]<ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoshengBoData[2]>ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==0){
			TrackMotLeftBack(TrackMotLeftData.SetSpeed.Data.floatdata);
			TrackMotRightStop();
		}else if(ChaoshengBoData[1]<ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoshengBoData[2]<ChaoShengBodata.SetData1[0].Data.floatdata&&ChaoShengBo_Ready400==0){
			TrackMotLeftBack(TrackMotLeftData.SetSpeed.Data.floatdata);
			TrackMotRightBack(TrackMotRightData.SetSpeed.Data.floatdata);
		}
		
//		/*超时判断*/
//		if(TimeOut++==10000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}		
		

		osDelay(2);
  }
}
/*下最后一个楼梯任务 */
void myTaskDownStairs_LastStair(void const * argument)
{	
	AutoModeTaskNumber=9;
	__IO float a;
	a=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
	TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*0.5);
	TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*0.5);
	uint32_t TimeOut,TimeOut1;

  for(;;)
  {

		if(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata<JY61_2.StepMotVibrationAngle2.Data.floatdata+5){
			osDelay(6000);
			TrackMotRightStop();
			TrackMotLeftStop();
			uint8_t TxData=200;
			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
			/*完成下楼任务*/
			AutoModeTaskNumber=0;
			vTaskDelete(NULL);
		}
//			/*超时判断*/
//		if(TimeOut++==100){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}

		osDelay(100);
  }
}
/*下楼任务 */
void myTaskDownStairs_Running(void const * argument)
{	
	AutoModeTaskNumber=8;
	float ChaoshengBoData[4];

	uint32_t TimeOut=0,TimeOut1=0,TimeOut2=0;
	__IO float a;
	a=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
__IO uint8_t DIR_Judge=0,DIR_Act=0;
//	float pid_chaoshengbodata=0,ek=0;


//	uint8_t n=0;
//	if(ChaoShengBodata.floatData[4].Data.floatdata<ChaoShengBodata.floatData[5].Data.floatdata){
//		pid_chaoshengbodata=ChaoShengBodata.floatData[4].Data.floatdata;
//		n=4;
//	}else{
//		pid_chaoshengbodata=ChaoShengBodata.floatData[5].Data.floatdata;
//		n=5;
//	}
//	pid_chaoshengbodata=ChaoShengBodata.floatData[5].Data.floatdata;
//	arm_pid_instance_f32 pid_data1;
//	pid_data1.A0=0.6;
//	pid_data1.A1=-0.6;
//	pid_data1.A2=0;
//	pid_data1.Kp=0.6;
//	pid_data1.Ki=0;
//	pid_data1.Kd=0;
//	pid_data1.state[0]=0;
//	pid_data1.state[1]=0;
//	pid_data1.state[2]=0;


  for(;;)
  {
		for(uint8_t n=0;n<4;n++){
			if(n<3){
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Horizontal[n];
			}
		}
		if(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata>JY61_2.StepMotVibrationAngle2.Data.floatdata+11){
			
//			/*PID 控制超声波*/
//			ek=pid_chaoshengbodata-ChaoShengBodata.floatData[5].Data.floatdata;
//			volatile float out;
//			out=arm_pid_f32(&pid_data1,ek);
//			if(n==5){
//				out=-out;
//			}
//			if(out>0){
//				if(out>30){
//					out=30;
//				}
//				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*(1+out/200));
//				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*(1-out/200));
//				
//			}else if(out<0){
//				out=-out;
//				if(out>30){
//					out=30;
//				}
//				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*(1-out/200));
//				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*(1+out/200));
//				
//			}else{
//				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
//				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
//			}


			/*往左偏*/
			if(JY61_2.StepMotVibrationAngle1.Data.floatdata>JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata+4&&DIR_Judge==0){
				DIR_Judge=1;
				DIR_Act=1;
			/*往右偏*/
			}else if(JY61_2.StepMotVibrationAngle1.Data.floatdata<JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata-4&&DIR_Judge==0){
				DIR_Judge=1;
				DIR_Act=1;
			}else{
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
			}
			/*往左偏*/
			if(JY61_2.StepMotVibrationAngle1.Data.floatdata>JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata+0.3&&DIR_Act==1){
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*1.5);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*0.5);
			/*往右偏*/
			}else if(JY61_2.StepMotVibrationAngle1.Data.floatdata<JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata-0.3&&DIR_Act==1){
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*0.5);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*1.5);

			}else{
				DIR_Act=0;
				DIR_Judge=0;
				TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
				TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
			}
		}else{
			TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata*0.5);
			TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata*0.5);
			/*启动最后一个阶梯任务*/
			osThreadDef(myTaskDownStairs_LastStair, myTaskDownStairs_LastStair, osPriorityNormal, 4, 128);
			myTaskDownStairs_LastStairHandle = osThreadCreate(osThread(myTaskDownStairs_LastStair), NULL);

			AutoModeTaskNumber=9;
			vTaskDelete(NULL);
		}
//			/*超时判断*/
//		if(TimeOut++==60000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			vTaskDelete(myTaskUp_DownStairsDirConnectJudgeHandle);
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}	
		osDelay(600);
  }
}
/*下楼准备1任务 */
void myTaskDownStairs_Ready1(void const * argument)
{
	AutoModeTaskNumber=7;
uint32_t TimeOut=0,TimeOut1=0,TimeOut2=0;
	float ChaoshengBoData[4];

	uint8_t ChaoShengBo_ReadyEarth=0,ChaoShengBo2_ReadyOK=0,ChaoShengBo3_ReadyOK=0;
	__IO float a;
	a=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
  for(;;)
  {
		TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
		TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
		while(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata<a+16){
			osDelay(2);
//			/*超时判断*/
//		if(TimeOut1++==10000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}	
		}
		TrackMotRightStop();
		TrackMotLeftStop();

		STEPMOTOR_STOP();
		STEPMOTOR_MoveHoriizontal();
		/*步进电机停止运动*/
		while(StepMotdata.State!=MOTOSTATE_FREE){
			
				osDelay(2);
//			/*超时判断*/
//		if(TimeOut2++==10000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}	
		}
		/*调整角度*/
		float a;
		if(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata>JY61_2.StepMotVibrationAngle2.Data.floatdata){
			a=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata-JY61_2.StepMotVibrationAngle2.Data.floatdata;
		}else{
			a=JY61_2.StepMotVibrationAngle2.Data.floatdata-JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
		}
		SteeringEngineSetAngle(&SteeringEngine1data,SteeringEngine1data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle.Data.floatdata);
		osDelay(1500);
		/*开始下楼任务*/
		osThreadDef(myTaskDownStairs_Running, myTaskDownStairs_Running, osPriorityNormal, 5, 128);
		myTaskDownStairs_RunningHandle = osThreadCreate(osThread(myTaskDownStairs_Running), NULL);
		AutoModeTaskNumber=8;
		vTaskDelete(NULL);	
//			/*超时判断*/
//		if(TimeOut++==2000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}	
		osDelay(2);
  }
}
/*下楼准备任务 */
void myTaskDownStairs_Ready(void const * argument)
{	
	AutoModeTaskNumber=6;
	uint32_t TimeOut=0,TimeOut1=0,TimeOut2=0;
	float ChaoshengBoData[4];
	uint8_t ChaoShengBo_ReadyEarth=0,ChaoShengBo2_ReadyOK=0,ChaoShengBo3_ReadyOK=0;
  for(;;)
  {
		for(uint8_t n=0;n<4;n++){
			if(n>0){
				ChaoshengBoData[n]=ChaoShengBodata.floatData[n].Data.floatdata+ChaoShengBoOffSetData_Vertical[n];
			}
		}
		
		if((ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata>=0.75*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata<=1.25*ChaoShengBodata.SetData[3].Data.floatdata)&&ChaoShengBo_ReadyEarth==1){
			TrackMotRightStop();
			
			ChaoShengBo2_ReadyOK=1;
			/*超声波2 调整好*/
			if(ChaoShengBo3_ReadyOK==1){		
			/*超声波2 3 调整好*/
				/*记录Z轴数据*/		
				JY61_2.StepMotVibrationAngle1.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata;	
				JY61_2.StepMotVibrationAngle2.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;	
				__IO float a;
				a=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
				STEPMOTOR_STOP();
				STEPMOTOR_MoveRel(-60);
				while(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata>JY61_2.StepMotVibrationAngle2.Data.floatdata-1&&StepMotdata.State!=MOTOSTATE_FREE){
					osDelay(10);
						/*超时判断*/
//					if(TimeOut1++==2000){
//						/*超时错误处理*/
//						TrackMotLeftStop();
//						TrackMotRightStop();
//						STEPMOTOR_STOP();
//						/*手动 自动 灯闪烁*/
//						uint8_t TxData=50;
//						if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//						if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//						AutoModeTaskNumber=0;
//						vTaskDelete(NULL);			
//					}	
				}
				STEPMOTOR_STOP();
			
			/*下一个任务 下楼任务*/
				osThreadDef(myTaskDownStairs_Ready1, myTaskDownStairs_Ready1, osPriorityNormal, 5, 128);
				myTaskDownStairs_Ready1Handle = osThreadCreate(osThread(myTaskDownStairs_Ready1), NULL);
				AutoModeTaskNumber=7;
				vTaskDelete(NULL);
			}	
		}else if(ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata<0.15*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoShengBo_ReadyEarth==1){
			TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
		}
		if((ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata>=0.75*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata<=1.25*ChaoShengBodata.SetData[3].Data.floatdata)&&ChaoShengBo_ReadyEarth==1){
			TrackMotLeftStop();		
			ChaoShengBo3_ReadyOK=1;
			/*超声波2 调整好*/
			if(ChaoShengBo2_ReadyOK==1){	
			/*超声波2 3 调整好*/
							/*记录Z轴数据*/		
				JY61_2.StepMotVibrationAngle1.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata;		
				JY61_2.StepMotVibrationAngle2.Data.floatdata=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
				__IO float a;
				a=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata;
				STEPMOTOR_STOP();
				STEPMOTOR_MoveContinuted(-1);
				while(JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata>JY61_2.StepMotVibrationAngle2.Data.floatdata-3.5){
					osDelay(10);
//						/*超时判断*/
//					if(TimeOut1++==2000){
//						/*超时错误处理*/
//						TrackMotLeftStop();
//						TrackMotRightStop();
//						STEPMOTOR_STOP();
//						/*手动 自动 灯闪烁*/
//						uint8_t TxData=50;
//						if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//						if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//						AutoModeTaskNumber=0;
//						vTaskDelete(NULL);			
//					}	
				}
				STEPMOTOR_STOP();

				
			/*下一个任务 下楼任务*/
				osThreadDef(myTaskDownStairs_Ready1, myTaskDownStairs_Ready1, osPriorityNormal, 5, 128);
				myTaskDownStairs_Ready1Handle = osThreadCreate(osThread(myTaskDownStairs_Ready1), NULL);
				AutoModeTaskNumber=7;
				vTaskDelete(NULL);
			}	
		}else if(ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata<0.15*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoShengBo_ReadyEarth==1){
			TrackMotLeftForward(TrackMotRightData.SetSpeed.Data.floatdata);
		}
		
		/*超声波同时在地面*/
		if(ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata<=0.15*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata<=0.15*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoShengBo_ReadyEarth==0){
			TrackMotLeftStop();
			TrackMotRightStop();
			ChaoShengBo_ReadyEarth=1;
		}else if((ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata>0.75*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata<1.25*ChaoShengBodata.SetData[3].Data.floatdata)&&ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata<0.15*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoShengBo_ReadyEarth==0){
			TrackMotRightBack(TrackMotRightData.SetSpeed.Data.floatdata);
			TrackMotLeftStop();
		}else if((ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata>0.75*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata<1.25*ChaoShengBodata.SetData[3].Data.floatdata)&&ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata<0.15*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoShengBo_ReadyEarth==0){
			TrackMotLeftBack(TrackMotLeftData.SetSpeed.Data.floatdata);
			TrackMotRightStop();
		}else if((ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata>0.75*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[2]-ChaoShengBodata.SetData2[2].Data.floatdata<1.25*ChaoShengBodata.SetData[3].Data.floatdata)&&(ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata>0.75*ChaoShengBodata.SetData[3].Data.floatdata&&ChaoshengBoData[1]-ChaoShengBodata.SetData2[2].Data.floatdata<1.25*ChaoShengBodata.SetData[3].Data.floatdata)&&ChaoShengBo_ReadyEarth==0){
			TrackMotLeftBack(TrackMotLeftData.SetSpeed.Data.floatdata);
			TrackMotRightBack(TrackMotRightData.SetSpeed.Data.floatdata);
		}
//			/*超时判断*/
//		if(TimeOut++==5000){
//			/*超时错误处理*/
//			TrackMotLeftStop();
//			TrackMotRightStop();
//			STEPMOTOR_STOP();
//			/*手动 自动 灯闪烁*/
//			uint8_t TxData=50;
//			if(xQueueSend( xQueue_AutoModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			if(xQueueSend( xQueue_ManualModeLedWinkle, ( void * ) &TxData,(TickType_t)0 )==pdPASS){}
//			AutoModeTaskNumber=0;
//			vTaskDelete(NULL);			
//		}		
	osDelay(2);
  }
}
/*调节轮椅平衡*/
void myTaskChairBalanceAct(void const * argument)
{

  for(;;)
  {
		/*调节平衡*/
		if(JY61_1.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata>(JY61_1.ChairBalanceAngleMax.Data.floatdata+JY61_1.ChairBalanceAngleMin.Data.floatdata)/2.0+0.2){
			PushRodFrontDown();
			PushRodBehindUp();
		}
		else if(JY61_1.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata<(JY61_1.ChairBalanceAngleMax.Data.floatdata+JY61_1.ChairBalanceAngleMin.Data.floatdata)/2.0-0.2){
			PushRodFrontUp();
			PushRodBehindDown();
		}else{
			PushRodFrontStop();
			PushRodBehindStop();
			osThreadDef(myTaskChairBalanceJudge, myTaskChairBalanceJudge, osPriorityNormal, 4, 128);
			myTaskChairBalanceJudgeHandle = osThreadCreate(osThread(myTaskChairBalanceJudge), NULL);
			vTaskDelete(NULL);

		}
		osDelay(4);
  }
}
/*判断轮椅平衡*/
void myTaskChairBalanceJudge(void const * argument)
{

  for(;;)
  {
	if(JY61_1.ReceiveData_OK==CMD_OPEN){
		/*调节平衡*/
		if(JY61_1.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata>JY61_1.ChairBalanceAngleMax.Data.floatdata){
			osThreadDef(myTaskChairBalanceAct, myTaskChairBalanceAct, osPriorityNormal, 4, 128);
			myTaskChairBalanceActHandle = osThreadCreate(osThread(myTaskChairBalanceAct), NULL);
			vTaskDelete(NULL);

		}
		else if(JY61_1.Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata<JY61_1.ChairBalanceAngleMin.Data.floatdata){
			osThreadDef(myTaskChairBalanceAct, myTaskChairBalanceAct, osPriorityNormal, 4, 128);
			myTaskChairBalanceActHandle = osThreadCreate(osThread(myTaskChairBalanceAct), NULL);
			vTaskDelete(NULL);

		}else{
			PushRodFrontStop();
			PushRodBehindStop();
		}
	}	
	osDelay(4);
	
  }
}

void myTaskJY61_2Data(void const * argument)
{
  for(;;)
  {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);  
		/*计算JY61_2姿态三个角度*/
		Roll_Pitch_Yaw_Data_Count(&JY61_2);
		
  }
}
void myTaskCMD(void const * argument)
{

  for(;;)
  {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);  
		/*Right DC Motor 行走右履带电机*/
		if(CMD_TrackMotRightForward.Data==CMD_OPEN){
			CMD_TrackMotRightForward.Data=CMD_CLOSE;
			TrackMotRightForward(TrackMotRightData.SetSpeed.Data.floatdata);
		}
		else if(CMD_TrackMotRightBack.Data==CMD_OPEN){
			CMD_TrackMotRightBack.Data=CMD_CLOSE;
			TrackMotRightBack(TrackMotRightData.SetSpeed.Data.floatdata);
		}
		else if(CMD_TrackMotRightStop.Data==CMD_OPEN){
			CMD_TrackMotRightStop.Data=CMD_CLOSE;
			TrackMotRightStop();
		}
		
		/*Left DC Motor 行走左履带电机*/
	  if(CMD_TrackMotLeftForward.Data==CMD_OPEN){
			CMD_TrackMotLeftForward.Data=CMD_CLOSE;
			TrackMotLeftForward(TrackMotLeftData.SetSpeed.Data.floatdata);
		}
		else if(CMD_TrackMotLeftBack.Data==CMD_OPEN){
			CMD_TrackMotLeftBack.Data=CMD_CLOSE;
			TrackMotLeftBack(TrackMotLeftData.SetSpeed.Data.floatdata);
		}
		else if(CMD_TrackMotLeftStop.Data==CMD_OPEN){
			CMD_TrackMotLeftStop.Data=CMD_CLOSE;
			TrackMotLeftStop();
		}
		/*舵机1*/
		if(CMD_SteeringEngine1Add.Data==CMD_OPEN){
			CMD_SteeringEngine1Add.Data=CMD_CLOSE;
			SteeringEngineSetAngle(&SteeringEngine1data,SteeringEngine1data.OffsetAngle.Data.floatdata);
		}
		/*舵机2*/
		else if(CMD_SteeringEngine2Add.Data==CMD_OPEN){
			CMD_SteeringEngine2Add.Data=CMD_CLOSE;
			SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine1data.OffsetAngle.Data.floatdata);
		}
		/*舵机3*/
		else if(CMD_SteeringEngine3Add.Data==CMD_OPEN){
			CMD_SteeringEngine3Add.Data=CMD_CLOSE;
			SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine1data.OffsetAngle.Data.floatdata);
		}
		/*舵机4*/
		else if(CMD_SteeringEngine4Add.Data==CMD_OPEN){
			CMD_SteeringEngine4Add.Data=CMD_CLOSE;
			SteeringEngineSetAngle(&SteeringEngine4data,SteeringEngine1data.OffsetAngle.Data.floatdata);
		}
		/*前推杆电机 上*/
		if(CMD_PushRodFrontUp.Data ==CMD_OPEN){
			CMD_PushRodFrontUp.Data =CMD_CLOSE;
			PushRodFrontUp();
		}
		/*前推杆电机 下*/
		if(CMD_PushRodFrontDown.Data ==CMD_OPEN){
			CMD_PushRodFrontDown.Data =CMD_CLOSE;
			PushRodFrontDown();
		}
		/*前推杆电机 停止*/
		if(CMD_PushRodFrontStop.Data ==CMD_OPEN){
			CMD_PushRodFrontStop.Data =CMD_CLOSE;
			PushRodFrontStop();
		}
		/*后推杆电机 上*/
		if(CMD_PushRodBehindUp.Data ==CMD_OPEN){
			CMD_PushRodBehindUp.Data =CMD_CLOSE;
			PushRodBehindUp();
		}
		/*后推杆电机 下*/
		if(CMD_PushRodBehindDown.Data ==CMD_OPEN){
			CMD_PushRodBehindDown.Data =CMD_CLOSE;
			PushRodBehindDown();
		}
		/*后 推杆电机 停止*/
		if(CMD_PushRodBehindStop.Data ==CMD_OPEN){
			CMD_PushRodBehindStop.Data =CMD_CLOSE;
			PushRodBehindStop();
		}
		/*步进电机*/
		if(CMD_StepMotUpContinued.Data==CMD_OPEN){
			CMD_StepMotUpContinued.Data=CMD_CLOSE;
			if(StepMotdata.State!=MOTOSTATE_BUSY){

				STEPMOTOR_MoveContinuted(1);


//				STEPMOTOR_AxisMoveRel_lin1(StepMotdata.OffsetAngle.Data.floatdata*25600*80/360,600,600,StepMotdata.Speed.Data.floatdata);
//	//			StepMotdata.CurrentAngle.Data.floatdata+=StepMotdata.OffsetAngle.Data.floatdata;
			}
		}
		else if(CMD_StepMotDownContinued.Data==CMD_OPEN){
			CMD_StepMotDownContinued.Data=CMD_CLOSE;
			if(StepMotdata.State!=MOTOSTATE_BUSY){

				STEPMOTOR_MoveContinuted(-1);

//				StepMotdata.ContinueMode=MOTOSTATE_BUSY;
//				STEPMOTOR_AxisMoveRel_lin1(-StepMotdata.OffsetAngle.Data.floatdata*25600*80/360,600,600,StepMotdata.Speed.Data.floatdata);
			}
			}else if(CMD_StepMotStopContinued.Data==CMD_OPEN){
			CMD_StepMotStopContinued.Data=CMD_CLOSE;		
			STEPMOTOR_STOP();
		}	
		else if(CMD_StepMotUpOneStep.Data==CMD_OPEN){
			CMD_StepMotUpOneStep.Data=CMD_CLOSE;
			if(StepMotdata.State!=MOTOSTATE_BUSY){
				
//				STEPMOTOR_MoveRel(StepMotdata.OffsetAngle.Data.floatdata);
				STEPMOTOR_MoveHoriizontal();
//				StepMotdata.ContinueMode=MOTOSTATE_FREE;
//				STEPMOTOR_AxisMoveRel_lin1(StepMotdata.OffsetAngle.Data.floatdata*25600*80/360,600,600,StepMotdata.Speed.Data.floatdata);
//	//			StepMotdata.CurrentAngle.Data.floatdata+=StepMotdata.OffsetAngle.Data.floatdata;
			}
		}
		else if(CMD_StepMotDownOneStep.Data==CMD_OPEN){
			CMD_StepMotDownOneStep.Data=CMD_CLOSE;
		if(StepMotdata.State!=MOTOSTATE_BUSY){
			
//			STEPMOTOR_MoveRel(-StepMotdata.OffsetAngle.Data.floatdata);
			STEPMOTOR_MoveHoriizontal();
//				StepMotdata.ContinueMode=MOTOSTATE_FREE;
//				STEPMOTOR_AxisMoveRel_lin1(-StepMotdata.OffsetAngle.Data.floatdata*25600*80/360,600,600,StepMotdata.Speed.Data.floatdata);
			}
		}

  }
}

void myTaskChaoShengBoTask(void const * argument)
{
	__IO	uint16_t range;
	__IO float Distant;
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
			/*超声波修改地址*/
//		osDelay(2000);
//		KS103_WriteOneByte(0XE8,2,0x9a);
//		osDelay(1);
//		KS103_WriteOneByte(0XE8,2,0x92);
//		osDelay(1);		 
//		KS103_WriteOneByte(0XE8,2,0x9e);
//		osDelay(1);
//		KS103_WriteOneByte(0XE8,2,0xd0);
//		osDelay(500);	
	
//
	uint8_t a,b;
  for(;;)
  {
		for(uint8_t n=0;n<6;n++){
			KS103_WriteOneByte(ChaoShengBodata.Add[n],0X02,0XB4);
			osDelay(90);
			ChaoShengBodata.Data[n]= KS103_ReadOneByte(ChaoShengBodata.Add[n], 0x02);
			ChaoShengBodata.Data[n] <<= 8;
			ChaoShengBodata.Data[n] += KS103_ReadOneByte(ChaoShengBodata.Add[n], 0x03);
			ChaoShengBodata.floatData[n].Data.floatdata=ChaoShengBodata.Data[n];
		}
		if(b==0&&a++==10){
			b=1;
			/*超声波校准任务*/
//			xTaskNotifyGive( myTaskChaoShengBo_CalibrateHandle );
		}
  }
  /* USER CODE END StartDefaultTask */
}

void myTaskChaoShengBo_Calibrate(void const * argument)
{
  for(;;)
  {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);  
		
		SteeringEngineSetAngle(&SteeringEngine1data,SteeringEngine1data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle.Data.floatdata);
		/*等待摆到水平位置*/
		osDelay(2500);

		/*1<2*/
		if(ChaoShengBodata.floatData[0].Data.floatdata<ChaoShengBodata.floatData[1].Data.floatdata){
			/*1<3*/
			if(ChaoShengBodata.floatData[0].Data.floatdata<ChaoShengBodata.floatData[2].Data.floatdata){
						ChaoShengBoOffSetData_Horizontal[0]=0;
						ChaoShengBoOffSetData_Horizontal[1]=ChaoShengBodata.floatData[0].Data.floatdata-ChaoShengBodata.floatData[1].Data.floatdata;
						ChaoShengBoOffSetData_Horizontal[2]=ChaoShengBodata.floatData[0].Data.floatdata-ChaoShengBodata.floatData[2].Data.floatdata;
			}else{
						ChaoShengBoOffSetData_Horizontal[0]=ChaoShengBodata.floatData[2].Data.floatdata-ChaoShengBodata.floatData[0].Data.floatdata;
						ChaoShengBoOffSetData_Horizontal[1]=ChaoShengBodata.floatData[2].Data.floatdata-ChaoShengBodata.floatData[1].Data.floatdata;
						ChaoShengBoOffSetData_Horizontal[2]=0;	
			}
		}else if(ChaoShengBodata.floatData[1].Data.floatdata<ChaoShengBodata.floatData[2].Data.floatdata){
						ChaoShengBoOffSetData_Horizontal[0]=ChaoShengBodata.floatData[1].Data.floatdata-ChaoShengBodata.floatData[0].Data.floatdata;
						ChaoShengBoOffSetData_Horizontal[1]=0;
						ChaoShengBoOffSetData_Horizontal[2]=ChaoShengBodata.floatData[1].Data.floatdata-ChaoShengBodata.floatData[2].Data.floatdata;
		}
		
		SteeringEngineSetAngle(&SteeringEngine4data,SteeringEngine4data.SetAngle1.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle1.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle1.Data.floatdata);
		/*等待摆到垂直位置*/
		osDelay(2500);

		/*1<2*/
		if(ChaoShengBodata.floatData[3].Data.floatdata<ChaoShengBodata.floatData[1].Data.floatdata){
			/*1<3*/
			if(ChaoShengBodata.floatData[3].Data.floatdata<ChaoShengBodata.floatData[2].Data.floatdata){
						ChaoShengBoOffSetData_Vertical[3]=0;
						ChaoShengBodata.SetData2[2].Data.floatdata=ChaoShengBodata.floatData[3].Data.floatdata;
						AT24CXX_Write(ChaoShengBodata.SetData2[2].Add,(uint8_t*)ChaoShengBodata.SetData2[2].Data.chardata,4);
						ChaoShengBoOffSetData_Vertical[1]=ChaoShengBodata.floatData[3].Data.floatdata-ChaoShengBodata.floatData[1].Data.floatdata;
						ChaoShengBoOffSetData_Vertical[2]=ChaoShengBodata.floatData[3].Data.floatdata-ChaoShengBodata.floatData[2].Data.floatdata;
			}else{
						ChaoShengBoOffSetData_Vertical[3]=ChaoShengBodata.floatData[2].Data.floatdata-ChaoShengBodata.floatData[3].Data.floatdata;
						ChaoShengBoOffSetData_Vertical[1]=ChaoShengBodata.floatData[2].Data.floatdata-ChaoShengBodata.floatData[1].Data.floatdata;
						ChaoShengBoOffSetData_Vertical[2]=0;	
						ChaoShengBodata.SetData2[2].Data.floatdata=ChaoShengBodata.floatData[2].Data.floatdata;
						AT24CXX_Write(ChaoShengBodata.SetData2[2].Add,(uint8_t*)ChaoShengBodata.SetData2[2].Data.chardata,4);
			}
		}else if(ChaoShengBodata.floatData[1].Data.floatdata<ChaoShengBodata.floatData[2].Data.floatdata){
						ChaoShengBoOffSetData_Vertical[3]=ChaoShengBodata.floatData[1].Data.floatdata-ChaoShengBodata.floatData[3].Data.floatdata;
						ChaoShengBoOffSetData_Vertical[1]=0;
						ChaoShengBodata.SetData2[2].Data.floatdata=ChaoShengBodata.floatData[1].Data.floatdata;
						AT24CXX_Write(ChaoShengBodata.SetData2[2].Add,(uint8_t*)ChaoShengBodata.SetData2[2].Data.chardata,4);
						ChaoShengBoOffSetData_Vertical[2]=ChaoShengBodata.floatData[1].Data.floatdata-ChaoShengBodata.floatData[2].Data.floatdata;
		}
		SteeringEngineSetAngle(&SteeringEngine1data,SteeringEngine1data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine2data,SteeringEngine2data.SetAngle.Data.floatdata);
		SteeringEngineSetAngle(&SteeringEngine3data,SteeringEngine3data.SetAngle.Data.floatdata);
		/*等待摆到水平位置*/
		osDelay(2500);

		ChaoShengBodata.CalibrateState=CMD_OPEN;
  }
}
void myTaskLED_Ds2(void const * argument)
{

  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOE, DS2_Pin);
		osDelay(100);
		
  }
}

void myTaskAutoModeLedTwinkle(void const * argument)
{
static	__IO uint8_t RxData=0;

  for(;;)
  {
		if(xQueueReceive( xQueue_AutoModeLedWinkle, ( void * ) &RxData,portMAX_DELAY)==pdPASS){
}
		while(uxQueueSpacesAvailable(xQueue_AutoModeLedWinkle)==200&&RxData>=3){
			HAL_GPIO_TogglePin(AutoModeLed_GPIO_Port, AutoModeLed_Pin);
			osDelay(RxData);
		}
		if(RxData==0){
			HAL_GPIO_WritePin(AutoModeLed_GPIO_Port, AutoModeLed_Pin,GPIO_PIN_SET);
		}else if(RxData==1){
			HAL_GPIO_WritePin(AutoModeLed_GPIO_Port, AutoModeLed_Pin,GPIO_PIN_RESET);
		}


		
  }
}
void myTaskManualModeLedTwinkle(void const * argument)
{
static	__IO uint8_t RxData=0;

  for(;;)
  {
		if(xQueueReceive( xQueue_ManualModeLedWinkle, ( void * ) &RxData,portMAX_DELAY)==pdPASS){
}
		while(uxQueueSpacesAvailable(xQueue_ManualModeLedWinkle)==200&&RxData>=3){
			HAL_GPIO_TogglePin(ManualModeLed_GPIO_Port, ManualModeLed_Pin);
			osDelay(RxData);
		}
		if(RxData==0){
			HAL_GPIO_WritePin(ManualModeLed_GPIO_Port, ManualModeLed_Pin,GPIO_PIN_SET);
		}else if(RxData==1){
			HAL_GPIO_WritePin(ManualModeLed_GPIO_Port, ManualModeLed_Pin,GPIO_PIN_RESET);
		}

		
  }
}
void myTaskReceiveFromVB(void const * argument)
{

__IO	char CMD_Data[4];
__IO   uint16_t writeAdd;
__IO	uint8_t writeData[4];
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);  
		if(ReceiveData==1){
			ReceiveData=0;
			writeAdd=(rxData[1]<<8)|rxData[2];	
			writeData[0]=rxData[3];
			writeData[1]=rxData[4];
			writeData[2]=rxData[5];
			writeData[3]=rxData[6];
			if(writeAdd<4000){
				AT24CXX_Write(writeAdd,(uint8_t*)writeData,4);
				FloatData_GetFormEEPROM_ONE(writeAdd);	
			}
			else{
				FloatData_GetFormEEPROM_TWO(writeAdd,writeData);
			}
			HAL_UART_Receive_IT(&huart1, rxData, 7);				
		}
		if(ReceiveCMD==1){
			ReceiveCMD=0;
			writeAdd=(rxData[1]<<8)|rxData[2];
			CMD_Data[0]=rxData[3];
			CMD_Data[1]=rxData[4];
			CMD_Data[2]=rxData[5];
			CMD_Data[3]=rxData[6];
			switch (writeAdd)
				{
				case CMD_SteeringEngine1Add_Add  : 
					CMD_SteeringEngine1Add.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );	
					break;
				case CMD_SteeringEngine1Sub_Add: 
					CMD_SteeringEngine1Sub.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );	
					break; 
				case CMD_SteeringEngine2Add_Add: 
					CMD_SteeringEngine2Add.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_SteeringEngine2Sub_Add: 
					CMD_SteeringEngine2Sub.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_SteeringEngine3Add_Add: 
					CMD_SteeringEngine3Add.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_SteeringEngine3Sub_Add: 
					CMD_SteeringEngine3Sub.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_SteeringEngine4Add_Add: 
					CMD_SteeringEngine4Add.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_SteeringEngine4Sub_Add: 
					CMD_SteeringEngine4Sub.Data=CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_TrackMotRightForward_Add:
					CMD_TrackMotRightForward.Data=CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_TrackMotRightBack_Add: 
					CMD_TrackMotRightBack.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_TrackMotRightStop_Add: 
					CMD_TrackMotRightStop.Data=CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_TrackMotLeftForward_Add:
					CMD_TrackMotLeftForward.Data=CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_TrackMotLeftBack_Add:
					CMD_TrackMotLeftBack.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_TrackMotLeftStop_Add:
					CMD_TrackMotLeftStop.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );				
					break;
				case CMD_StepMotUpContinued_Add:
					CMD_StepMotUpContinued.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_StepMotDownContinued_Add:
					CMD_StepMotDownContinued.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_StepMotStopContinued_Add:
					CMD_StepMotStopContinued.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_StepMotUpOneStep_Add:
					CMD_StepMotUpOneStep.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_StepMotDownOneStep_Add:
					CMD_StepMotDownOneStep.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_PushRodFrontUp_Add:
					CMD_PushRodFrontUp.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_PushRodFrontDown_Add:
					CMD_PushRodFrontDown.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_PushRodFrontStop_Add:
					CMD_PushRodFrontStop.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_PushRodBehindUp_Add:
					CMD_PushRodBehindUp.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_PushRodBehindDown_Add:
					CMD_PushRodBehindDown.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				case CMD_PushRodBehindStop_Add:
					CMD_PushRodBehindStop.Data =CMD_Data[0];
					xTaskNotifyGive( myTaskCMDHandle );
					break;
				default:
					break;
				}
			HAL_UART_Receive_IT(&huart1, rxData, 7);					
		}
	}
}
	void myTaskSendToVB(void const * argument)
{
		UART1Message_Typedef UART1_txMessage_txQueue;
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY); 
		UART1_txMessage_txQueue.ID=JY61_1.ChairBalanceAngleMax.Add;
		UART1_txMessage_txQueue.Message=JY61_1.ChairBalanceAngleMax.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		UART1_txMessage_txQueue.ID=JY61_1.ChairBalanceAngleMin.Add;
		UART1_txMessage_txQueue.Message=JY61_1.ChairBalanceAngleMin.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=JY61_2.StepMotVibrationAngle1.Add;
		UART1_txMessage_txQueue.Message=JY61_2.StepMotVibrationAngle1.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=JY61_2.StepMotVibrationAngle2.Add;
		UART1_txMessage_txQueue.Message=JY61_2.StepMotVibrationAngle2.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData[0].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData[0].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData[1].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData[1].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}			
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData[2].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData[2].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData[3].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData[3].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData1[0].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData1[0].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData1[1].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData1[1].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}			
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData1[2].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData1[2].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData1[3].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData1[3].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData2[0].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData2[0].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData2[1].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData2[1].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}			
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData2[2].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData2[2].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}
		UART1_txMessage_txQueue.ID=ChaoShengBodata.SetData2[3].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.SetData2[3].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		

			
		UART1_txMessage_txQueue.ID=TrackMotRightData.SetSpeed.Add;
		UART1_txMessage_txQueue.Message=TrackMotRightData.SetSpeed.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=TrackMotRightData.SetSpeed1.Add;
		UART1_txMessage_txQueue.Message=TrackMotRightData.SetSpeed1.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
			
			
		UART1_txMessage_txQueue.ID=TrackMotLeftData.SetSpeed.Add;
		UART1_txMessage_txQueue.Message=TrackMotLeftData.SetSpeed.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	

		UART1_txMessage_txQueue.ID=TrackMotLeftData.SetSpeed1.Add;
		UART1_txMessage_txQueue.Message=TrackMotLeftData.SetSpeed1.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
			
		UART1_txMessage_txQueue.ID=StepMotdata.CurrentAngle.Add;
		UART1_txMessage_txQueue.Message=StepMotdata.CurrentAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
			
		UART1_txMessage_txQueue.ID=StepMotdata.Speed.Add;
		UART1_txMessage_txQueue.Message=StepMotdata.Speed.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				

		UART1_txMessage_txQueue.ID=SteeringEngine1data.SetAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine1data.SetAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		UART1_txMessage_txQueue.ID=SteeringEngine2data.SetAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine2data.SetAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=SteeringEngine3data.SetAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine3data.SetAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=SteeringEngine4data.SetAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine4data.SetAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
			
		UART1_txMessage_txQueue.ID=SteeringEngine1data.SetAngle1.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine1data.SetAngle1.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		UART1_txMessage_txQueue.ID=SteeringEngine2data.SetAngle1.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine2data.SetAngle1.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=SteeringEngine3data.SetAngle1.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine3data.SetAngle1.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=SteeringEngine4data.SetAngle1.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine4data.SetAngle1.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
			
		UART1_txMessage_txQueue.ID=SteeringEngine1data.SetAngle2.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine1data.SetAngle2.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		UART1_txMessage_txQueue.ID=SteeringEngine2data.SetAngle2.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine2data.SetAngle2.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=SteeringEngine3data.SetAngle2.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine3data.SetAngle2.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
		UART1_txMessage_txQueue.ID=SteeringEngine4data.SetAngle2.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine4data.SetAngle2.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		
		UART1_txMessage_txQueue.ID=JY61_1.Roll_Pitch_Yaw_Data.Roll_X.Add;			
		UART1_txMessage_txQueue.Message=JY61_1.Roll_Pitch_Yaw_Data.Roll_X.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		UART1_txMessage_txQueue.ID=JY61_1.Roll_Pitch_Yaw_Data.Pitch_Y.Add;
		UART1_txMessage_txQueue.Message=JY61_1.Roll_Pitch_Yaw_Data.Pitch_Y.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		UART1_txMessage_txQueue.ID=JY61_1.Roll_Pitch_Yaw_Data.Yaw_Z.Add;
		UART1_txMessage_txQueue.Message=JY61_1.Roll_Pitch_Yaw_Data.Yaw_Z.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=JY61_1.Roll_Pitch_Yaw_Data.T.Add;
		UART1_txMessage_txQueue.Message=JY61_1.Roll_Pitch_Yaw_Data.T.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
			
		UART1_txMessage_txQueue.ID=JY61_2.Roll_Pitch_Yaw_Data.Roll_X.Add;			
		UART1_txMessage_txQueue.Message=JY61_2.Roll_Pitch_Yaw_Data.Roll_X.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		UART1_txMessage_txQueue.ID=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Add;
		UART1_txMessage_txQueue.Message=JY61_2.Roll_Pitch_Yaw_Data.Pitch_Y.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}				
		UART1_txMessage_txQueue.ID=JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Add;
		UART1_txMessage_txQueue.Message=JY61_2.Roll_Pitch_Yaw_Data.Yaw_Z.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=JY61_2.Roll_Pitch_Yaw_Data.T.Add;
		UART1_txMessage_txQueue.Message=JY61_2.Roll_Pitch_Yaw_Data.T.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}		
			
		UART1_txMessage_txQueue.ID=ChaoShengBodata.floatData[0].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.floatData[0].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=ChaoShengBodata.floatData[1].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.floatData[1].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=ChaoShengBodata.floatData[2].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.floatData[2].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=ChaoShengBodata.floatData[3].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.floatData[3].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	

		UART1_txMessage_txQueue.ID=TrackMotRightData.CurrentSpeed.Add;
		UART1_txMessage_txQueue.Message=TrackMotRightData.CurrentSpeed.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=TrackMotLeftData.CurrentSpeed.Add;
		UART1_txMessage_txQueue.Message=TrackMotLeftData.CurrentSpeed.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=SteeringEngine1data.CurrentAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine1data.CurrentAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=SteeringEngine2data.CurrentAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine2data.CurrentAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=SteeringEngine3data.CurrentAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine3data.CurrentAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=SteeringEngine4data.CurrentAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine4data.CurrentAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=SteeringEngine1data.OffsetAngle.Add;
		UART1_txMessage_txQueue.Message=SteeringEngine1data.OffsetAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=StepMotdata.OffsetAngle.Add;
		UART1_txMessage_txQueue.Message=StepMotdata.OffsetAngle.Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	

		UART1_txMessage_txQueue.ID=ChaoShengBodata.floatData[4].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.floatData[4].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	
		UART1_txMessage_txQueue.ID=ChaoShengBodata.floatData[5].Add;
		UART1_txMessage_txQueue.Message=ChaoShengBodata.floatData[5].Data.chardata;
		if(xQueueSend( xQueue_UART1, ( void * ) &UART1_txMessage_txQueue,(TickType_t)0 )==pdPASS){}	

	}
  /* USER CODE END StartDefaultTask */
}
 void myTaskUart1(void const * argument)
{
  /* USER CODE BEGIN myTaskUSB2_Uart1 */
	UART1Message_Typedef UART1_txMessage_rxQueue;
	uint8_t data[7];
  /* Infinite loop */
  for(;;)
  {
		if(xQueueReceive( xQueue_UART1, ( void * ) &UART1_txMessage_rxQueue,portMAX_DELAY)==pdPASS){
			data[0]='D';
			data[1]=UART1_txMessage_rxQueue.ID>>8;
			data[2]=UART1_txMessage_rxQueue.ID;
			data[3]=UART1_txMessage_rxQueue.Message[0];
			data[4]=UART1_txMessage_rxQueue.Message[1];
			data[5]=UART1_txMessage_rxQueue.Message[2];
			data[6]=UART1_txMessage_rxQueue.Message[3];
			HAL_UART_Transmit(&huart1,data,7,1);	
			osDelay(4);
			if(uxQueueSpacesAvailable(xQueue_UART1)==200){
				osDelay(100);	
				xTaskNotifyGive( myTaskSendToVBHandle );
			
			}			
		}
		
  }
  /* USER CODE END myTaskUSB2_Uart1 */
}  

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
