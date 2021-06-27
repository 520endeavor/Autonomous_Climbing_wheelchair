/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Datatypedef.h"	 
#define TIM3_PRESCALER  42000000.0f
/* USER CODE BEGIN Includes */

#define MOTOSTATE_FREE              10
#define MOTOSTATE_BUSY              20
/*轮边电机 动作函数*/
#define   HubMotLeftForward(uint32_t )  TrackMotLeftForward(uint32_t)
#define   HubMotLeftBack(uint32_t )  TrackMotLeftBack(uint32_t)
#define   HubMotLeftStop(uint32_t )  TrackMotLeftStop(uint32_t)
#define   HubMotRightForward(uint32_t )  TrackMotRightForward(uint32_t)
#define   HubMotRightBack(uint32_t )  TrackMotRightBack(uint32_t)
#define   HubMotRightStop(uint32_t )  TrackMotRightStop(uint32_t)

typedef struct{
	volatile  float  Angle;
	volatile uint32_t PulseWide;
	FloatData SetAngle;
	FloatData SetAngle1;
	FloatData SetAngle2;
	FloatData CurrentAngle;
	FloatData OffsetAngle;
}SteeringEngineData;	 
typedef struct{
	FloatData SetSpeed;
	FloatData SetSpeed1;
	FloatData CurrentSpeed;
}TrackMotData;	 
typedef struct{
	FloatData CurrentAngle;
	FloatData Speed;
	FloatData OffsetAngle;
	volatile 	uint8_t State;
	volatile uint8_t ContinueMode;
	int CurrentPulse;
	uint8_t WorkEnable;
	int CurrentPulse2;
}StepMotData;
typedef struct {
  __IO uint8_t  run_state ;  // µç»úÐý×ª×´Ì¬
  __IO uint8_t  dir ;        // µç»úÐý×ª·½Ïò
  __IO int32_t  step_delay;  // ÏÂ¸öÂö³åÖÜÆÚ£¨Ê±¼ä¼ä¸ô£©£¬Æô¶¯Ê±Îª¼ÓËÙ¶È
  __IO uint32_t decel_start; // Æô¶¯¼õËÙÎ»ÖÃ
  __IO int32_t  decel_val;   // ¼õËÙ½×¶Î²½Êý
  __IO int32_t  min_delay;   // ×îÐ¡Âö³åÖÜÆÚ(×î´óËÙ¶È£¬¼´ÔÈËÙ¶ÎËÙ¶È)
  __IO int32_t  accel_count; // ¼Ó¼õËÙ½×¶Î¼ÆÊýÖµ
}speedRampData;

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    0 // Ë³Ê±Õë
#define CCW                                   1 // ÄæÊ±Õë

#define STOP                                  0 // ¼Ó¼õËÙÇúÏß×´Ì¬£ºÍ£Ö¹
#define ACCEL                                 1 // ¼Ó¼õËÙÇúÏß×´Ì¬£º¼ÓËÙ½×¶Î
#define DECEL                                 2 // ¼Ó¼õËÙÇúÏß×´Ì¬£º¼õËÙ½×¶Î
#define RUN                                   3 // ¼Ó¼õËÙÇúÏß×´Ì¬£ºÔÈËÙ½×¶Î
#define OTHER																	4
#define T1_FREQ                               TIM3_PRESCALER // ÆµÂÊftÖµ
#define FSPR                                  200         //²½½øµç»úµ¥È¦²½Êý
#define MICRO_STEP                            64          // ²½½øµç»úÇý¶¯Æ÷Ï¸·ÖÊý
#define SPR                                   12800   // Ðý×ªÒ»È¦ÐèÒªµÄÂö³åÊý

// ÊýÑ§³£Êý
#define ALPHA                                 ((float)(2*3.1415926f/SPR))       // ¦Á= 2*pi/spr
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10)) // 0.676ÎªÎó²îÐÞÕýÖµ
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void Error_Handler(void);
extern SteeringEngineData SteeringEngine1data,SteeringEngine2data,SteeringEngine3data,SteeringEngine4data;
extern TrackMotData TrackMotRightData,TrackMotLeftData;
extern StepMotData StepMotdata;
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM8_Init(void);
void MX_TIM5_Init(void);              
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void STEPMOTOR_AxisMoveRel_lin1(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed);
void STEPMOTOR_STOP(void); 
void STEPMOTOR_MoveRel(int rel);
void STEPMOTOR_MoveContinuted(int dir);
void STEPMOTOR_MoveHoriizontal();

void SteeringEngineSetAngle(SteeringEngineData * SteeringEnginedata,float angle);
void TrackMotLeftForward(uint32_t speed);
void TrackMotLeftBack(uint32_t speed);
void TrackMotLeftStop(void);
void TrackMotRightForward(uint32_t speed);
void TrackMotRightBack(uint32_t speed);
void TrackMotRightStop(void);

void   HubMotRight_Move(int speed );
void   HubMotLeft_Move(int speed );
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
