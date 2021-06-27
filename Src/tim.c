/**
  ******************************************************************************
  * File Name          : TIM.c
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "math.h"
#include "24cxx.h"
uint32_t a,b,c;
/* USER CODE BEGIN 0 */
speedRampData srd1               = {STOP,CW,0,0,0,0,0};        // �Ӽ������߱���

__IO int32_t  step_position     = 0;           //�ֶ����Ƶ�ǰλ��
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
__IO uint16_t Toggle_Pulse[1]={10};         // �Ƚ�������ڣ�ֵԽС���Ƶ��Խ��
__IO uint32_t pulse_count[6]; /*  ���������һ�����������������2 */
SteeringEngineData SteeringEngine1data,SteeringEngine2data,SteeringEngine3data,SteeringEngine4data;
TrackMotData TrackMotRightData,TrackMotLeftData;
StepMotData StepMotdata;
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

TIM_HandleTypeDef htim5;

/* TIM5 init function */
void MX_TIM5_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 420-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_TIM_Base_Start_IT(&htim5);
}
/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = Toggle_Pulse[0];
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
}


void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* tim_ocHandle)
{

  if(tim_ocHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    /**TIM1 GPIO Configuration    
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4 
    */
    GPIO_InitStruct.Pin = TrackMotLeft_Pin|TrackMotRight_Pin|HubMotLeft_Pin|HubMotRight_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    /**TIM3 GPIO Configuration    
    PC8     ------> TIM3_CH3 
    */
    GPIO_InitStruct.Pin = StepMotPulse_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(StepMotPulse_0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
  if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM5_IRQn);

  }
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
}

void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* tim_ocHandle)
{

  if(tim_ocHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
} 

void TrackMotLeftForward(uint32_t speed){
	if(speed>999){
		speed=999;
	}
	TrackMotLeftData.CurrentSpeed.Data.floatdata=speed;
	HAL_GPIO_WritePin(HubMotLeftDirBack_GPIO_Port, HubMotLeftDirBack_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);
}
void TrackMotLeftBack(uint32_t speed){
	if(speed>999){
		speed=999;
	}
	TrackMotLeftData.CurrentSpeed.Data.floatdata=speed;
	HAL_GPIO_WritePin(HubMotLeftDirBack_GPIO_Port, HubMotLeftDirBack_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);
}
void TrackMotLeftStop(void){
	TrackMotLeftData.CurrentSpeed.Data.floatdata=0;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
}
void TrackMotRightForward(uint32_t speed){
	if(speed>999){
		speed=999;
	}
	TrackMotRightData.CurrentSpeed.Data.floatdata=speed;
	HAL_GPIO_WritePin(HubMotRightDirBack_GPIO_Port, HubMotRightDirBack_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);
}
void TrackMotRightBack(uint32_t speed){
	if(speed>999){
		speed=999;
	}
	TrackMotRightData.CurrentSpeed.Data.floatdata=speed;
	HAL_GPIO_WritePin(HubMotRightDirBack_GPIO_Port, HubMotRightDirBack_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);
}
void TrackMotRightStop(void){
	TrackMotRightData.CurrentSpeed.Data.floatdata=0;
	HAL_GPIO_WritePin(HubMotRightDirBack_GPIO_Port, HubMotRightDirBack_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
}


/* USER CODE BEGIN 1 */
void STEPMOTOR_AxisMoveRel_lin1(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{  
  __IO uint16_t tim_count;
  // ´ïµ½×î´óËÙ¶ÈÊ±µÄ²½Êý
  __IO uint32_t max_s_lim;
  // ±ØÐëÒª¿ªÊ¼¼õËÙµÄ²½Êý£¨Èç¹û¼ÓËÙÃ»ÓÐ´ïµ½×î´óËÙ¶È£©
  __IO uint32_t accel_lim;

  if(step < 0) // ²½ÊýÎª¸ºÊý
  {
    srd1.dir = CCW; // ÄæÊ±Õë·½ÏòÐý×ª
		MOTOR_DIR1_FORWARD();
    step =-step;   // »ñÈ¡²½Êý¾ø¶ÔÖµ
  }
  else
  {
    srd1.dir = CW; // Ë³Ê±Õë·½ÏòÐý×ª
		MOTOR_DIR1_REVERSAL();

  }
  
  if(step == 1)    // ²½ÊýÎª1
  {
    srd1.accel_count = -1;   // Ö»ÒÆ¶¯Ò»²½
    srd1.run_state = DECEL;  // ¼õËÙ×´Ì¬.
    srd1.step_delay = 1000;	// ¶ÌÑÓÊ±	
  }
  else if(step != 0)  // Èç¹ûÄ¿±êÔË¶¯²½Êý²»Îª0
  {
    // ÎÒÃÇµÄÇý¶¯Æ÷ÓÃ»§ÊÖ²áÓÐÏêÏ¸µÄ¼ÆËã¼°ÍÆµ¼¹ý³Ì

    // ÉèÖÃ×î´óËÙ¶È¼«ÏÞ, ¼ÆËãµÃµ½min_delayÓÃÓÚ¶¨Ê±Æ÷µÄ¼ÆÊýÆ÷µÄÖµ¡£
    // min_delay = (alpha / tt)/ w
    srd1.min_delay = (int32_t)(A_T_x10/speed);

    // Í¨¹ý¼ÆËãµÚÒ»¸ö(c0) µÄ²½½øÑÓÊ±À´Éè¶¨¼ÓËÙ¶È£¬ÆäÖÐaccelµ¥Î»Îª0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    srd1.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);

    // ¼ÆËã¶àÉÙ²½Ö®ºó´ïµ½×î´óËÙ¶ÈµÄÏÞÖÆ
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
    // Èç¹û´ïµ½×î´óËÙ¶ÈÐ¡ÓÚ0.5²½£¬ÎÒÃÇ½«ËÄÉáÎåÈëÎª0
    // µ«Êµ¼ÊÎÒÃÇ±ØÐëÒÆ¶¯ÖÁÉÙÒ»²½²ÅÄÜ´ïµ½ÏëÒªµÄËÙ¶È
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // ¼ÆËã¶àÉÙ²½Ö®ºóÎÒÃÇ±ØÐë¿ªÊ¼¼õËÙ
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = (uint32_t)(step*decel/(accel+decel));
    // ÎÒÃÇ±ØÐë¼ÓËÙÖÁÉÙ1²½²ÅÄÜ²ÅÄÜ¿ªÊ¼¼õËÙ.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Ê¹ÓÃÏÞÖÆÌõ¼þÎÒÃÇ¿ÉÒÔ¼ÆËã³ö¼õËÙ½×¶Î²½Êý
    if(accel_lim <= max_s_lim){
      srd1.decel_val = accel_lim - step;
    }
    else{
      srd1.decel_val = -(max_s_lim*accel/decel);
    }
    // µ±Ö»Ê£ÏÂÒ»²½ÎÒÃÇ±ØÐë¼õËÙ
    if(srd1.decel_val == 0){
      srd1.decel_val = -1;
    }

    // ¼ÆËã¿ªÊ¼¼õËÙÊ±µÄ²½Êý
    srd1.decel_start = step + srd1.decel_val;

    // Èç¹û×î´óËÙ¶ÈºÜÂý£¬ÎÒÃÇ¾Í²»ÐèÒª½øÐÐ¼ÓËÙÔË¶¯
    if(srd1.step_delay <= srd1.min_delay){
      srd1.step_delay = srd1.min_delay;
      srd1.run_state = RUN;
    }
    else{
      srd1.run_state = ACCEL;
    }    
    // ¸´Î»¼ÓËÙ¶È¼ÆÊýÖµ
    srd1.accel_count = 0;
  }
		StepMotdata.State=MOTOSTATE_BUSY;
		tim_count=__HAL_TIM_GET_COUNTER(&htim3);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,tim_count+srd1.step_delay); // ÉèÖÃ¶¨Ê±Æ÷±È½ÏÖµ
		HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_3);// Ê¹ÄÜ¶¨Ê±Æ÷Í¨µÀ 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{  
	if(htim==&htim3){
	if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
		{
	if(StepMotdata.State==MOTOSTATE_BUSY){		
		__IO uint16_t tim_count1=0;
  // ±£´æÐÂ£¨ÏÂ£©Ò»¸öÑÓÊ±ÖÜÆÚ
 __IO  uint16_t static new_step_delay1=0;
  // ¼ÓËÙ¹ý³ÌÖÐ×îºóÒ»´ÎÑÓÊ±£¨Âö³åÖÜÆÚ£©.
  __IO static uint16_t last_accel_delay1=0;
  // ×ÜÒÆ¶¯²½Êý¼ÆÊýÆ÷
  __IO static uint32_t step_count1 = 0;
  // ¼ÇÂ¼new_step_delayÖÐµÄÓàÊý£¬Ìá¸ßÏÂÒ»²½¼ÆËãµÄ¾«¶È
  __IO static int32_t rest1 = 0;
  //¶¨Ê±Æ÷Ê¹ÓÃ·­×ªÄ£Ê½£¬ÐèÒª½øÈëÁ½´ÎÖÐ¶Ï²ÅÊä³öÒ»¸öÍêÕûÂö³å
  __IO static uint8_t i1=0;
    // ÉèÖÃ±È½ÏÖµ
			tim_count1=__HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,tim_count1+srd1.step_delay);

   i1++;     // ¶¨Ê±Æ÷ÖÐ¶Ï´ÎÊý¼ÆÊýÖµ
		/*脉冲输出*/
			HAL_GPIO_TogglePin(StepMotPulse_GPIO_Port,StepMotPulse_Pin);			
    if(i1==2) // 2´Î£¬ËµÃ÷ÒÑ¾­Êä³öÒ»¸öÍêÕûÂö³å
    {
      i1=0;   // ÇåÁã¶¨Ê±Æ÷ÖÐ¶Ï´ÎÊý¼ÆÊýÖµ
      switch(srd1.run_state) // ¼Ó¼õËÙÇúÏß½×¶Î
      {
        case STOP:
          step_count1 = 0;  // ÇåÁã²½Êý¼ÆÊýÆ÷
          rest1 = 0;        // ÇåÁãÓàÖµ
          // ¹Ø±ÕÍ¨µÀ
					StepMotdata.State=MOTOSTATE_FREE;
          TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_3, TIM_CCx_DISABLE);        
          __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC3);				
          break;
        case ACCEL:
          step_count1++;      // ²½Êý¼Ó1
          if(srd1.dir==CW)
          {	  	
            step_position++; // ¾ø¶ÔÎ»ÖÃ¼Ó1
          }
          else
          {
            step_position--; // ¾ø¶ÔÎ»ÖÃ¼õ1
          }
          srd1.accel_count++; // ¼ÓËÙ¼ÆÊýÖµ¼Ó1
          new_step_delay1 = srd1.step_delay - (((2 *srd1.step_delay) + rest1)/(4 * srd1.accel_count + 1));//¼ÆËãÐÂ(ÏÂ)Ò»²½Âö³åÖÜÆÚ(Ê±¼ä¼ä¸ô)
          rest1 = ((2 * srd1.step_delay)+rest1)%(4 * srd1.accel_count + 1);// ¼ÆËãÓàÊý£¬ÏÂ´Î¼ÆËã²¹ÉÏÓàÊý£¬¼õÉÙÎó²î
          if(step_count1 >= srd1.decel_start)// ¼ì²éÊÇ¹»Ó¦¸Ã¿ªÊ¼¼õËÙ
          {
            srd1.accel_count = srd1.decel_val; // ¼ÓËÙ¼ÆÊýÖµÎª¼õËÙ½×¶Î¼ÆÊýÖµµÄ³õÊ¼Öµ
            srd1.run_state = DECEL;           // ÏÂ¸öÂö³å½øÈë¼õËÙ½×¶Î
          }
          else if(new_step_delay1 <= srd1.min_delay) // ¼ì²éÊÇ·ñµ½´ïÆÚÍûµÄ×î´óËÙ¶È
          {
            last_accel_delay1 = new_step_delay1; // ±£´æ¼ÓËÙ¹ý³ÌÖÐ×îºóÒ»´ÎÑÓÊ±£¨Âö³åÖÜÆÚ£©
            new_step_delay1 = srd1.min_delay;    // Ê¹ÓÃmin_delay£¨¶ÔÓ¦×î´óËÙ¶Èspeed£©
            rest1 = 0;                          // ÇåÁãÓàÖµ
            srd1.run_state = RUN;               // ÉèÖÃÎªÔÈËÙÔËÐÐ×´Ì¬
          }
          break;

        case RUN:
          step_count1++;  // ²½Êý¼Ó1
          if(srd1.dir==CW)
          {	  	
            step_position++; // ¾ø¶ÔÎ»ÖÃ¼Ó1
          }
          else
          {
            step_position--; // ¾ø¶ÔÎ»ÖÃ¼õ1
          }
          new_step_delay1 = srd1.min_delay;     // Ê¹ÓÃmin_delay£¨¶ÔÓ¦×î´óËÙ¶Èspeed£©
          if(step_count1 >= srd1.decel_start)   // ÐèÒª¿ªÊ¼¼õËÙ
          {
            srd1.accel_count = srd1.decel_val;  // ¼õËÙ²½Êý×öÎª¼ÓËÙ¼ÆÊýÖµ
            new_step_delay1 = last_accel_delay1;// ¼Ó½×¶Î×îºóµÄÑÓÊ±×öÎª¼õËÙ½×¶ÎµÄÆðÊ¼ÑÓÊ±(Âö³åÖÜÆÚ)
            srd1.run_state = DECEL;            // ×´Ì¬¸Ä±äÎª¼õËÙ
          }
          break;

        case DECEL:
					if(StepMotdata.ContinueMode==MOTOSTATE_FREE){
          step_count1++;  // ²½Êý¼Ó1
          if(srd1.dir==CW)
          {	  	
            step_position++; // ¾ø¶ÔÎ»ÖÃ¼Ó1
          }
          else
          {
            step_position--; // ¾ø¶ÔÎ»ÖÃ¼õ1
          }
          srd1.accel_count++;
          new_step_delay1 = srd1.step_delay - (((2 * srd1.step_delay) + rest1)/(4 * srd1.accel_count + 1)); //¼ÆËãÐÂ(ÏÂ)Ò»²½Âö³åÖÜÆÚ(Ê±¼ä¼ä¸ô)
          rest1 = ((2 * srd1.step_delay)+rest1)%(4 * srd1.accel_count + 1);// ¼ÆËãÓàÊý£¬ÏÂ´Î¼ÆËã²¹ÉÏÓàÊý£¬¼õÉÙÎó²î
          
          //¼ì²éÊÇ·ñÎª×îºóÒ»²½
          if(srd1.accel_count >= 0)
          {
            srd1.run_state = STOP;
          }
				}
          break;
      } 
				srd1.step_delay = new_step_delay1; // ÎªÏÂ¸ö(ÐÂµÄ)ÑÓÊ±(Âö³åÖÜÆÚ)¸³Öµ
    }
  }
}
}
	}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	__IO static uint32_t count,num1,num2,num3,num4,Pulse;

	if(htim==&htim5){

		if(count==SteeringEngine4data.PulseWide){
				HAL_GPIO_WritePin(SteeringEngine4_GPIO_Port, SteeringEngine4_Pin, GPIO_PIN_RESET);
		}
		if(count==SteeringEngine3data.PulseWide){
				HAL_GPIO_WritePin(SteeringEngine3_GPIO_Port, SteeringEngine3_Pin, GPIO_PIN_RESET);
		}
		if(count==SteeringEngine1data.PulseWide){
				HAL_GPIO_WritePin(SteeringEngine1_GPIO_Port, SteeringEngine1_Pin, GPIO_PIN_RESET);
		}
		if(count==SteeringEngine2data.PulseWide){
				HAL_GPIO_WritePin(SteeringEngine2_GPIO_Port, SteeringEngine2_Pin, GPIO_PIN_RESET);
		}
		if(count++==2000){

		if(SteeringEngine1data.PulseWide!=0){
			num1++;
			if(num1==75){
				num1=0;
				SteeringEngine1data.PulseWide=0;
			}
		}
		if(SteeringEngine2data.PulseWide!=0){
			num2++;
			if(num2==75){
				num2=0;
				SteeringEngine2data.PulseWide=0;
			}
		}
		if(SteeringEngine3data.PulseWide!=0){
			num3++;
			if(num3==75){
				num3=0;
				SteeringEngine3data.PulseWide=0;
			}
		}
		if(SteeringEngine4data.PulseWide!=0){
			num4++;
			if(num4==75){
				num4=0;
				SteeringEngine4data.PulseWide=0;
			}
		}
			count=0;
			HAL_GPIO_WritePin(SteeringEngine1_GPIO_Port, SteeringEngine1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SteeringEngine2_GPIO_Port, SteeringEngine2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SteeringEngine3_GPIO_Port, SteeringEngine3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SteeringEngine4_GPIO_Port, SteeringEngine4_Pin, GPIO_PIN_SET);
		}


}

	if(StepMotdata.WorkEnable==MOTOSTATE_BUSY){
		StepMotdata.State=MOTOSTATE_BUSY;
		Pulse++;
		if(Pulse==StepMotdata.Speed.Data.floatdata){
			Pulse=0;
		if(	HAL_GPIO_ReadPin(StepMotDir_GPIO_Port, StepMotDir_Pin)==GPIO_PIN_SET){
			StepMotdata.CurrentPulse2++;
		}else{
			StepMotdata.CurrentPulse2--;
		}
			HAL_GPIO_TogglePin(StepMotPulse_GPIO_Port,StepMotPulse_Pin);
			if(	StepMotdata.ContinueMode!=MOTOSTATE_BUSY){
			StepMotdata.CurrentPulse--;
			if(StepMotdata.CurrentPulse==0){
				StepMotdata.WorkEnable=MOTOSTATE_FREE;
			}
			}
		}
	}else{
			Pulse=0;
			StepMotdata.State=MOTOSTATE_FREE;
		}
	
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	}
void STEPMOTOR_MoveHoriizontal(){
	if(StepMotdata.CurrentPulse2>0){
		MOTOR_DIR1_FORWARD();
		StepMotdata.CurrentPulse=StepMotdata.CurrentPulse2;
	}else if(StepMotdata.CurrentPulse2<0){
		MOTOR_DIR1_REVERSAL();
		StepMotdata.CurrentPulse=-StepMotdata.CurrentPulse2;
	}else if(StepMotdata.CurrentPulse2==0){
		StepMotdata.WorkEnable=MOTOSTATE_FREE;
		return;
	}
		StepMotdata.State=MOTOSTATE_BUSY;
		StepMotdata.WorkEnable=MOTOSTATE_BUSY;
		StepMotdata.ContinueMode=MOTOSTATE_FREE;
		osDelay(2);

}
void STEPMOTOR_MoveContinuted(int dir){
	if(dir>0){
		MOTOR_DIR1_REVERSAL();
	}else if(dir<0){
		MOTOR_DIR1_FORWARD();
	}else if(dir==0){
		StepMotdata.WorkEnable=MOTOSTATE_FREE;
		return;
	}
		StepMotdata.State=MOTOSTATE_BUSY;
		StepMotdata.WorkEnable=MOTOSTATE_BUSY;
		StepMotdata.ContinueMode=MOTOSTATE_BUSY;
		osDelay(2);

}
void STEPMOTOR_MoveRel(int rel){
	uint32_t a;
	if(rel>0){
		a=rel*1024000/360;
		MOTOR_DIR1_REVERSAL();
	}else if(rel<0){
		a=-rel*1024000/360;
		MOTOR_DIR1_FORWARD();
	}else if(a==0){
		StepMotdata.WorkEnable=MOTOSTATE_FREE;
		return;
	}

		if(StepMotdata.State!=MOTOSTATE_BUSY){
			
			StepMotdata.CurrentPulse=a*2;
			StepMotdata.WorkEnable=MOTOSTATE_BUSY;
			StepMotdata.State=MOTOSTATE_BUSY;
			
			StepMotdata.ContinueMode=MOTOSTATE_FREE;
			
			StepMotdata.CurrentAngle.Data.floatdata+=rel;
//			AT24CXX_Write(StepMotdata.CurrentAngle.Add,(uint8_t*)StepMotdata.CurrentAngle.Data.chardata,4);
		osDelay(2);	
	}

}
void STEPMOTOR_STOP(void){
	StepMotdata.ContinueMode=MOTOSTATE_FREE;
	StepMotdata.WorkEnable=MOTOSTATE_FREE;
	StepMotdata.State=MOTOSTATE_FREE;
	osDelay(2);
}
void SteeringEngineSetAngle(SteeringEngineData * SteeringEnginedata,float angle){
	if(angle!=0){
		if(angle<0){
			angle=0;
		}else if(angle>180){
			angle=180;
		}
		
		SteeringEnginedata->CurrentAngle.Data.floatdata=angle;
		
		SteeringEnginedata->PulseWide=angle*200/180+50;
	}
	else{
		SteeringEnginedata->PulseWide=0;
}
}
void   HubMotLeft_Move(int speed ){
	if(speed>0){
		TrackMotLeftForward(speed);
	}
	else if(speed<0){
		TrackMotLeftBack(-speed);
	}
	else{
		TrackMotLeftStop();
	}
} 
void   HubMotRight_Move(int speed ){
	if(speed>0){
		TrackMotRightForward(speed);
	}
	else if(speed<0){
		TrackMotRightBack(-speed);
	}
	else{
		TrackMotRightStop();
	}
} 


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
