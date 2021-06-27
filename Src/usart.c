/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"
#include "cmsis_os.h"
#include "DataStorage.h"

uint8_t ReceiveData,ReceiveCMD;
uint8_t rxData[7];
extern osThreadId myTaskReceiveFromVBHandle;
extern osThreadId myTaskChairBalanceJudgeHandle;
extern osThreadId myTaskJY61_2DataHandle;

/* USER CODE BEGIN 0 */
JY61 JY61_1,JY61_2;
/* USER CODE END 0 */


UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
void  CopeSerialData(JY61 * jy61);
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_UART_Receive_IT(&huart1, rxData, 7);
}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_UART_Receive_IT(&huart2, JY61_1.rxData, 1);
}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_UART_Receive_IT(&huart3, JY61_2.rxData, 1);
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = JY61_1_TX_Pin|JY61_1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = JY61_2_TX_Pin|JY61_2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOD, JY61_1_TX_Pin|JY61_1_RX_Pin);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, JY61_2_TX_Pin|JY61_2_RX_Pin);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
		if(huart==&huart1){
		if((char)rxData[0]=='D'){
			ReceiveData=1;
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
				vTaskNotifyGiveFromISR (myTaskReceiveFromVBHandle,&xHigherPriorityTaskWoken);      
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
			
		}
		if((char)rxData[0]=='C'){
			ReceiveCMD=1;
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
				vTaskNotifyGiveFromISR (myTaskReceiveFromVBHandle,&xHigherPriorityTaskWoken);      
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
		}
	}
	if(huart==&huart3){
		CopeSerialData(&JY61_1);
		HAL_UART_Receive_IT(&huart3, JY61_1.rxData, 1);
	}
	if(huart==&huart2){
		CopeSerialData(&JY61_2);
		HAL_UART_Receive_IT(&huart2, JY61_2.rxData, 1);
	}
	
	
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}
/* USER CODE BEGIN 1 */

void CharToLong(char Dest[],char Source[])
{
     *Dest      = Source[3];
     *(Dest+1)  = Source[2];
     *(Dest+2)  = Source[1];
     *(Dest+3)  = Source[0];
}
void  CopeSerialData(JY61 * jy61)
{
__IO static uint16_t a,b,c,d;
    jy61->ucRxBuffer[jy61->ucRxCnt++]=jy61->rxData[0];
    if (jy61->ucRxBuffer[0]!=0x55) //?????,????
    {
        jy61->ucRxCnt=0;
        return;
    }
    if (jy61->ucRxCnt<11) {return;}//????11?,???
    else
    {
        switch(jy61->ucRxBuffer[1])
        {
        case 0x50:
            jy61->stcTime.ucYear       = jy61->ucRxBuffer[2];
            jy61->stcTime.ucMonth     = jy61->ucRxBuffer[3];
            jy61->stcTime.ucDay       = jy61->ucRxBuffer[4];
            jy61->stcTime.ucHour      = jy61->ucRxBuffer[5];
            jy61->stcTime.ucMinute    = jy61->ucRxBuffer[6];
            jy61->stcTime.ucSecond    = jy61->ucRxBuffer[7];
            jy61->stcTime.usMiliSecond=((unsigned short)jy61->ucRxBuffer[9]<<8)|jy61->ucRxBuffer[8];
            break;
        case 0x51:
            jy61->stcAcc.a[0] = ((unsigned short)jy61->ucRxBuffer[3]<<8)|jy61->ucRxBuffer[2];
            jy61->stcAcc.a[1] = ((unsigned short)jy61->ucRxBuffer[5]<<8)|jy61->ucRxBuffer[4];
            jy61->stcAcc.a[2] = ((unsigned short)jy61->ucRxBuffer[7]<<8)|jy61->ucRxBuffer[6];
            jy61->stcAcc.T= ((unsigned short)jy61->ucRxBuffer[9]<<8)|jy61->ucRxBuffer[8];
            break;

        case 0x52:
            jy61->stcGyro.w[0] = ((unsigned short)jy61->ucRxBuffer[3]<<8)|jy61->ucRxBuffer[2];
            jy61->stcGyro.w[1] = ((unsigned short)jy61->ucRxBuffer[5]<<8)|jy61->ucRxBuffer[4];
            jy61->stcGyro.w[2] = ((unsigned short)jy61->ucRxBuffer[7]<<8)|jy61->ucRxBuffer[6];
            jy61->stcGyro.T=((unsigned short)jy61->ucRxBuffer[9]<<8)|jy61->ucRxBuffer[8];
            break;

        case 0x53:
            jy61->stcAngle.Angle[0] = ((unsigned short)jy61->ucRxBuffer[3]<<8)|jy61->ucRxBuffer[2];
            jy61->stcAngle.Angle[1] = ((unsigned short)jy61->ucRxBuffer[5]<<8)|jy61->ucRxBuffer[4];
            jy61->stcAngle.Angle[2] = ((unsigned short)jy61->ucRxBuffer[7]<<8)|jy61->ucRxBuffer[6];
            jy61->stcAngle.T = ((unsigned short)jy61->ucRxBuffer[9]<<8)|jy61->ucRxBuffer[8];
			Roll_Pitch_Yaw_Data_Count(jy61);
			if(jy61==&JY61_1&&a++==5&&b==0){
				a=0;
				b=1;
				JY61_1.ReceiveData_OK=CMD_OPEN;
			}
			if(jy61==&JY61_2&&c++==5&&d==0){
				c=0;
				d=0;
				JY61_2.ReceiveData_OK=CMD_OPEN;
			}
            break;
        case 0x54:
            jy61->stcMag.h[0] = ((unsigned short)jy61->ucRxBuffer[3]<<8)|jy61->ucRxBuffer[2];
            jy61->stcMag.h[1] = ((unsigned short)jy61->ucRxBuffer[5]<<8)|jy61->ucRxBuffer[4];
            jy61->stcMag.h[2] = ((unsigned short)jy61->ucRxBuffer[7]<<8)|jy61->ucRxBuffer[6];
            jy61->stcMag.T = ((unsigned short)jy61->ucRxBuffer[9]<<8)|jy61->ucRxBuffer[8];
            break;

        case 0x55:
            jy61->stcDStatus.sDStatus[0] = ((unsigned short)jy61->ucRxBuffer[3]<<8)|jy61->ucRxBuffer[2];
            jy61->stcDStatus.sDStatus[1] = ((unsigned short)jy61->ucRxBuffer[5]<<8)|jy61->ucRxBuffer[4];
            jy61->stcDStatus.sDStatus[2] = ((unsigned short)jy61->ucRxBuffer[7]<<8)|jy61->ucRxBuffer[6];
            jy61->stcDStatus.sDStatus[3] = ((unsigned short)jy61->ucRxBuffer[9]<<8)|jy61->ucRxBuffer[8];
            break;

        case 0x56:
            jy61->ucRxBuffer[2] = 0x12;jy61->ucRxBuffer[3] = 0x34;jy61->ucRxBuffer[4] = 0x56;jy61->ucRxBuffer[5] = 0x78;
            CharToLong((char*)&jy61->stcPress.lPressure,(char*)&jy61->ucRxBuffer[2]);
            CharToLong((char*)&jy61->stcPress.lAltitude,(char*)&jy61->ucRxBuffer[6]);
            break;

        case 0x57:
            CharToLong((char*)&jy61->stcLonLat.lLon,(char*)&jy61->ucRxBuffer[2]);
            CharToLong((char*)&jy61->stcLonLat.lLat,(char*)&jy61->ucRxBuffer[6]);
            break;

        case 0x58:
            jy61->stcGPSV.sGPSHeight = ((unsigned short)jy61->ucRxBuffer[3]<<8)|jy61->ucRxBuffer[2];
            jy61->stcGPSV.sGPSYaw = ((unsigned short)jy61->ucRxBuffer[5]<<8)|jy61->ucRxBuffer[4];
            CharToLong((char*)&jy61->stcGPSV.lGPSVelocity,(char*)&jy61->ucRxBuffer[6]);
            break;
        }
        jy61->ucRxCnt=0;
    }
}

void Roll_Pitch_Yaw_Data_Count(JY61 * jy61){
		jy61->Roll_Pitch_Yaw_Data.Roll_X.Data.floatdata=jy61->stcAngle.Angle[0]/32768.0*180;
		jy61->Roll_Pitch_Yaw_Data.Pitch_Y.Data.floatdata=jy61->stcAngle.Angle[1]/32768.0*180;
		jy61->Roll_Pitch_Yaw_Data.Yaw_Z.Data.floatdata=jy61->stcAngle.Angle[2]/32768.0*180;
		jy61->Roll_Pitch_Yaw_Data.T.Data.floatdata=jy61->stcAngle.T/100.0;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
