/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Datatypedef.h"
/* USER CODE BEGIN Includes */

#define SAVE            0x00
#define CALSW       0x01
#define RSW             0x02
#define RRATE           0x03
#define BAUD            0x04
#define AXOFFSET    0x05
#define AYOFFSET    0x06
#define AZOFFSET    0x07
#define GXOFFSET    0x08
#define GYOFFSET    0x09
#define GZOFFSET    0x0a
#define HXOFFSET    0x0b
#define HYOFFSET    0x0c
#define HZOFFSET    0x0d
#define D0MODE      0x0e
#define D1MODE      0x0f
#define D2MODE      0x10
#define D3MODE      0x11
#define D0PWMH      0x12
#define D1PWMH      0x13
#define D2PWMH      0x14
#define D3PWMH      0x15
#define D0PWMT      0x16
#define D1PWMT      0x17
#define D2PWMT      0x18
#define D3PWMT      0x19
#define IICADDR     0x1a
#define LEDOFF      0x1b
#define GPSBAUD     0x1c

#define YYMM                0x30
#define DDHH                0x31
#define MMSS                0x32
#define MS                  0x33
#define AX                  0x34
#define AY                  0x35
#define AZ                  0x36
#define GX                  0x37
#define GY                  0x38
#define GZ                  0x39
#define HX                  0x3a
#define HY                  0x3b
#define HZ                  0x3c
#define Roll                0x3d
#define Pitch               0x3e
#define Yaw                 0x3f
#define TEMP                0x40
#define D0Status        0x41
#define D1Status        0x42
#define D2Status        0x43
#define D3Status        0x44
#define PressureL       0x45
#define PressureH       0x46
#define HeightL         0x47
#define HeightH         0x48
#define LonL                0x49
#define LonH                0x4a
#define LatL                0x4b
#define LatH                0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL               0x4f
#define GPSVH               0x50

#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5


typedef struct 
{
    unsigned char ucYear;
    unsigned char ucMonth;
    unsigned char ucDay;
    unsigned char ucHour;
    unsigned char ucMinute;
    unsigned char ucSecond;
    unsigned short usMiliSecond;
}STime;
typedef struct 
{
    short a[3];
    short T;
}SAcc;
typedef struct 
{
    short w[3];
    short T;
}SGyro;
typedef struct 
{
    short Angle[3];
    short T;
}SAngle;
typedef struct 
{
    short h[3];
    short T;
}SMag;

typedef struct 
{
    short sDStatus[4];
}SDStatus;

typedef struct 
{
    long lPressure;
    long lAltitude;
}SPress;

typedef struct 
{
    long lLon;
    long lLat;
}SLonLat;

typedef struct 
{
    short sGPSHeight;
    short sGPSYaw;
    long lGPSVelocity;
}SGPSV;


typedef struct 
{
	 FloatData Roll_X;
	 FloatData Pitch_Y;
	 FloatData Yaw_Z;
	 FloatData T;
	
}Roll_Pitch_Yaw;
	 
typedef struct {

 unsigned char ucRxBuffer[12];
 unsigned char ucRxCnt ;
	uint8_t rxData[1];
 STime        stcTime;
 SAcc         stcAcc;
 SGyro        stcGyro;
 SAngle       stcAngle;
 SMag         stcMag;
 SDStatus     stcDStatus;
 SPress       stcPress;
 SLonLat      stcLonLat;
 SGPSV        stcGPSV;
 Roll_Pitch_Yaw Roll_Pitch_Yaw_Data;
 FloatData ChairBalanceAngleMax;
 FloatData ChairBalanceAngleMin;
 FloatData StepMotVibrationAngle1;
 FloatData StepMotVibrationAngle2;
 uint8_t ReceiveData_OK;
}JY61;





void  CopeSerialData(JY61 * jy61);
	/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern JY61 JY61_1,JY61_2;
extern uint8_t ReceiveData,ReceiveCMD;
extern uint8_t rxData[7];
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void Roll_Pitch_Yaw_Data_Count(JY61 * jy61);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
