/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
#ifndef __MAIN_H
#define __MAIN_H

  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DS2_Pin GPIO_PIN_3
#define DS2_GPIO_Port GPIOE
#define DS3_Pin GPIO_PIN_4
#define DS3_GPIO_Port GPIOE
#define DS4_Pin GPIO_PIN_5
#define DS4_GPIO_Port GPIOE
#define DS5_Pin GPIO_PIN_6
#define DS5_GPIO_Port GPIOE
#define PushRodFrontUp_Pin GPIO_PIN_15
#define PushRodFrontUp_GPIO_Port GPIOE
#define PushRodFrontDown_Pin GPIO_PIN_1
#define PushRodFrontDown_GPIO_Port GPIOG
#define PushRodBehindUp_Pin GPIO_PIN_12
#define PushRodBehindUp_GPIO_Port GPIOF
#define PushRodBehindDown_Pin GPIO_PIN_11
#define PushRodBehindDown_GPIO_Port GPIOF
#define PushRodSwitchPush_Pin GPIO_PIN_4
#define PushRodSwitchPush_GPIO_Port GPIOC
#define PushRodSwitchPull_Pin GPIO_PIN_5
#define PushRodSwitchPull_GPIO_Port GPIOC
#define AutoModeLed_Pin GPIO_PIN_13
#define AutoModeLed_GPIO_Port GPIOF
#define ManualModeLed_Pin GPIO_PIN_14
#define ManualModeLed_GPIO_Port GPIOF
#define StepMotPulse_Pin GPIO_PIN_15
#define StepMotPulse_GPIO_Port GPIOF
#define ModeSwitchKey_Pin GPIO_PIN_7
#define ModeSwitchKey_GPIO_Port GPIOE
#define SETP_Up_Manual_Pin GPIO_PIN_8
#define SETP_Up_Manual_GPIO_Port GPIOE
#define TrackMotLeft_Pin GPIO_PIN_9
#define TrackMotLeft_GPIO_Port GPIOE
#define SETP_Down_Manual_Pin GPIO_PIN_10
#define SETP_Down_Manual_GPIO_Port GPIOE
#define TrackMotRight_Pin GPIO_PIN_11
#define TrackMotRight_GPIO_Port GPIOE
#define StepMotDir_Pin GPIO_PIN_12
#define StepMotDir_GPIO_Port GPIOE
#define HubMotLeft_Pin GPIO_PIN_13
#define HubMotLeft_GPIO_Port GPIOE
#define HubMotRight_Pin GPIO_PIN_14
#define HubMotRight_GPIO_Port GPIOE
#define JY61_2_TX_Pin GPIO_PIN_10
#define JY61_2_TX_GPIO_Port GPIOB
#define JY61_2_RX_Pin GPIO_PIN_11
#define JY61_2_RX_GPIO_Port GPIOB
#define StepMotPulse_0_Pin GPIO_PIN_8
#define StepMotPulse_0_GPIO_Port GPIOC
#define SteeringEngine1_Pin GPIO_PIN_1
#define SteeringEngine1_GPIO_Port GPIOD
#define SteeringEngine2_Pin GPIO_PIN_2
#define SteeringEngine2_GPIO_Port GPIOD
#define SteeringEngine3_Pin GPIO_PIN_3
#define SteeringEngine3_GPIO_Port GPIOD
#define SteeringEngine4_Pin GPIO_PIN_4
#define SteeringEngine4_GPIO_Port GPIOD
#define JY61_1_TX_Pin GPIO_PIN_5
#define JY61_1_TX_GPIO_Port GPIOD
#define JY61_1_RX_Pin GPIO_PIN_6
#define JY61_1_RX_GPIO_Port GPIOD
#define HubMotLeftDirBack_Pin GPIO_PIN_7
#define HubMotLeftDirBack_GPIO_Port GPIOD
#define HubMotRightDirBack_Pin GPIO_PIN_9
#define HubMotRightDirBack_GPIO_Port GPIOG
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOG
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOG

#define MOTOR_DIR1_FORWARD()   HAL_GPIO_WritePin(StepMotDir_GPIO_Port, StepMotDir_Pin, GPIO_PIN_RESET)
#define MOTOR_DIR1_REVERSAL()  HAL_GPIO_WritePin(StepMotDir_GPIO_Port, StepMotDir_Pin, GPIO_PIN_SET)
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
