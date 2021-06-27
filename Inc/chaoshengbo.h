#ifndef __chaoshengbo_H
#define __chaoshengbo_H

#include "myiic.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Datatypedef.h"
typedef uint8_t u8;
typedef struct{
	uint16_t Data[10];
	uint8_t Add[10];
	FloatData floatData[10];
	FloatData SetData[4];
	FloatData SetData1[4];
	FloatData SetData2[4];
	uint8_t CalibrateState;
}ChaoShengBoData;
extern ChaoShengBoData ChaoShengBodata;



#define ChaoShengBo_SDA_IN()  {GPIOG->MODER&=~(3<<(11*2));GPIOG->MODER|=0<<11*2;}	//PE5输入模式
#define ChaoShengBo_SDA_OUT() {GPIOG->MODER&=~(3<<(11*2));GPIOG->MODER|=1<<11*2;} //PE5输出模式

//IO操作
#define ChaoShengBo_IIC_SCL   PGout(10) //SCL
#define ChaoShengBo_IIC_SDA   PGout(11) //SDA

#define ChaoShengBo_READ_SDA  PGin(11)  //输入SDA

u8 KS103_ReadOneByte(u8 address, u8 reg);
void KS103_WriteOneByte(u8 address,u8 reg,u8 command);
void ChaoShengBo_IIC_Init(void);

#endif

