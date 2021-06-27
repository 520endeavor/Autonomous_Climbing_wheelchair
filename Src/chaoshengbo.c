#include "chaoshengbo.h"
static void IIC_Start(void);
static void IIC_Stop(void);
static u8 IIC_Wait_Ack(void);
static void IIC_Send_Byte(u8 txd);
static u8 IIC_Read_Byte(unsigned char ack);
ChaoShengBoData ChaoShengBodata;
static void delay_us(uint32_t cnt)  
{  
    uint32_t i,j;  
    for(i=0;i<cnt;i++)  
    {  
        for(j=0;j<180;j++);  
    }  
}

u8 KS103_ReadOneByte(u8 address, u8 reg)
{
	u8 temp=0;
	IIC_Start();
	IIC_Send_Byte(address); //发送低地址
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //发送低地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(address + 1); //进入接收模式
	IIC_Wait_Ack();
	delay_us(50); //增加此代码通信成功！！！
	temp=IIC_Read_Byte(0); //读寄存器3
	IIC_Stop();//产生一个停止条件
	return temp;
}
void KS103_WriteOneByte(u8 address,u8 reg,u8 command)
{
	IIC_Start();
	IIC_Send_Byte(address); //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);//发送高地址
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //发送低地址
	IIC_Wait_Ack();
	IIC_Stop();//产生一个停止条件
}
void ChaoShengBo_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;  
    __HAL_RCC_GPIOG_CLK_ENABLE();   //使能GPIOE时钟
    //PE4,5初始化设置
  /*Configure GPIO pins : PGPin PGPin PGPin */
  GPIO_InitStruct.Pin = I2C_SCL_Pin|I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    
    ChaoShengBo_IIC_SDA=1;
    ChaoShengBo_IIC_SCL=1; 


}
//产生IIC 起始信号
static void IIC_Start(void)
{
	ChaoShengBo_SDA_OUT(); //sda 线输出
	ChaoShengBo_IIC_SDA=1;
	ChaoShengBo_IIC_SCL=1;
	delay_us(10);
	ChaoShengBo_IIC_SDA=0;//START:when CLK is high,DATA change form high to low
	delay_us(10);
	ChaoShengBo_IIC_SCL=0;//钳住I2C 总线，准备发送或接收数据
}
//产生IIC 停止信号
static void IIC_Stop(void)
{
	ChaoShengBo_SDA_OUT();//sda 线输出
	ChaoShengBo_IIC_SCL=0;
	ChaoShengBo_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(10);
	ChaoShengBo_IIC_SCL=1;
	ChaoShengBo_IIC_SDA=1;//发送I2C 总线结束信号
	delay_us(10);
}
//等待应答信号到来
//返回值：1，接收应答失败
// 0，接收应答成功
static u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	ChaoShengBo_SDA_IN(); //SDA 设置为输入
	ChaoShengBo_IIC_SDA=1;delay_us(6);
	ChaoShengBo_IIC_SCL=1;delay_us(6);
	while(ChaoShengBo_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	ChaoShengBo_IIC_SCL=0;//时钟输出0
	return 0;
}
//产生ACK 应答
static void IIC_Ack(void)
{
	ChaoShengBo_IIC_SCL=0;
	ChaoShengBo_SDA_OUT();
	ChaoShengBo_IIC_SDA=0;
	delay_us(10);
	ChaoShengBo_IIC_SCL=1;
	delay_us(10);
	ChaoShengBo_IIC_SCL=0;
}
//不产生ACK 应答
static void IIC_NAck(void)
{
	ChaoShengBo_IIC_SCL=0;
	ChaoShengBo_SDA_OUT();
	ChaoShengBo_IIC_SDA=1;
	delay_us(10);
	ChaoShengBo_IIC_SCL=1;
	delay_us(10);
	ChaoShengBo_IIC_SCL=0;
}
//IIC 发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答

static void IIC_Send_Byte(u8 txd)
	{
	u8 t;
	ChaoShengBo_SDA_OUT();
	ChaoShengBo_IIC_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{
		ChaoShengBo_IIC_SDA=(txd&0x80)>>7;
		txd<<=1;
		delay_us(10);
		ChaoShengBo_IIC_SCL=1;
		delay_us(10);
		ChaoShengBo_IIC_SCL=0;
		delay_us(10);
	}
}
//读1 个字节，ack=1 时，发送ACK，ack=0，发送nACK
static u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	ChaoShengBo_SDA_IN();//SDA 设置为输入
	for(i=0;i<8;i++ )
	{
		ChaoShengBo_IIC_SCL=0;
		delay_us(10);
		ChaoShengBo_IIC_SCL=1;
		receive<<=1;
		if(ChaoShengBo_READ_SDA)receive++;
		delay_us(5);
	}
	if (!ack)
	IIC_NAck();//发送nACK
	else
	IIC_Ack(); //发送ACK
	return receive;
}


