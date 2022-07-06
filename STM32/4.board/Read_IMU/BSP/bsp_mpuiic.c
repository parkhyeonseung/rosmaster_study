/*
 * bsp_mpuiic.c
 *
 *  Created on: Mar 4, 2022
 *      Author: Administrator
 */

#include "bsp_mpuiic.h"


// Microsecond delay  微秒级延迟
static void Delay_For_Pin(uint8_t nCount)
{
    uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < 10; i++);
    }
}

#define delay_us  Delay_For_Pin

// Initialize the IIC  初始化IIC
void MPU_IIC_Init(void)
{
	delay_us(1);
}


// Generates the IIC initiation signal  产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SDA(1);
	MPU_IIC_SCL(1);
	delay_us(4);
 	MPU_IIC_SDA(0);
	delay_us(4);
	MPU_IIC_SCL(0);
}

// Generates an IIC stop signal  产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SCL(0);
	MPU_IIC_SDA(0);
 	delay_us(4);
	MPU_IIC_SCL(1);
	MPU_IIC_SDA(1);
	delay_us(4);
}

// 等待应答信号到来
// 返回值：1，接收应答失败. 0，接收应答成功
// Wait for the answer signal to arrive.
// Return value: 1, receive and reply failed 0, receive and reply succeeded
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MPU_SDA_IN();
	MPU_IIC_SDA(1);delay_us(1);
	MPU_IIC_SCL(1);delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL(0);
	return 0;
}

// Generate AN ACK reply  产生ACK应答
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(0);
	delay_us(2);
	MPU_IIC_SCL(1);
	delay_us(2);
	MPU_IIC_SCL(0);
}
// No ACK response is generated  不产生ACK应答
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(1);
	delay_us(2);
	MPU_IIC_SCL(1);
	delay_us(2);
	MPU_IIC_SCL(0);
}

// IIC发送一个字节，返回从机有无应答，1，有应答，0，无应答
// The IIC sends a byte that returns whether the slave machine answered, 1, yes, 0, no
void MPU_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	MPU_SDA_OUT();
    MPU_IIC_SCL(0);
    for(t=0;t<8;t++)
    {
        MPU_IIC_SDA((txd&0x80)>>7);
        txd<<=1;
		delay_us(2);
		MPU_IIC_SCL(1);
		delay_us(2);
		MPU_IIC_SCL(0);
		delay_us(2);
    }
}
// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
// Read 1 byte, ack=1, send ACK, ack=0, send nACK
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL(0);
        delay_us(2);
		MPU_IIC_SCL(1);
        receive<<=1;
        if(READ_SDA)receive++;
		delay_us(1);
    }
    if (!ack)
        MPU_IIC_NAck();
    else
        MPU_IIC_Ack();
    return receive;
}
