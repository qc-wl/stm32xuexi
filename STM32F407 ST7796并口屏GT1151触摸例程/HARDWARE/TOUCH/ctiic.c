#include "ctiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//WKS STM32F407VET6核心板
//电容触摸屏-IIC 驱动代码
//版本：V1.0
//////////////////////////////////////////////////////////////////////////////////

//控制I2C速度的延时
void CT_Delay(void)
{
    delay_us(2);
}
//电容触摸芯片IIC接口初始化
void CT_IIC_Init(void)
{
    RCC->AHB1ENR |= 1 << 1;    		//使能PORTB时钟
    GPIO_Set(GPIOB, PIN12, GPIO_MODE_OUT, GPIO_OTYPE_OD, GPIO_SPEED_100M, GPIO_PUPD_PU); 	//PB12设置为推挽输出
    GPIO_Set(GPIOB, PIN15, GPIO_MODE_OUT, GPIO_OTYPE_OD, GPIO_SPEED_100M, GPIO_PUPD_PU);  //PB15设置为推挽输出
}
//产生IIC起始信号
void CT_IIC_Start(void)
{
    CT_IIC_SDA = 1;
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SDA = 0; //开始信号:SCL为高电平时,SDA由高电平向低电平跳变 
    CT_Delay();
    CT_IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
    CT_Delay();
}
//产生IIC停止信号
void CT_IIC_Stop(void)
{
    CT_IIC_SDA = 0; //结束信号:SCL为高电平时，SDA由低电平向高电平跳变
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SDA = 1; //发送I2C总线结束信号
    CT_Delay();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 CT_IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    u8 rack = 0;

    CT_IIC_SDA = 1;
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();

    while (CT_READ_SDA)
    {
        ucErrTime++;

        if (ucErrTime > 250)
        {
            CT_IIC_Stop();
            rack = 1;
            break;
        }

        CT_Delay();
    }

    CT_IIC_SCL = 0; //时钟输出0
    CT_Delay();
    return rack;
}
//产生ACK应答
void CT_IIC_Ack(void)
{
    CT_IIC_SDA = 0;
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SCL = 0;
    CT_Delay();
    CT_IIC_SDA = 1;
    CT_Delay();
}
//不产生ACK应答
void CT_IIC_NAck(void)
{
    CT_IIC_SDA = 1;
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SCL = 0;
    CT_Delay();
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void CT_IIC_Send_Byte(u8 txd)
{
    u8 t;

    for (t = 0; t < 8; t++)
    {
        CT_IIC_SDA = (txd & 0x80) >> 7;
        CT_Delay();
        CT_IIC_SCL = 1;  //在SCL上升沿时串行传送SDA上的数据
        CT_Delay();
        CT_IIC_SCL = 0;
        txd <<= 1;
    }

    CT_IIC_SDA = 1;
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 CT_IIC_Read_Byte(unsigned char ack)
{
    u8 i, receive = 0;

    for (i = 0; i < 8; i++ )
    {
        receive <<= 1;
        CT_IIC_SCL = 1;
        CT_Delay();

        if (CT_READ_SDA)receive++;  //SCL为高电平期间，读取SDA上的数据

        CT_IIC_SCL = 0;
        CT_Delay();
    }

    if (!ack)CT_IIC_NAck();//发送nACK
    else CT_IIC_Ack(); //发送ACK

    return receive;
}




























