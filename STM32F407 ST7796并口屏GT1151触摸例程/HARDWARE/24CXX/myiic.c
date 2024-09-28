#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6核心板
//IIC 驱动代码	   
//版本：V1.0					  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化IIC
void IIC_Init(void)
{					     
	RCC->AHB1ENR|=1<<1;    //使能PORTB时钟	   	  
	GPIO_Set(GPIOB,PIN8|PIN9,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB8/PB9设置 
	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //SDA线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;  //开始信号:SCL为高电平时,SDA由高电平向低电平跳变 
	delay_us(4);
	IIC_SCL=0;  //钳住IIC总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//SDA线输出
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4); 
	IIC_SCL=1;
 	delay_us(4); //延时，等待SCL稳定为高电平
	IIC_SDA=1;//结束信号:SCL为高电平时，SDA由低电平向高电平跳变			
  delay_us(4);	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   //释放SDA线
	IIC_SCL=1;delay_us(1);	   //时钟线为高电平
	while(READ_SDA)            //判断是否接收到了应答信号，低电平为有效信号
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;    //没有收到应答信号
		}
	}
	IIC_SCL=0;//时钟线输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();     //SDA设置为输出
	IIC_SDA=0;     //产生应答信号
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();    //SDA设置为输出
	IIC_SDA=1;    //非应答信号  
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;     //在SCL上升沿SDA上的数据进行串行传送
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   //SCL为高电平期间，读取SDA上的数据
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;  //返回读到的数据
}



























