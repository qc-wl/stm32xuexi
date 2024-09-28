#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6���İ�
//IIC ��������	   
//�汾��V1.0					  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��IIC
void IIC_Init(void)
{					     
	RCC->AHB1ENR|=1<<1;    //ʹ��PORTBʱ��	   	  
	GPIO_Set(GPIOB,PIN8|PIN9,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB8/PB9���� 
	IIC_SCL=1;
	IIC_SDA=1;
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //SDA�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;  //��ʼ�ź�:SCLΪ�ߵ�ƽʱ,SDA�ɸߵ�ƽ��͵�ƽ���� 
	delay_us(4);
	IIC_SCL=0;  //ǯסIIC���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//SDA�����
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4); 
	IIC_SCL=1;
 	delay_us(4); //��ʱ���ȴ�SCL�ȶ�Ϊ�ߵ�ƽ
	IIC_SDA=1;//�����ź�:SCLΪ�ߵ�ƽʱ��SDA�ɵ͵�ƽ��ߵ�ƽ����			
  delay_us(4);	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   //�ͷ�SDA��
	IIC_SCL=1;delay_us(1);	   //ʱ����Ϊ�ߵ�ƽ
	while(READ_SDA)            //�ж��Ƿ���յ���Ӧ���źţ��͵�ƽΪ��Ч�ź�
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;    //û���յ�Ӧ���ź�
		}
	}
	IIC_SCL=0;//ʱ�������0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();     //SDA����Ϊ���
	IIC_SDA=0;     //����Ӧ���ź�
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();    //SDA����Ϊ���
	IIC_SDA=1;    //��Ӧ���ź�  
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;     //��SCL������SDA�ϵ����ݽ��д��д���
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   //SCLΪ�ߵ�ƽ�ڼ䣬��ȡSDA�ϵ�����
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;  //���ض���������
}



























