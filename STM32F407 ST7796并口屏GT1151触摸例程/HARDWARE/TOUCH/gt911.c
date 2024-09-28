#include "gt911.h"
#include "touch.h"
#include "ctiic.h"
#include "usart.h"
#include "delay.h" 
#include "string.h" 
#include "lcd.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6���İ�
//���ݴ�����-GT911 ��������	   
//�汾��V1.0								  
////////////////////////////////////////////////////////////////////////////////// 

//GT911���������
//x����������ֵ0x01E0=480
//y����������ֵ0x0110=272
const u8 GT911_CFG_TBL[]=
{ 
	
	0x43,0x20,0x03,0xE0,0x01,0x05,0x0D,0x00,0x01,0x08,
	0x28,0x05,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8A,0x2A,0x0C,
	0x35,0x33,0x7C,0x06,0x00,0x00,0x01,0x9A,0x03,0x1D,
	0x00,0x00,0x00,0x00,0x00,0x03,0x64,0x32,0x00,0x00,
	0x00,0x32,0x82,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
	0x77,0x37,0x00,0x63,0x42,0x00,0x53,0x50,0x00,0x44,
	0x62,0x00,0x39,0x76,0x00,0x39,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,
	0x12,0x14,0x16,0x18,0xFF,0xFF,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0F,
	0x10,0x12,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,0x21,
	0x22,0x24,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x31,0x01
		
};  

void GT911_Checksum(void)
	
{
	u8 buf[2];
	u8 buf1[10];
	u8 i=0;
	buf[0]=0;
	for(i=0;i<(sizeof(GT911_CFG_TBL)-2);i++)  buf[0]+=GT911_CFG_TBL[i];//����У���
	buf[0]=(~buf[0])+1;
	printf("Checksum:0x%x\r\n",buf[0]);	
	sprintf((char*)buf1,"Checksum:0x%X",buf[0]);
//	LCD_ShowString(10,74,1024,600,16,buf1);
}


//����GT911���ò���
//mode:0,���������浽flash
//     1,�������浽flash
u8 GT911_Send_Cfg(u8 mode)
{

	u8 buf[2];
	u8 i=0;
	buf[0]=0;
	buf[1]=mode;	//�Ƿ�д�뵽GT911 FLASH?  ���Ƿ���籣��
	for(i=0;i<(sizeof(GT911_CFG_TBL)-2);i++)buf[0]+=GT911_CFG_TBL[i];		//����У���
	buf[0]=(~buf[0])+1;
	printf("Checksum:0x%X\r\n",buf[0]);
	GT911_WR_Reg(GT911_CFGS_REG,(u8*)GT911_CFG_TBL,sizeof(GT911_CFG_TBL)-2);	//���ͼĴ�������
	GT911_WR_Reg(GT911_CHECK_REG,buf,2);    //д��У���,�����ø��±��
	return 0;
} 




//��GT911д��һ������
//reg:��ʼ�Ĵ�����ַ
//buf:���ݻ�������
//len:д���ݳ���
//����ֵ:0,�ɹ�;1,ʧ��.
u8 GT911_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT911_CMD_WR);   	//����д���� 	  0x28
	CT_IIC_Wait_Ack();
	CT_IIC_Send_Byte(reg>>8);   	//���͸�8λ��ַ
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//���͵�8λ��ַ
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    CT_IIC_Send_Byte(buf[i]);  	//������
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//����һ��ֹͣ����	    
	return ret; 
}
//��GT911����һ������
//reg:��ʼ�Ĵ�����ַ
//buf:���ݻ�������
//len:�����ݳ���			  
void GT911_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT911_CMD_WR);   //����д���� 	0x28 
	CT_IIC_Wait_Ack();
 	CT_IIC_Send_Byte(reg>>8);   	  //���͸�8λ��ַ
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//���͵�8λ��ַ
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(GT911_CMD_RD);   //���Ͷ�����		    0x29
	CT_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //������	  
	} 
    CT_IIC_Stop();//����һ��ֹͣ����    
} 
//��ʼ��GT911������
//����ֵ:0,��ʼ���ɹ�;1,��ʼ��ʧ�� 
u8 GT911_Init(void)
{
	u8 temp[5]; 

	
	RCC->AHB1ENR|=1<<1;    		//ʹ��PORTBʱ�� 
	RCC->AHB1ENR|=1<<2;    		//ʹ��PORTCʱ��  
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU); 	//PB1����Ϊ��������
	GPIO_Set(GPIOC,PIN13,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PC13����Ϊ�������
	
	CT_IIC_Init();  //��ʼ����������I2C����  
	GT_RST=0;	    //��λ
	delay_ms(10);
 	GT_RST=1;	   //�ͷŸ�λ		    
	delay_ms(10); 
	
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_NONE);//PB1����Ϊ��������
	
	delay_ms(100);  
	GT911_RD_Reg(GT911_PID_REG,temp,4);//��ȡ��ƷID
	temp[4]=0;
	
	printf("CTP ID:%s\r\n",temp);	 //��ӡID
	if(strcmp((char*)temp,"911")==0)//ID==911
	{
			temp[0]=0X02;			
			GT911_WR_Reg(GT911_CTRL_REG,temp,1);//��λGT911
			GT911_RD_Reg(GT911_CFGS_REG,temp,1);//��ȡGT_CFGS_REG�Ĵ���
			printf("Default Ver:%d\r\n",temp[0]);
		//if(temp[0]<0X60)//Ĭ�ϰ汾�Ƚϵ�,��Ҫ����flash����
		//if(temp[0]<0x60)//Ĭ�ϰ汾�Ƚϵ�,��Ҫ����flash����
		/*
		{
			printf("Default Ver:%d\r\n",temp[0]);
			GT911_Send_Cfg(1);//���²���������
		}
		*/
			//GT911_RD_Reg(GT_CFGS_REG,temp,1);
			//printf("Default Ver:%d\r\n",temp[0]);
		
		delay_ms(10);
		temp[0]=0X00;	 
		GT911_WR_Reg(GT911_CTRL_REG,temp,1);//������λ   
		return 0;
		//printf(" CTP ID:%s\r\n",temp);
	} 
	return 1;
}
const u16 GT911_TPX_TBL1[5]=
{GT911_TP1_REG,GT911_TP2_REG,GT911_TP3_REG,GT911_TP4_REG,GT911_TP5_REG,};
//ɨ�败����(���ò�ѯ��ʽ)
//mode:0,����ɨ��.
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
u8 GT911_Scan(u8 mode)
{
	u8 buf[4];
	u8 i=0;
	u8 res=0;
	u16 temp;
	u16 tempsta;
 	static u8 t=0;//���Ʋ�ѯ���,�Ӷ�����CPUռ����   
	t++;
	if((t%10)==0||t<10)//����ʱ,ÿ����10��CTP_Scan�����ż��1��,�Ӷ���ʡCPUʹ����
	{
		GT911_RD_Reg(GT911_GSTID_REG,&mode,1);	//��ȡ�������״̬  
 		if(mode&0X80&&((mode&0XF)<11))
		{
			i=0;
			GT911_WR_Reg(GT911_GSTID_REG,&i,1);	//���־ 		
		}		
		if((mode&0XF)&&((mode&0XF)<11))
		{
			temp=0XFFFF<<(mode&0XF);	//����ĸ���ת��Ϊ1��λ��,ƥ��tp_dev.sta���� 
			tempsta=tp_dev.sta;			//���浱ǰ��tp_dev.staֵ
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];	//���津��0������
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<10;i++)
			{
				if(tp_dev.sta&(1<<i))	//������Ч?
				{
					GT911_RD_Reg(GT911_TPX_TBL1[i],buf,6);	//��ȡXY����ֵ
									
					tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
					tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
				
	//				if((tp_dev.x[i]>0)&&(tp_dev.x[i]<LCD_XSIZE_TFT)&&(tp_dev.y[i]>0)&&(tp_dev.y[i]<LCD_YSIZE_TFT))
	//				printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]); 
				}			
			} 
			res=1;
			if(tp_dev.x[0]>lcddev.width||tp_dev.y[0]>lcddev.height)//�Ƿ�����(���곬����)
			{ 
				if((mode&0XF)>1)		//��������������,�򸴵ڶ�����������ݵ���һ������.
				{
					tp_dev.x[0]=tp_dev.x[1];
					tp_dev.y[0]=tp_dev.y[1];
					t=0;				//����һ��,��������������10��,�Ӷ����������
				}else					//�Ƿ�����,����Դ˴�����(��ԭԭ����)  
				{
					tp_dev.x[0]=tp_dev.x[4];
					tp_dev.y[0]=tp_dev.y[4];
					mode=0X80;		
					tp_dev.sta=tempsta;	//�ָ�tp_dev.sta
				}
			}else t=0;					//����һ��,��������������10��,�Ӷ����������
		}
	}
	if((mode&0X8F)==0X80)//�޴����㰴��
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)		//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~TP_PRES_DOWN;	//��ǰ����ɿ�
		}else							//֮ǰ��û�б�����
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE000;			//�������Ч���	
		}	 
	} 	
	if(t>240)t=10;//���´�10��ʼ����
	return res;
}
 



























