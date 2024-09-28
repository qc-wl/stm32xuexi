#include "gt5668.h"
#include "touch.h"
#include "ctiic.h"
#include "usart.h"
#include "delay.h" 
#include "string.h" 

//GT5668���������



//x����������ֵ0x0140=320
//y����������ֵ0x01E0=480
//0x48=72
//0x8050~0x813B  239���Ĵ���
//0x813C 0x813DУ���
//0x813E ���ø��±��
const u8 GT5668_CFG_TBL[]=
{ 
	0x48,0x40,0x01,0xE0,0x01,0x05,0x75,0x14,0x00,0x40,
	0x00,0x0A,0x55,0x3C,0x53,0x11,0x01,0x01,0x00,0x00,
	0x14,0x18,0x1A,0x1E,0x0F,0x04,0x00,0x00,0x10,0x10,
	0x00,0x20,0x00,0x00,0x28,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x64,0x1E,0x28,0x88,0x27,0x0A,0x27,
	0x25,0x12,0x0C,0x60,0x12,0x60,0x11,0x03,0x27,0x00,
	0x00,0x1E,0x50,0x80,0x02,0x03,0x00,0x00,0x53,0xD6,
	0x26,0xAF,0x2E,0x94,0x37,0x80,0x3F,0x72,0x47,0x67,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x32,0x20,0x32,0x32,0x64,0x00,0x00,0x00,0x00,0x00,
	0x02,0x09,0x03,0x0A,0x04,0x0B,0x05,0x0C,0x06,0x0D,
	0xFF,0xFF,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,0x05,
	0x06,0x0E,0x0D,0x0C,0x0B,0x0A,0x09,0x08,0x07,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x04,
	0x33,0x02,0xF4,0x0A,0xBA,0x00,0x19,0x7F,0x44,0x28,
	0x46,0x32,0x50,0x00,0x00,0x12,0x50,0x23,0x01
};  




u16 CRC162(u8 *srcdata,u16 length)
{
	u16 crc=0xffff;
	u16 i,j;
	u8 value;
	for(i=0;i<length;i++)
	{
		for(j=0;j<8;j++)
		{
			value=((srcdata[i]<<j)&0x80)^((crc&0x8000)>>8);	
			crc<<=1;
			if(value!=0)
			{
				crc^=0x8005;
			}
		}
	}
	return crc;
}
//����GT5668���ò���
//mode:0,���������浽flash
//     1,�������浽flash
u8 GT5668_Send_Cfg(u8 mode)
{

	u16 checksum=0;
	u8 buf[3];
	u8 i=0;	
	for(i=0;i<(sizeof(GT5668_CFG_TBL)-3);i+=2)
		checksum +=((GT5668_CFG_TBL[i]<<8)|GT5668_CFG_TBL[i+1]);//����У���  0xafdd
		//checksum +=(GT5668_CFG_TBL[i]<<8)+GT5668_CFG_TBL[i+1];
	//checksum =0-checksum;
	 checksum =(~checksum)+1;
	//printf("Bytesum:%d,\r\n",sizeof(GT5668_CFG_TBL));
	printf("chksum:0x%x,\r\n",checksum);
	buf[0]= checksum>>8;
	buf[1]= checksum;
	buf[2]= mode;	//�Ƿ�д�뵽GT5668 FLASH?  ���Ƿ���籣��
	printf("chksum_H:0x%x,\r\n",buf[0]);
	printf("chksum_L:0x%x,\r\n",buf[1]);
	printf("\r\ncrc=%x",CRC162((u8*)GT5668_CFG_TBL,sizeof(GT5668_CFG_TBL)));
	GT5668_WR_Reg(GT_CFGS_REG,(u8*)GT5668_CFG_TBL,sizeof(GT5668_CFG_TBL));//���ͼĴ�������
	GT5668_WR_Reg(GT_CHECK_REG,buf,3);//д��У���,�����ø��±��
	return 0;

} 

//����GT5668���ò���
//mode:0,���������浽flash
//     1,�������浽flash
u8 GT5668_Send_Cfg1(u8 mode)
{

	u16 checksum=0;
	u8 buf[3];
	u8 i=0;	
	for(i=0;i<(sizeof(GT5668_CFG_TBL)-3);i+=2)
		checksum +=(GT5668_CFG_TBL[i]<<8)+GT5668_CFG_TBL[i+1];//����У���
	checksum =0-checksum;
	//printf("Bytesum:%d,\r\n",sizeof(GT5668_CFG_TBL));
	printf("chksum:0x%x,\r\n",checksum);
	buf[0]= checksum>>8;
	buf[1]= checksum;
	buf[2]= mode;	//�Ƿ�д�뵽GT5668 FLASH?  ���Ƿ���籣��
	printf("chksum_H:0x%x,\r\n",buf[0]);
	printf("chksum_L:0x%x,\r\n",buf[1]);
	printf("\r\ncrc=%x",CRC162((u8*)GT5668_CFG_TBL,sizeof(GT5668_CFG_TBL)));
	GT5668_WR_Reg(GT_CFGS_REG,(u8*)GT5668_CFG_TBL,sizeof(GT5668_CFG_TBL));//���ͼĴ�������
	GT5668_WR_Reg(GT_CHECK_REG,buf,3);//д��У���,�����ø��±��
	return 0;

} 


//��GT5668д��һ������
//reg:��ʼ�Ĵ�����ַ
//buf:���ݻ�������
//len:д���ݳ���
//����ֵ:0,�ɹ�;1,ʧ��.
u8 GT5668_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   	//����д���� 	  0x28
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
//��GT5668����һ������
//reg:��ʼ�Ĵ�����ַ
//buf:���ݻ�������
//len:�����ݳ���			  
void GT5668_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   //����д���� 	0x28 
	CT_IIC_Wait_Ack();
 	CT_IIC_Send_Byte(reg>>8);   	  //���͸�8λ��ַ
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//���͵�8λ��ַ
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(GT_CMD_RD);   //���Ͷ�����	0x29
	CT_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //������	  
	} 
    CT_IIC_Stop();//����һ��ֹͣ����    
} 
//��ʼ��GT5668������
//����ֵ:0,��ʼ���ɹ�;1,��ʼ��ʧ�� 
u8 GT5668_Init(void)
{
	u8 temp[5]; 
	u8 i=0;
	u8 Cfg_Info[239] = {0};
  RCC->AHB1ENR|=1<<1;    		//ʹ��PORTBʱ�� 
	RCC->AHB1ENR|=1<<2;    		//ʹ��PORTCʱ��  
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU); 	//PB1����Ϊ��������
	GPIO_Set(GPIOC,PIN13,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PC13����Ϊ�������
	
	CT_IIC_Init();      	//��ʼ����������I2C����  
	GT_RST=0;				//��λ
	delay_ms(10);
 	GT_RST=1;				//�ͷŸ�λ		    
	delay_ms(10); 
	
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_NONE);//PB1����Ϊ��������
	
	delay_ms(100);  
	GT5668_RD_Reg(GT_PID_REG,temp,4);//��ȡ��ƷID
	temp[4]=0;
	printf("CTP ID:%s\r\n",temp);	//��ӡID
	
	//GT5668_RD_Reg(GT_GSTID_REG,temp1,1);	//��ȡ�������״̬
	//printf("0X814E:0x%x,\r\n",temp1[0]);
	
	//GT5668_RD_Reg(0x81A8,temp2,1);
	//printf("0X81A8:0x%x,\r\n",temp2[0]);
	
	if(strcmp((char*)temp,"5668")==0)//ID==9147
	{
		temp[0]=0X02;			
		//GT5668_WR_Reg(GT_CTRL_REG,temp,1);//��λGT5668
		GT5668_RD_Reg(GT_CFGS_REG,temp,1);//��ȡGT_CFGS_REG�Ĵ���
		if(temp[0]<0X60)//Ĭ�ϰ汾�Ƚϵ�,��Ҫ����flash����
		{
			//printf("Default Ver:%d\r\n",temp[0]);
			printf("Default Ver:0x%x\r\n",temp[0]);
			//GT5668_Send_Cfg(1);//���²���������
			
			//GT5668_RD_Reg(0x813C,temp,1);
			//printf("0x813C:0x%x\r\n",temp[0]);
			//GT5668_RD_Reg(0x813D,temp,1);
			//printf("0x813D:0x%x\r\n",temp[0]);
		}
		
		#if 1
		GT5668_RD_Reg(0x8050,Cfg_Info,239);	
		printf("Config Info:\r\n");
		for( i = 0; i < 239; i++ )
		{
		printf("0x%02X,",Cfg_Info[i]);
		if((i+1)%10==0)
		printf("\r\n");
		}
		printf("\r\n");
		#endif	
		
		
		//delay_ms(10);
		//temp[0]=0X00;	 
		//GT5668_WR_Reg(GT_CTRL_REG,temp,1);//������λ 

		//GT5668_RD_Reg(GT_GSTID_REG,temp1,1);	//��ȡ�������״̬
		//printf("0X814E:0x%x,\r\n",temp1[0]);
		return 0;
	} 
	return 1;
}
const u16 GT5668_TPX_TBL[5]={GT_TP1_REG,GT_TP2_REG,GT_TP3_REG,GT_TP4_REG,GT_TP5_REG};
//ɨ�败����(���ò�ѯ��ʽ)
//mode:0,����ɨ��.
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
u8 GT5668_Scan(u8 mode)
{
	
	u8 buf[4];
	//u8 temp2[1];
	u8 i=0;
	u8 res=0;
	u8 temp;
	u8 tempsta;
 	static u8 t=0;//���Ʋ�ѯ���,�Ӷ�����CPUռ����   
	t++;
	if((t%10)==0||t<10)//����ʱ,ÿ����10��CTP_Scan�����ż��1��,�Ӷ���ʡCPUʹ����
	{
		GT5668_RD_Reg(GT_GSTID_REG,&mode,1);	//��ȡ�������״̬
		//printf("0X814E:0x%x,\r\n",mode);
		//GT5668_RD_Reg(GT_GSTID_REG,temp2,1);
		//printf("0X814E:0x%x,\r\n",temp2[0]);
		//delay_ms(100);
 		if(mode&0X80&&((mode&0XF)<6))
		{
			temp=0;
			GT5668_WR_Reg(GT_GSTID_REG,&temp,1);//���־ 		
		}		
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);		//����ĸ���ת��Ϊ1��λ��,ƥ��tp_dev.sta���� 
			tempsta=tp_dev.sta;			//���浱ǰ��tp_dev.staֵ
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];	//���津��0������
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<5;i++)
			{
				if(tp_dev.sta&(1<<i))	//������Ч?
				{
					GT5668_RD_Reg(GT5668_TPX_TBL[i],buf,4);	//��ȡXY����ֵ
					if(tp_dev.touchtype&0X01)//����
					{
						tp_dev.y[i]=((u16)buf[1]<<8)+buf[0];
						tp_dev.x[i]=800-(((u16)buf[3]<<8)+buf[2]);
					}else
					{
						tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
						tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
					}  
					printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]);
				}			
			} 
			res=1;
			if(tp_dev.x[0]>320||tp_dev.y[0]>240)//�Ƿ�����(���곬����)
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
		if(tp_dev.sta&TP_PRES_DOWN)	//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~(1<<7);	//��ǰ����ɿ�
		}else						//֮ǰ��û�б�����
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//�������Ч���	
		}	 
	} 	
	if(t>240)t=10;//���´�10��ʼ����
	return res;
	
}
 



























