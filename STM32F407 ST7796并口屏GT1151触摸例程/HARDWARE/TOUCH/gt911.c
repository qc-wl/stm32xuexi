#include "gt911.h"
#include "touch.h"
#include "ctiic.h"
#include "usart.h"
#include "delay.h" 
#include "string.h" 
#include "lcd.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6核心板
//电容触摸屏-GT911 驱动代码	   
//版本：V1.0								  
////////////////////////////////////////////////////////////////////////////////// 

//GT911配置数组表
//x坐标输出最大值0x01E0=480
//y坐标输出最大值0x0110=272
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
	for(i=0;i<(sizeof(GT911_CFG_TBL)-2);i++)  buf[0]+=GT911_CFG_TBL[i];//计算校验和
	buf[0]=(~buf[0])+1;
	printf("Checksum:0x%x\r\n",buf[0]);	
	sprintf((char*)buf1,"Checksum:0x%X",buf[0]);
//	LCD_ShowString(10,74,1024,600,16,buf1);
}


//发送GT911配置参数
//mode:0,参数不保存到flash
//     1,参数保存到flash
u8 GT911_Send_Cfg(u8 mode)
{

	u8 buf[2];
	u8 i=0;
	buf[0]=0;
	buf[1]=mode;	//是否写入到GT911 FLASH?  即是否掉电保存
	for(i=0;i<(sizeof(GT911_CFG_TBL)-2);i++)buf[0]+=GT911_CFG_TBL[i];		//计算校验和
	buf[0]=(~buf[0])+1;
	printf("Checksum:0x%X\r\n",buf[0]);
	GT911_WR_Reg(GT911_CFGS_REG,(u8*)GT911_CFG_TBL,sizeof(GT911_CFG_TBL)-2);	//发送寄存器配置
	GT911_WR_Reg(GT911_CHECK_REG,buf,2);    //写入校验和,和配置更新标记
	return 0;
} 




//向GT911写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
u8 GT911_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT911_CMD_WR);   	//发送写命令 	  0x28
	CT_IIC_Wait_Ack();
	CT_IIC_Send_Byte(reg>>8);   	//发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    CT_IIC_Send_Byte(buf[i]);  	//发数据
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//产生一个停止条件	    
	return ret; 
}
//从GT911读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void GT911_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT911_CMD_WR);   //发送写命令 	0x28 
	CT_IIC_Wait_Ack();
 	CT_IIC_Send_Byte(reg>>8);   	  //发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(GT911_CMD_RD);   //发送读命令		    0x29
	CT_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
    CT_IIC_Stop();//产生一个停止条件    
} 
//初始化GT911触摸屏
//返回值:0,初始化成功;1,初始化失败 
u8 GT911_Init(void)
{
	u8 temp[5]; 

	
	RCC->AHB1ENR|=1<<1;    		//使能PORTB时钟 
	RCC->AHB1ENR|=1<<2;    		//使能PORTC时钟  
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU); 	//PB1设置为上拉输入
	GPIO_Set(GPIOC,PIN13,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PC13设置为推挽输出
	
	CT_IIC_Init();  //初始化电容屏的I2C总线  
	GT_RST=0;	    //复位
	delay_ms(10);
 	GT_RST=1;	   //释放复位		    
	delay_ms(10); 
	
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_NONE);//PB1设置为浮空输入
	
	delay_ms(100);  
	GT911_RD_Reg(GT911_PID_REG,temp,4);//读取产品ID
	temp[4]=0;
	
	printf("CTP ID:%s\r\n",temp);	 //打印ID
	if(strcmp((char*)temp,"911")==0)//ID==911
	{
			temp[0]=0X02;			
			GT911_WR_Reg(GT911_CTRL_REG,temp,1);//软复位GT911
			GT911_RD_Reg(GT911_CFGS_REG,temp,1);//读取GT_CFGS_REG寄存器
			printf("Default Ver:%d\r\n",temp[0]);
		//if(temp[0]<0X60)//默认版本比较低,需要更新flash配置
		//if(temp[0]<0x60)//默认版本比较低,需要更新flash配置
		/*
		{
			printf("Default Ver:%d\r\n",temp[0]);
			GT911_Send_Cfg(1);//更新并保存配置
		}
		*/
			//GT911_RD_Reg(GT_CFGS_REG,temp,1);
			//printf("Default Ver:%d\r\n",temp[0]);
		
		delay_ms(10);
		temp[0]=0X00;	 
		GT911_WR_Reg(GT911_CTRL_REG,temp,1);//结束复位   
		return 0;
		//printf(" CTP ID:%s\r\n",temp);
	} 
	return 1;
}
const u16 GT911_TPX_TBL1[5]=
{GT911_TP1_REG,GT911_TP2_REG,GT911_TP3_REG,GT911_TP4_REG,GT911_TP5_REG,};
//扫描触摸屏(采用查询方式)
//mode:0,正常扫描.
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
u8 GT911_Scan(u8 mode)
{
	u8 buf[4];
	u8 i=0;
	u8 res=0;
	u16 temp;
	u16 tempsta;
 	static u8 t=0;//控制查询间隔,从而降低CPU占用率   
	t++;
	if((t%10)==0||t<10)//空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率
	{
		GT911_RD_Reg(GT911_GSTID_REG,&mode,1);	//读取触摸点的状态  
 		if(mode&0X80&&((mode&0XF)<11))
		{
			i=0;
			GT911_WR_Reg(GT911_GSTID_REG,&i,1);	//清标志 		
		}		
		if((mode&0XF)&&((mode&0XF)<11))
		{
			temp=0XFFFF<<(mode&0XF);	//将点的个数转换为1的位数,匹配tp_dev.sta定义 
			tempsta=tp_dev.sta;			//保存当前的tp_dev.sta值
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];	//保存触点0的数据
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<10;i++)
			{
				if(tp_dev.sta&(1<<i))	//触摸有效?
				{
					GT911_RD_Reg(GT911_TPX_TBL1[i],buf,6);	//读取XY坐标值
									
					tp_dev.x[i]=((u16)buf[1]<<8)+buf[0];
					tp_dev.y[i]=((u16)buf[3]<<8)+buf[2];
				
	//				if((tp_dev.x[i]>0)&&(tp_dev.x[i]<LCD_XSIZE_TFT)&&(tp_dev.y[i]>0)&&(tp_dev.y[i]<LCD_YSIZE_TFT))
	//				printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]); 
				}			
			} 
			res=1;
			if(tp_dev.x[0]>lcddev.width||tp_dev.y[0]>lcddev.height)//非法数据(坐标超出了)
			{ 
				if((mode&0XF)>1)		//有其他点有数据,则复第二个触点的数据到第一个触点.
				{
					tp_dev.x[0]=tp_dev.x[1];
					tp_dev.y[0]=tp_dev.y[1];
					t=0;				//触发一次,则会最少连续监测10次,从而提高命中率
				}else					//非法数据,则忽略此次数据(还原原来的)  
				{
					tp_dev.x[0]=tp_dev.x[4];
					tp_dev.y[0]=tp_dev.y[4];
					mode=0X80;		
					tp_dev.sta=tempsta;	//恢复tp_dev.sta
				}
			}else t=0;					//触发一次,则会最少连续监测10次,从而提高命中率
		}
	}
	if((mode&0X8F)==0X80)//无触摸点按下
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)		//之前是被按下的
		{
			tp_dev.sta&=~TP_PRES_DOWN;	//标记按键松开
		}else							//之前就没有被按下
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE000;			//清除点有效标记	
		}	 
	} 	
	if(t>240)t=10;//重新从10开始计数
	return res;
}
 



























