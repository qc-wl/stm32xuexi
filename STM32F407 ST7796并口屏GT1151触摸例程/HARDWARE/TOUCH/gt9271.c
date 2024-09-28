#include "gt9271.h"
#include "touch.h"
//#include "LT738_Lib.h"
#include "ctiic.h"
#include "usart.h"
#include "delay.h" 
#include "string.h" 


//GT9271配置参数表
//第一个字节为版本号(0X41),必须保证新的版本号大于等于GT9271内部
//flash原有版本号,才会更新配置.
//分辨率1280*800

//0x8047~0x8100  186个寄存器
//0x8047~0x0x80FE   184个寄存器为配置区
//0x80FF为校验和
//0x8100 Flash更新标记

const u8 GT9271_CFG_TBL[]=
{ 
	0x42,0x00,0x04,0x58,0x02,0x0A,0x3D,0x20,0x01,0x0A,
	0x28,0x0F,0x6E,0x5A,0x03,0x05,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x18,0x1A,0x1E,0x14,0x8F,0x2F,0xAA,
	0x26,0x24,0x0C,0x08,0x00,0x00,0x00,0x81,0x03,0x2D,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x1A,0x3C,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
	0x9E,0x1C,0x00,0x89,0x21,0x00,0x77,0x27,0x00,0x68,
	0x2E,0x00,0x5B,0x37,0x00,0x5B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x19,0x18,0x17,0x16,0x15,0x14,0x11,0x10,
	0x0F,0x0E,0x0D,0x0C,0x09,0x08,0x07,0x06,0x05,0x04,
	0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x02,0x04,0x06,0x07,0x08,0x0A,0x0C,
	0x0D,0x0F,0x10,0x11,0x12,0x13,0x14,0x19,0x1B,0x1C,
	0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
	0x28,0x29,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x26,0x01
};  

/*

//发送GT9271配置参数
//mode:0,参数不保存到flash
//     1,参数保存到flash
u8 GT9271_Send_Cfg(u8 mode)
{
	u8 buf[2];
	u8 i=0;
	buf[0]=0;
	buf[1]=mode;	//是否写入到GT9271 FLASH?  即是否掉电保存
	for(i=0;i<sizeof(GT9271_CFG_TBL);i++)buf[0]+=GT9271_CFG_TBL[i];//计算校验和
    buf[0]=(~buf[0])+1;
	GT9271_WR_Reg(GT9271_CFGS_REG,(u8*)GT9271_CFG_TBL,sizeof(GT9271_CFG_TBL));//发送寄存器配置
	GT9271_WR_Reg(GT9271_CHECK_REG,buf,2);//写入校验和,和配置更新标记
	return 0;
} 

*/


u8 GT9271_Send_Cfg(u8 mode)
{
	u8 buf[2];
	u8 i=0;
	buf[0]=0;
	buf[1]=mode;	//是否写入到GT9271 FLASH?  即是否掉电保存
	for(i=0;i<(sizeof(GT9271_CFG_TBL)-2);i++)buf[0]+=GT9271_CFG_TBL[i];//计算校验和
  buf[0]=(~buf[0])+1;
	//printf("Config_Chksum:0x%x\r\n",buf[0]);
	GT9271_WR_Reg(GT9271_CFGS_REG,(u8*)GT9271_CFG_TBL,sizeof(GT9271_CFG_TBL));//发送寄存器配置
	//GT9271_WR_Reg(GT9271_CHECK_REG,buf,2);//写入校验和,和配置更新标记
	return 0;
} 


//向GT9271写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
u8 GT9271_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT9271_CMD_WR);   	//发送写命令 	 
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
//从GT9271读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void GT9271_RD_Reg(u16 reg,u8 *buf,u8 len)
{
		u8 i; 
		CT_IIC_Start();	
		CT_IIC_Send_Byte(GT9271_CMD_WR);   //发送写命令 	 
		CT_IIC_Wait_Ack();
		CT_IIC_Send_Byte(reg>>8);   	//发送高8位地址
		CT_IIC_Wait_Ack(); 	 										  		   
		CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
		CT_IIC_Wait_Ack();  
		CT_IIC_Start();  	 	   
		CT_IIC_Send_Byte(GT9271_CMD_RD);   //发送读命令		   
		CT_IIC_Wait_Ack();	   
		for(i=0;i<len;i++)
		{	   
				buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
		} 
			CT_IIC_Stop();//产生一个停止条件    
		} 
		//初始化GT9271触摸屏
		//返回值:0,初始化成功;1,初始化失败 
u8 GT9271_Init(void)
	{
			u8 temp[5]; 
  RCC->AHB1ENR|=1<<1;    		//使能PORTB时钟 
	RCC->AHB1ENR|=1<<2;    		//使能PORTC时钟  
	GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU); 	//PB1设置为上拉输入
	GPIO_Set(GPIOC,PIN13,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PC13设置为推挽输出
	
	
	CT_IIC_Init();  //初始化电容屏的I2C总线  
	GT9271_RST=0;	    //复位
	delay_ms(10);
 	GT9271_RST=1;	   //释放复位		    
	delay_ms(10); 
	
  GPIO_Set(GPIOB,PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_NONE);//PB1设置为浮空输入
			delay_ms(100);  
			GT9271_RD_Reg(GT9271_PID_REG,temp,4);//读取产品ID
			temp[4]=0;
			printf("CTP ID:GT%s\r\n",temp);	//打印ID
			GT9271_RD_Reg(0x8047,temp,1);
			printf("固件版本:0x%x\r\n",temp[0]);
			
		  if(temp[0]<0x43)//默认版本比较低,需要更新flash配置
			{
				GT9271_Send_Cfg(1);//更新并保存配置
			}

			
			GT9271_RD_Reg(0x80FF,temp,1);
			printf("Config_Chksum:0x%x\r\n\r\n",temp[0]);
/*
			GT9271_RD_Reg(0x8048,temp,2);
			printf("X分辨率:%d\r\n\r\n",(temp[1]<<8|temp[0]));

			GT9271_RD_Reg(0x804A,temp,2);
			printf("Y分辨率:%d\r\n\r\n",(temp[1]<<8|temp[0]));

			GT9271_RD_Reg(0x804D,temp,1);//
			printf("INT_trigger:0x%x\r\n\r\n",temp[0]);

			GT9271_RD_Reg(0x80FF,temp,1);//
			printf("Config_Chksum:0x%x\r\n\r\n",temp[0]);
			
		  //GT9271_RD_Reg(0x8100,temp,1);//
			//printf("Config_Flash:0x%x\r\n",temp[0]);
	*/
	

			
			return 0;

}

const u16 GT9271_TPX_TBL[5]={GT9271_TP1_REG,GT9271_TP2_REG,GT9271_TP3_REG,GT9271_TP4_REG,GT9271_TP5_REG};
//扫描触摸屏(采用查询方式)
//mode:0,正常扫描.
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
u8 GT9271_Scan(u8 mode)
{
	u8 buf[4];
	u8 i=0;
	u8 res=0;
	u8 temp;
	u8 tempsta;
 	static u8 t=0;//控制查询间隔,从而降低CPU占用率   
	t++;
	if((t%10)==0||t<10)//空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率
	{
		GT9271_RD_Reg(GT9271_GSTID_REG,&mode,1);	//读取触摸点的状态  
 		if(mode&0X80&&((mode&0XF)<6))
		{
			temp=0;
			GT9271_WR_Reg(GT9271_GSTID_REG,&temp,1);//清标志 		
		}		
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);	//将点的个数转换为1的位数,匹配tp_dev.sta定义 
			tempsta=tp_dev.sta;			//保存当前的tp_dev.sta值
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];//保存触点0的数据
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<5;i++)
			{
				if(tp_dev.sta&(1<<i))	//触摸有效?
				{
					GT9271_RD_Reg(GT9271_TPX_TBL[i],buf,4);	//读取XY坐标值
					tp_dev.x[i]=(((u16)buf[1]<<8)+buf[0]);
					tp_dev.y[i]=(((u16)buf[3]<<8)+buf[2]);
					printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]);
				}			
			} 
			res=1;
			if(tp_dev.x[0]>LCD_XSIZE_TFT||tp_dev.y[0]>LCD_YSIZE_TFT)//非法数据(坐标超出了)
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
		if(tp_dev.sta&TP_PRES_DOWN)	//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);	//标记按键松开
		}else						//之前就没有被按下
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//清除点有效标记	
		}	 
	} 	
	if(t>240)t=10;//重新从10开始计数
	return res;
}
 



























