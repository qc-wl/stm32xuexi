#ifndef __GT911_H
#define __GT911_H	
#include "sys.h"	
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6核心板
//电容触摸屏-GT911 驱动代码	   
//版本：V1.0											  
////////////////////////////////////////////////////////////////////////////////// 


//IO操作函数	 
#define GT_RST    		PCout(13)	//GT911复位引脚
#define GT_INT    		PBin(1)		//GT911中断引脚	
   	
 
//I2C读写命令	
#define GT911_CMD_WR 		0X28    	//写命令
#define GT911_CMD_RD 		0X29		//读命令
  
//GT911 部分寄存器定义 
#define GT911_CTRL_REG 		0X8040   	//GT911控制寄存器
#define GT911_CFGS_REG 		0X8047   	//GT911配置起始地址寄存器
#define GT911_CHECK_REG 	0X80FF   	//GT911校验和寄存器
#define GT911_PID_REG 		0X8140   	//GT911产品ID寄存器

#define GT911_GSTID_REG 	0X814E   	//GT911当前检测到的触摸情况
#define GT911_TP1_REG 		0X8150  	//第一个触摸点数据地址
#define GT911_TP2_REG 		0X8158		//第二个触摸点数据地址
#define GT911_TP3_REG 		0X8160		//第三个触摸点数据地址
#define GT911_TP4_REG 		0X8168		//第四个触摸点数据地址
#define GT911_TP5_REG 		0X8170		//第五个触摸点数据地址 
#define GT911_TP6_REG 		0X8178		//第六个触摸点数据地址 
#define GT911_TP7_REG 		0X8180		//第七个触摸点数据地址 
#define GT911_TP8_REG 		0X8188		//第八个触摸点数据地址 
#define GT911_TP9_REG 		0X8190		//第九个触摸点数据地址 
#define GT911_TP10_REG 		0X8198		//第十个触摸点数据地址  
 
 
u8 GT911_Send_Cfg(u8 mode);
u8 GT911_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT911_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT911_Init(void);
u8 GT911_Scan(u8 mode); 
#endif













