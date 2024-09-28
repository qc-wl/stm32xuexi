#ifndef __GT1151_H
#define __GT1151_H	
#include "sys.h"	
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6核心板
//电容触摸屏-GT1151 驱动代码	   
//版本：V1.0								  
////////////////////////////////////////////////////////////////////////////////// 


//IO操作函数	 
#define GT_RST    		PBout(14)	  //GT1151复位引脚
#define GT_INT    		PBin(13)		//GT1151断引脚	
   	
 
//I2C读写命令	
#define GT_CMD_WR 		0X28    	//写命令
#define GT_CMD_RD 		0X29		//读命令
  
//GT1151 部分寄存器定义 
#define GT_CTRL_REG 	0X8040   	//GT1151控制寄存器
#define GT_CFGS_REG 	0X8050   	//GT1151地址寄存器
#define GT_CHECK_REG 	0X813C   	//GT1151验和寄存器
#define GT_PID_REG 		0X8140   	//GT1151产品ID寄存器

#define GT_GSTID_REG 	0X814E   	//GT1151前检测到的触摸情况
#define GT_TP1_REG 		0X8150  	//第一个触摸点数据地址
#define GT_TP2_REG 		0X8158		//第二个触摸点数据地址
#define GT_TP3_REG 		0X8160		//第三个触摸点数据地址
#define GT_TP4_REG 		0X8168		//第四个触摸点数据地址
#define GT_TP5_REG 		0X8170		//第五个触摸点数据地址  
 
 
u8 GT1151_Send_Cfg(u8 mode);
u8 GT1151_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT1151_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT1151_Init(void);
u8 GT1151_Scan(u8 mode); 
#endif













