#ifndef __GT9271_H
#define __GT9271_H	
#include "sys.h"	


//IO操作函数	 
#define GT9271_RST    		PCout(13)	//GT5668复位引脚
#define GT9271_INT    		PBin(1)		//GT5668中断引脚	
 
//I2C读写命令	
#define GT9271_CMD_WR 		0X28    //写命令
#define GT9271_CMD_RD 		0X29		//读命令
  
//GT9271 部分寄存器定义 
#define GT9271_CTRL_REG 	0X8040   	//GT9271控制寄存器
#define GT9271_CFGS_REG 	0X8047   	//GT9271配置起始地址寄存器
#define GT9271_CHECK_REG 	0X80FF   	//GT9271校验和寄存器
#define GT9271_PID_REG 		0X8140   	//GT9271产品ID寄存器

#define GT9271_GSTID_REG 	0X814E   	//GT9271当前检测到的触摸情况
#define GT9271_TP1_REG 		0X8150  	//第一个触摸点数据地址
#define GT9271_TP2_REG 		0X8158		//第二个触摸点数据地址
#define GT9271_TP3_REG 		0X8160		//第三个触摸点数据地址
#define GT9271_TP4_REG 		0X8168		//第四个触摸点数据地址
#define GT9271_TP5_REG 		0X8170		//第五个触摸点数据地址  
 
 
u8 GT9271_Send_Cfg(u8 mode);
u8 GT9271_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT9271_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT9271_Init(void);
u8 GT9271_Scan(u8 mode); 
#endif













