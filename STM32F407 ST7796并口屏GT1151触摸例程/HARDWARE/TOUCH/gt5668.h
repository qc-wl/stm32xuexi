#ifndef __GT5668_H
#define __GT5668_H	
#include "sys.h"	


//IO��������	 
#define GT_RST    		PCout(13)	//GT5668��λ����
#define GT_INT    		PBin(1)		//GT5668�ж�����	
   	
 
//I2C��д����	
#define GT_CMD_WR 		0X28    //д����
#define GT_CMD_RD 		0X29		//������
  
//GT5668 ���ּĴ������� 
#define GT_CTRL_REG 	0X8040   	//GT5668���ƼĴ���
//#define GT_CFGS_REG 	0X8050   	//GT5668������ʼ��ַ�Ĵ���
//#define GT_CHECK_REG 	0X813C   	//GT5668У��ͼĴ���
#define GT_PID_REG 		0X8140   	//GT5668��ƷID�Ĵ���

#define GT_GSTID_REG 	0X814E   	//GT5668��ǰ��⵽�Ĵ������
#define GT_TP1_REG 		0X8150  	//��һ�����������ݵ�ַ
#define GT_TP2_REG 		0X8158		//�ڶ������������ݵ�ַ
#define GT_TP3_REG 		0X8160		//���������������ݵ�ַ
#define GT_TP4_REG 		0X8168		//���ĸ����������ݵ�ַ
#define GT_TP5_REG 		0X8170		//��������������ݵ�ַ  
 
 
u8 GT5668_Send_Cfg(u8 mode);
u8 GT5668_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT5668_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT5668_Init(void);
u8 GT5668_Scan(u8 mode); 
#endif













