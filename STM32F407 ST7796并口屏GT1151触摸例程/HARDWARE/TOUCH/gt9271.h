#ifndef __GT9271_H
#define __GT9271_H	
#include "sys.h"	


//IO��������	 
#define GT9271_RST    		PCout(13)	//GT5668��λ����
#define GT9271_INT    		PBin(1)		//GT5668�ж�����	
 
//I2C��д����	
#define GT9271_CMD_WR 		0X28    //д����
#define GT9271_CMD_RD 		0X29		//������
  
//GT9271 ���ּĴ������� 
#define GT9271_CTRL_REG 	0X8040   	//GT9271���ƼĴ���
#define GT9271_CFGS_REG 	0X8047   	//GT9271������ʼ��ַ�Ĵ���
#define GT9271_CHECK_REG 	0X80FF   	//GT9271У��ͼĴ���
#define GT9271_PID_REG 		0X8140   	//GT9271��ƷID�Ĵ���

#define GT9271_GSTID_REG 	0X814E   	//GT9271��ǰ��⵽�Ĵ������
#define GT9271_TP1_REG 		0X8150  	//��һ�����������ݵ�ַ
#define GT9271_TP2_REG 		0X8158		//�ڶ������������ݵ�ַ
#define GT9271_TP3_REG 		0X8160		//���������������ݵ�ַ
#define GT9271_TP4_REG 		0X8168		//���ĸ����������ݵ�ַ
#define GT9271_TP5_REG 		0X8170		//��������������ݵ�ַ  
 
 
u8 GT9271_Send_Cfg(u8 mode);
u8 GT9271_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT9271_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT9271_Init(void);
u8 GT9271_Scan(u8 mode); 
#endif













