#ifndef __GT911_H
#define __GT911_H	
#include "sys.h"	
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6���İ�
//���ݴ�����-GT911 ��������	   
//�汾��V1.0											  
////////////////////////////////////////////////////////////////////////////////// 


//IO��������	 
#define GT_RST    		PCout(13)	//GT911��λ����
#define GT_INT    		PBin(1)		//GT911�ж�����	
   	
 
//I2C��д����	
#define GT911_CMD_WR 		0X28    	//д����
#define GT911_CMD_RD 		0X29		//������
  
//GT911 ���ּĴ������� 
#define GT911_CTRL_REG 		0X8040   	//GT911���ƼĴ���
#define GT911_CFGS_REG 		0X8047   	//GT911������ʼ��ַ�Ĵ���
#define GT911_CHECK_REG 	0X80FF   	//GT911У��ͼĴ���
#define GT911_PID_REG 		0X8140   	//GT911��ƷID�Ĵ���

#define GT911_GSTID_REG 	0X814E   	//GT911��ǰ��⵽�Ĵ������
#define GT911_TP1_REG 		0X8150  	//��һ�����������ݵ�ַ
#define GT911_TP2_REG 		0X8158		//�ڶ������������ݵ�ַ
#define GT911_TP3_REG 		0X8160		//���������������ݵ�ַ
#define GT911_TP4_REG 		0X8168		//���ĸ����������ݵ�ַ
#define GT911_TP5_REG 		0X8170		//��������������ݵ�ַ 
#define GT911_TP6_REG 		0X8178		//���������������ݵ�ַ 
#define GT911_TP7_REG 		0X8180		//���߸����������ݵ�ַ 
#define GT911_TP8_REG 		0X8188		//�ڰ˸����������ݵ�ַ 
#define GT911_TP9_REG 		0X8190		//�ھŸ����������ݵ�ַ 
#define GT911_TP10_REG 		0X8198		//��ʮ�����������ݵ�ַ  
 
 
u8 GT911_Send_Cfg(u8 mode);
u8 GT911_WR_Reg(u16 reg,u8 *buf,u8 len);
void GT911_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 GT911_Init(void);
u8 GT911_Scan(u8 mode); 
#endif













