#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6���İ�
//����������������	   
//�汾��V1.0								  
////////////////////////////////////////////////////////////////////////////////// 	 


#define KEY0 		  PCin(0)   //PC0
#define WK_UP 		PAin(0)		//PA0 

#define KEY0_PRES 	1	//KEY0����
#define WKUP_PRES   2	//KEY_UP����(��WK_UP)

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��					    
#endif
