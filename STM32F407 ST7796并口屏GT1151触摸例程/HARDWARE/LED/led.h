#ifndef __LED_H
#define __LED_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6核心板
//LED驱动代码	   
//版本：V1.0							  
////////////////////////////////////////////////////////////////////////////////// 	

//LED端口定义
#define LED0 PAout(1)	// LED0
#define LED1 PAout(2)	// LED1	 

void LED_Init(void);//LED初始化		 				    
#endif

















