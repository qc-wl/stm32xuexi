#include "led.h" 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6核心板
//LED驱动代码	   
//版本：V1.0				  							  
////////////////////////////////////////////////////////////////////////////////// 	 

//初始化PA1和PA2为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{    	 
	RCC->AHB1ENR|=1<<0;//使能PORTA时钟 
	GPIO_Set(GPIOA,PIN1|PIN2,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA1,PA2设置
	LED0=1;//LED0关闭
	LED1=1;//LED1关闭
}






