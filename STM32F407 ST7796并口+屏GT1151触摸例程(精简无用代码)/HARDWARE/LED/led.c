#include "led.h" 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//WKS STM32F407VET6���İ�
//LED��������	   
//�汾��V1.0				  							  
////////////////////////////////////////////////////////////////////////////////// 	 

//��ʼ��PA1��PA2Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
	RCC->AHB1ENR|=1<<0;//ʹ��PORTAʱ�� 
	GPIO_Set(GPIOA,PIN1|PIN2,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA1,PA2����
	LED0=1;//LED0�ر�
	LED1=1;//LED1�ر�
}






