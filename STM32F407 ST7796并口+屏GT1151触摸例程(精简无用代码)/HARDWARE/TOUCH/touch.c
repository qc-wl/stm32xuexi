#include "touch.h" 
#include "lcd.h"
#include "delay.h"
#include "stdlib.h"
#include "math.h"


_m_tp_dev tp_dev=
{
	TP_Init,
	0,
	0,
	0,
	0, 
	0,
	0,
	0,
	0,	  	 		
	0,
	0,	  	 		
};					
//Ĭ��Ϊtouchtype=0������.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;
 	 			    					   

//��������ʼ��  		    
//����ֵ:0,û�н���У׼
//       1,���й�У׼
u8 TP_Init(void)
{	
	if(lcddev.id==0X7796||lcddev.id==0X5310)    //3.5����ݴ�����
	{
	 if(GT1151_Init()==0)          //�ж��ǵ��ݴ��������ǵ��败����
	 {tp_dev.scan=GT1151_Scan;    //ɨ�躯��ָ��GT1151������ɨ��
		tp_dev.touchtype|=0X80;     //������ 
		tp_dev.touchtype|=lcddev.dir&0X01;//������������
		return 0;
	 }
	}
	  return 1; 		
}

