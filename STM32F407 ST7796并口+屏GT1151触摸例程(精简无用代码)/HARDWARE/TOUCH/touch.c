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
//默认为touchtype=0的数据.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;
 	 			    					   

//触摸屏初始化  		    
//返回值:0,没有进行校准
//       1,进行过校准
u8 TP_Init(void)
{	
	if(lcddev.id==0X7796||lcddev.id==0X5310)    //3.5寸电容触摸屏
	{
	 if(GT1151_Init()==0)          //判断是电容触摸屏还是电阻触摸屏
	 {tp_dev.scan=GT1151_Scan;    //扫描函数指向GT1151触摸屏扫描
		tp_dev.touchtype|=0X80;     //电容屏 
		tp_dev.touchtype|=lcddev.dir&0X01;//横屏还是竖屏
		return 0;
	 }
	}
	  return 1; 		
}

