#include "lcd.h"
#include "stdlib.h"
#include "font.h" 
#include "usart.h"	 
#include "delay.h"	 


//LCD�Ļ�����ɫ�ͱ���ɫ	   
u16 POINT_COLOR=0x0000;	//������ɫ
u16 BACK_COLOR=0xFFFF;  //����ɫ 
  
//����LCD��Ҫ����
//Ĭ��Ϊ����
_lcd_dev lcddev;
	 
//д�Ĵ�������
//regval:�Ĵ���ֵ
void LCD_WR_REG(vu16 regval)
{   
	regval=regval;		//ʹ��-O2�Ż���ʱ��,����������ʱ
	LCD->LCD_REG=regval;//д��Ҫд�ļĴ������	 
}
//дLCD����
//data:Ҫд���ֵ
void LCD_WR_DATA(vu16 data)
{	  
	data=data;			//ʹ��-O2�Ż���ʱ��,����������ʱ
	LCD->LCD_RAM=data;		 
}
//��LCD����
//����ֵ:������ֵ
u16 LCD_RD_DATA(void)
{
	vu16 ram;			//��ֹ���Ż�
	ram=LCD->LCD_RAM;	
	return ram;	 
}					   
//д�Ĵ���
//LCD_Reg:�Ĵ�����ַ
//LCD_RegValue:Ҫд�������
void LCD_WriteReg(u16 LCD_Reg,u16 LCD_RegValue)
{	
	LCD->LCD_REG = LCD_Reg;		//д��Ҫд�ļĴ������	 
	LCD->LCD_RAM = LCD_RegValue;//д������	    		 
}	   
//���Ĵ���
//LCD_Reg:�Ĵ�����ַ
//����ֵ:����������
u16 LCD_ReadReg(u16 LCD_Reg)
{										   
	LCD_WR_REG(LCD_Reg);		//д��Ҫ���ļĴ������
	delay_us(5);		  
	return LCD_RD_DATA();		//���ض�����ֵ
}   
//��ʼдGRAM
void LCD_WriteRAM_Prepare(void)
{
 	LCD->LCD_REG=lcddev.wramcmd;	  
}	 
//LCDдGRAM
//RGB_Code:��ɫֵ
void LCD_WriteRAM(u16 RGB_Code)
{							    
	LCD->LCD_RAM = RGB_Code;//дʮ��λGRAM
}

//ͨ���ú���ת��GBR��ʽΪRGB��ʽ
//c:GBR��ʽ����ɫֵ
//����ֵ��RGB��ʽ����ɫֵ
u16 LCD_BGR2RGB(u16 c)
{
	u16  r,g,b,rgb;   
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;	 
	rgb=(b<<11)+(g<<5)+(r<<0);		 
	return(rgb);
} 
//��mdk -O1ʱ���Ż�ʱ��Ҫ����
//��ʱi
void opt_delay(u8 i)
{
	while(i--);
}
//��ȡ��ĳ�����ɫֵ	 
//x,y:����
//����ֵ:�˵����ɫ
u16 LCD_ReadPoint(u16 x,u16 y)
{
    u16 r, g, b;

    if (x >= lcddev.width || y >= lcddev.height)return 0;   //�����˷�Χ,ֱ�ӷ���

    LCD_SetCursor(x, y);

    if (lcddev.id == 0X5510)    //5510 ���Ͷ�GRAMָ��
    {
        LCD_WR_REG(0X2E00);
    }
    else                        //����IC(7796/5310/7789)���Ͷ�GRAMָ��
    {
        LCD_WR_REG(0X2E);
    }

    r = LCD_RD_DATA();          //�ٶ�


    r = LCD_RD_DATA();          //ʵ��������ɫ

    //7796/5310/5510/7789 Ҫ��2�ζ���
    b = LCD_RD_DATA();
    g = r & 0XFF;               //����7796/5310/5510/7789,��һ�ζ�ȡ����RG��ֵ,R��ǰ,G�ں�,��ռ8λ
    g <<= 8;
		if (lcddev.id == 0X7796)    //5510 ���Ͷ�GRAMָ��
    {
        return  r;
    }
    return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));  // 7796/5310/5510/7789��Ҫ��ʽת��һ��
}			 

//LCD������ʾ
void LCD_DisplayOn(void)
{					   
    if (lcddev.id == 0X5510)    //5510������ʾָ��
    {
        LCD_WR_REG(0X2900);     //������ʾ
    }
    else                        //7796/5310/7789���Ϳ�����ʾָ��
    {
        LCD_WR_REG(0X29);       //������ʾ
    }
}
//LCD�ر���ʾ
void LCD_DisplayOff(void)
{	   
    if (lcddev.id == 0X5510)    //5510�ر���ʾָ��
    {
        LCD_WR_REG(0X2800);     //�ر���ʾ
    }
    else                        //7796/5310/7789���͹ر���ʾָ��
    {
        LCD_WR_REG(0X28);       //�ر���ʾ
    }
}
//���ù��λ��
//Xpos:������
//Ypos:������
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
    if (lcddev.id == 0X5510)   //5510��������
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(Xpos >> 8);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(Xpos & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(Ypos >> 8);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(Ypos & 0XFF);
    }
    else     //7796/5310/7789��������
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(Xpos >> 8);
        LCD_WR_DATA(Xpos & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(Ypos >> 8);
        LCD_WR_DATA(Ypos & 0XFF);
    }
}

//����LCD���Զ�ɨ�跽��
//dir:0~7,����8������(���嶨���lcd.h)
//7796/5310/5510/7789��IC�Ѿ�ʵ�ʲ���
//ע��:�����������ܻ��ܵ��˺������õ�Ӱ��,
//����,һ������ΪL2R_U2D����,�������Ϊ����ɨ�跽ʽ,���ܵ�����ʾ������.   	   
void LCD_Scan_Dir(u8 dir)
{
    u16 regval = 0;
    u16 dirreg = 0;
    u16 temp;

    //����ʱ��IC�ı�ɨ�跽��
    if (lcddev.dir == 1 )
    {
        switch (dir)   //����ת��
        {
            case 0:
                dir = 6;
                break;

            case 1:
                dir = 7;
                break;

            case 2:
                dir = 4;
                break;

            case 3:
                dir = 5;
                break;

            case 4:
                dir = 1;
                break;

            case 5:
                dir = 0;
                break;

            case 6:
                dir = 3;
                break;

            case 7:
                dir = 2;
                break;
        }
    }

    switch (dir)
    {
        case L2R_U2D://������,���ϵ���
            regval |= (0 << 7) | (0 << 6) | (0 << 5);
            break;

        case L2R_D2U://������,���µ���
            regval |= (1 << 7) | (0 << 6) | (0 << 5);
            break;

        case R2L_U2D://���ҵ���,���ϵ���
            regval |= (0 << 7) | (1 << 6) | (0 << 5);
            break;

        case R2L_D2U://���ҵ���,���µ���
            regval |= (1 << 7) | (1 << 6) | (0 << 5);
            break;

        case U2D_L2R://���ϵ���,������
            regval |= (0 << 7) | (0 << 6) | (1 << 5);
            break;

        case U2D_R2L://���ϵ���,���ҵ���
            regval |= (0 << 7) | (1 << 6) | (1 << 5);
            break;

        case D2U_L2R://���µ���,������
            regval |= (1 << 7) | (0 << 6) | (1 << 5);
            break;

        case D2U_R2L://���µ���,���ҵ���
            regval |= (1 << 7) | (1 << 6) | (1 << 5);
            break;
    }

    if (lcddev.id == 0X5510)dirreg = 0X3600;
    else dirreg = 0X36;

    if (lcddev.id == 0X7796)   //7796 & 7789 Ҫ����BGRλ
    {
        regval |= 0X08;
    }

    LCD_WriteReg(dirreg, regval);

    
        if (regval & 0X20)
        {
            if (lcddev.width < lcddev.height)   //����X,Y
            {
                temp = lcddev.width;
                lcddev.width = lcddev.height;
                lcddev.height = temp;
            }
        }
        else
        {
            if (lcddev.width > lcddev.height)   //����X,Y
            {
                temp = lcddev.width;
                lcddev.width = lcddev.height;
                lcddev.height = temp;
            }
        }
    

    //������ʾ����(����)��С
    if (lcddev.id == 0X5510)
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setxcmd + 2);
        LCD_WR_DATA((lcddev.width - 1) >> 8);
        LCD_WR_REG(lcddev.setxcmd + 3);
        LCD_WR_DATA((lcddev.width - 1) & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setycmd + 2);
        LCD_WR_DATA((lcddev.height - 1) >> 8);
        LCD_WR_REG(lcddev.setycmd + 3);
        LCD_WR_DATA((lcddev.height - 1) & 0XFF);
    }
    else
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(0);
        LCD_WR_DATA(0);
        LCD_WR_DATA((lcddev.width - 1) >> 8);
        LCD_WR_DATA((lcddev.width - 1) & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(0);
        LCD_WR_DATA(0);
        LCD_WR_DATA((lcddev.height - 1) >> 8);
        LCD_WR_DATA((lcddev.height - 1) & 0XFF);
    }
}

//����
//x,y:����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);		//���ù��λ�� 
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	if (lcddev.id == 0X7789)
  {
      LCD->LCD_RAM = POINT_COLOR >> 8;
      LCD->LCD_RAM = POINT_COLOR;
  }
  else
  {
      LCD->LCD_RAM = POINT_COLOR;
  }
}
//���ٻ���
//x,y:����
//color:��ɫ
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
{	   
 if (lcddev.id == 0X5510)
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(x >> 8);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(x & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(y >> 8);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(y & 0XFF);
    }
    else     //7796/5310/7789����������
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(x >> 8);
        LCD_WR_DATA(x & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(y >> 8);
        LCD_WR_DATA(y & 0XFF);
    }

    LCD->LCD_REG=lcddev.wramcmd; 
    if (lcddev.id == 0X7789)
    {
        LCD->LCD_RAM = color >> 8;
        LCD->LCD_RAM = color;
    }
    else
    {
        LCD->LCD_RAM = color;
    }  
}


//����LCD��ʾ����
//dir:0,������1,����
void LCD_Display_Dir(u8 dir)
{
    lcddev.dir = dir;       //����/����

    if (dir == 0)           //����
    {
        lcddev.width = 240;
        lcddev.height = 320;

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;
            lcddev.setxcmd = 0X2A00;
            lcddev.setycmd = 0X2B00;
            lcddev.width = 480;
            lcddev.height = 800;
        }
        else                        //����IC, ����: 7796 / 5310 / 7789��IC
        {
            lcddev.wramcmd = 0X2C;
            lcddev.setxcmd = 0X2A;
            lcddev.setycmd = 0X2B;
        }

        if (lcddev.id == 0X5310||lcddev.id == 0X7796)    //�����5310/7796 ���ʾ�� 320*480�ֱ���
        {
            lcddev.width = 320;
            lcddev.height = 480;
        }
    }
    else     //����
    {
        lcddev.width = 320;
        lcddev.height = 240;

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;
            lcddev.setxcmd = 0X2A00;
            lcddev.setycmd = 0X2B00;
            lcddev.width = 800;
            lcddev.height = 480;
        }
        else                        //����IC, ����: 7796 / 5310 / 7789��IC
        {
            lcddev.wramcmd = 0X2C;
            lcddev.setxcmd = 0X2A;
            lcddev.setycmd = 0X2B;
        }

        if (lcddev.id == 0X5310||lcddev.id == 0X7796)    //�����5310/7796 ���ʾ�� 320*480�ֱ���
        {
            lcddev.width = 480;
            lcddev.height = 320;
        }
    }

    LCD_Scan_Dir(DFT_SCAN_DIR);     //Ĭ��ɨ�跽��
}

//���ô���,���Զ����û������굽�������Ͻ�(sx,sy).
//sx,sy:������ʼ����(���Ͻ�)
//width,height:���ڿ�Ⱥ͸߶�,�������0!!
//�����С:width*height.
void LCD_Set_Window(u16 sx, u16 sy, u16 width, u16 height)
{
    u16 twidth, theight;
    twidth = sx + width - 1;
    theight = sy + height - 1;

     if (lcddev.id == 0X5510)
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(sx >> 8);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(sx & 0XFF);
        LCD_WR_REG(lcddev.setxcmd + 2);
        LCD_WR_DATA(twidth >> 8);
        LCD_WR_REG(lcddev.setxcmd + 3);
        LCD_WR_DATA(twidth & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(sy >> 8);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(sy & 0XFF);
        LCD_WR_REG(lcddev.setycmd + 2);
        LCD_WR_DATA(theight >> 8);
        LCD_WR_REG(lcddev.setycmd + 3);
        LCD_WR_DATA(theight & 0XFF);
    }
    else     //7796/5310/7789���������ô���
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(sx >> 8);
        LCD_WR_DATA(sx & 0XFF);
        LCD_WR_DATA(twidth >> 8);
        LCD_WR_DATA(twidth & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(sy >> 8);
        LCD_WR_DATA(sy & 0XFF);
        LCD_WR_DATA(theight >> 8);
        LCD_WR_DATA(theight & 0XFF);
    }
}

//��ʼ��lcd
//�ó�ʼ���������Գ�ʼ������LCDҺ����
//������ռ�ýϴ�flash,�û����Ը����Լ���ʵ�����,ɾ��δ�õ���LCD��ʼ������.�Խ�ʡ�ռ�.
void LCD_Init(void)
{ 	 
	RCC->AHB1ENR|=0X3<<3;    	//ʹ��PD,PEʱ��
	RCC->AHB3ENR|=1<<0;     	//ʹ��FSMCʱ��  
	
	GPIO_Set(GPIOD,PIN12,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);							//PD12 �������,���Ʊ���
	GPIO_Set(GPIOD,PIN11,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);							//PD11 �������,LCD��λ����
	
	GPIO_Set(GPIOD,(3<<0)|(3<<4)|(15<<7)|(7<<13),GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);	//PD0,1,4,5,7,8,9,10,13,14,15 AF OUT
	GPIO_Set(GPIOE,(0X1FF<<7),GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);						          //PE7~15,AF OUT

 	GPIO_AF_Set(GPIOD,0,12);	//PD0,AF12
 	GPIO_AF_Set(GPIOD,1,12);	//PD1,AF12
 	GPIO_AF_Set(GPIOD,4,12);	//PD4,AF12
 	GPIO_AF_Set(GPIOD,5,12);	//PD5,AF12 
	GPIO_AF_Set(GPIOD,7,12);	//PD7,AF12
 	GPIO_AF_Set(GPIOD,8,12);	//PD8,AF12
 	GPIO_AF_Set(GPIOD,9,12);	//PD9,AF12
 	GPIO_AF_Set(GPIOD,10,12);	//PD10,AF12 
	GPIO_AF_Set(GPIOD,13,12);	//PD13,AF12
 	GPIO_AF_Set(GPIOD,14,12);	//PD14,AF12
 	GPIO_AF_Set(GPIOD,15,12);	//PD15,AF12
	
 	GPIO_AF_Set(GPIOE,7,12);	//PE7,AF12
 	GPIO_AF_Set(GPIOE,8,12);	//PE8,AF12
 	GPIO_AF_Set(GPIOE,9,12);	//PE9,AF12
 	GPIO_AF_Set(GPIOE,10,12);	//PE10,AF12
 	GPIO_AF_Set(GPIOE,11,12);	//PE11,AF12
 	GPIO_AF_Set(GPIOE,12,12);	//PE12,AF12
 	GPIO_AF_Set(GPIOE,13,12);	//PE13,AF12
 	GPIO_AF_Set(GPIOE,14,12);	//PE14,AF12
 	GPIO_AF_Set(GPIOE,15,12);	//PE15,AF12
	 
	//�Ĵ�������
	//bank1��NE1~4,ÿһ����һ��BCR+TCR�������ܹ��˸��Ĵ�����
	//��������ʹ��NE1 ��Ҳ�Ͷ�ӦBTCR[0],[1]��				    
	FSMC_Bank1->BTCR[0]=0X00000000;
	FSMC_Bank1->BTCR[1]=0X00000000;
	FSMC_Bank1E->BWTR[0]=0X00000000;
	//����BCR�Ĵ���	ʹ���첽ģʽ
	FSMC_Bank1->BTCR[0]|=1<<12;		//�洢��дʹ��
	FSMC_Bank1->BTCR[0]|=1<<14;		//��дʹ�ò�ͬ��ʱ��
	FSMC_Bank1->BTCR[0]|=1<<4; 		//�洢�����ݿ��Ϊ16bit 	    
	//����BTR�Ĵ���	
	//��ʱ����ƼĴ��� 							    
	FSMC_Bank1->BTCR[1]|=0<<28;		//ģʽA 	 						  	 
	FSMC_Bank1->BTCR[1]|=0XF<<0; 	//��ַ����ʱ��(ADDSET)Ϊ15��HCLK 1/168M=6ns*15=90ns	
	//��ΪҺ������IC�Ķ����ݵ�ʱ���ٶȲ���̫�졣
	FSMC_Bank1->BTCR[1]|=60<<8;  	//���ݱ���ʱ��(DATAST)Ϊ60��HCLK	=6*60=360ns
	//дʱ����ƼĴ���  
	FSMC_Bank1E->BWTR[0]|=0<<28; 	//ģʽA 	 							    
	FSMC_Bank1E->BWTR[0]|=9<<0;		//��ַ����ʱ��(ADDSET)Ϊ9��HCLK=54ns
 	//9��HCLK��HCLK=168M��,ĳЩҺ������IC��д�ź���������Ҳ��50ns��  	 
	FSMC_Bank1E->BWTR[0]|=8<<8; 	//���ݱ���ʱ��(DATAST)Ϊ6ns*9��HCLK=54ns
	//ʹ��BANK1,����4
	FSMC_Bank1->BTCR[0]|=1<<0;		//ʹ��BANK1������4    
			 
 	delay_ms(50); // delay 50 ms 
  
	//LCD��λ
	LCD_RST=1;
	delay_ms(10);
	LCD_RST=0;
	delay_ms(50);
	LCD_RST=1; 
	delay_ms(200);   
	
	//��7796 ID�Ķ�ȡ		
	LCD_WR_REG(0XD3);				   
	lcddev.id=LCD_RD_DATA();	//dummy read 	
	lcddev.id=LCD_RD_DATA();	//����0X00
	lcddev.id=LCD_RD_DATA();   	//��ȡ77								   
	lcddev.id<<=8;
	lcddev.id|=LCD_RD_DATA();  	//��ȡ96 	   			   
	
  printf(" LCD ID:%x\r\n", lcddev.id); //��ӡLCD ID

    if (lcddev.id == 0X7796)    //7796��ʼ��
    {
			  LCD_WR_REG(0xF0);     
				LCD_WR_DATA(0xC3);   

				LCD_WR_REG(0xF0);     
				LCD_WR_DATA(0x96);   

				LCD_WR_REG(0x36);     
				LCD_WR_DATA(0x48);   

				LCD_WR_REG(0x3A);     
				LCD_WR_DATA(0x55);   

				LCD_WR_REG(0xB4);     //1-dot Inversion
				LCD_WR_DATA(0x01);   
						
				LCD_WR_REG(0xB6);     //
				LCD_WR_DATA(0x80); 
				LCD_WR_DATA(0x22);	
				LCD_WR_DATA(0x3B);			

				LCD_WR_REG(0xB7);     
				LCD_WR_DATA(0xC6);

				LCD_WR_REG(0xC0);     
				LCD_WR_DATA(0x80);   
				LCD_WR_DATA(0x16);   

				LCD_WR_REG(0xC1);     
				LCD_WR_DATA(0x19);   //18  //00

				LCD_WR_REG(0xC2);     
				LCD_WR_DATA(0xA7);   

				LCD_WR_REG(0xC5);     
				LCD_WR_DATA(0x16);   


				LCD_WR_REG(0xE8);     
				LCD_WR_DATA(0x40);
				LCD_WR_DATA(0x8A);
				LCD_WR_DATA(0x00);
				LCD_WR_DATA(0x00);
				LCD_WR_DATA(0x29);
				LCD_WR_DATA(0x19);
				LCD_WR_DATA(0xA5);
				LCD_WR_DATA(0x33);

				LCD_WR_REG(0xE0);     
				LCD_WR_DATA(0xF0);   
				LCD_WR_DATA(0x07);   
				LCD_WR_DATA(0x0D);   
				LCD_WR_DATA(0x04);   
				LCD_WR_DATA(0x05);   
				LCD_WR_DATA(0x14);   
				LCD_WR_DATA(0x36);   
				LCD_WR_DATA(0x54);   
				LCD_WR_DATA(0x4C);   
				LCD_WR_DATA(0x38);   
				LCD_WR_DATA(0x13);   
				LCD_WR_DATA(0x14);   
				LCD_WR_DATA(0x2E);   
				LCD_WR_DATA(0x34);   

				LCD_WR_REG(0xE1);     
				LCD_WR_DATA(0xF0);   
				LCD_WR_DATA(0x10);   
				LCD_WR_DATA(0x14);   
				LCD_WR_DATA(0x0E);   
				LCD_WR_DATA(0x0C);   
				LCD_WR_DATA(0x08);   
				LCD_WR_DATA(0x35);   
				LCD_WR_DATA(0x44);   
				LCD_WR_DATA(0x4C);   
				LCD_WR_DATA(0x26);   
				LCD_WR_DATA(0x10);   
				LCD_WR_DATA(0x12);   
				LCD_WR_DATA(0x2C);   
				LCD_WR_DATA(0x32); 

        LCD_WR_REG(0xF0);     
        LCD_WR_DATA(0x3C);   

        LCD_WR_REG(0xF0);     
        LCD_WR_DATA(0x69);   

        LCD_WR_REG(0x35);     
        LCD_WR_DATA(0x00); 

        LCD_WR_REG(0x21); 
 
        LCD_WR_REG(0x11);     

        delay_ms(120);                //ms

        LCD_WR_REG(0x29);     
        delay_ms(50); 

				LCD_WR_REG(0x2A);    //320 
				LCD_WR_DATA(0x00);   
				LCD_WR_DATA(0x00);   
				LCD_WR_DATA(0x01);   
        LCD_WR_DATA(0x3F);   

        LCD_WR_REG(0x2B);    //480
        LCD_WR_DATA(0x00);   
        LCD_WR_DATA(0x00);   
        LCD_WR_DATA(0x01);   
        LCD_WR_DATA(0xDF); 

        LCD_WR_REG(0x2C); 
						
    }
    

	//��ʼ������Ժ�,����
	if(lcddev.id==0X7796)//������⼸��IC,������WRʱ��Ϊ���
	{
		//��������дʱ����ƼĴ�����ʱ��   	 							    
		FSMC_Bank1E->BWTR[6]&=~(0XF<<0);//��ַ����ʱ��(ADDSET)���� 	 
		FSMC_Bank1E->BWTR[6]&=~(0XF<<8);//���ݱ���ʱ������
		FSMC_Bank1E->BWTR[6]|=3<<0;		//��ַ����ʱ��(ADDSET)Ϊ3��HCLK =18ns  	 
        if(lcddev.id == 0X7789)
        {
            FSMC_Bank1E->BWTR[6]|=4<<8; 	//���ݱ���ʱ��(DATAST)Ϊ6ns*5��HCLK=30ns
        }
        else
        {
            FSMC_Bank1E->BWTR[6]|=2<<8; 	//���ݱ���ʱ��(DATAST)Ϊ6ns*3��HCLK=18ns
        }
    }
	LCD_Display_Dir(0);		//Ĭ��Ϊ����
	LCD_LED=1;				//��������
	LCD_Clear(WHITE);
}  
//��������
//color:Ҫ���������ɫ
void LCD_Clear(u16 color)
{
    u32 index = 0;
    u32 totalpoint = lcddev.width;
    totalpoint *= lcddev.height;    //�õ��ܵ���

    LCD_SetCursor(0x00, 0x0000);    //���ù��λ��
    LCD_WriteRAM_Prepare();         //��ʼд��GRAM

    for (index = 0; index < totalpoint; index++)
    {
        if (lcddev.id == 0X7789)
        {
            LCD->LCD_RAM = color >> 8;
            LCD->LCD_RAM = color;
        }
        else
        {
            LCD->LCD_RAM = color;
        }
    }
}

//��ָ�����������ָ����ɫ
//�����С:(xend-xsta+1)*(yend-ysta+1)
//xsta
//color:Ҫ������ɫ
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color)
{
    u16 i, j;
    u16 xlen = 0;

    xlen = ex - sx + 1;

    for (i = sy; i <= ey; i++)
    {
        LCD_SetCursor(sx, i);       //���ù��λ��
        LCD_WriteRAM_Prepare();     //��ʼд��GRAM

        for (j = 0; j < xlen; j++)
        {
            if (lcddev.id == 0X7789)
            {
                LCD->LCD_RAM = color >> 8;
                LCD->LCD_RAM = color;
            }
            else
            {
                LCD->LCD_RAM = color;
            }
        }
    }
}

//��ָ�����������ָ����ɫ��
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)
//color:Ҫ������ɫ
void LCD_Color_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 *color)
{
    u16 height, width;
    u16 i, j;
    width = ex - sx + 1;            //�õ����Ŀ��
    height = ey - sy + 1;           //�߶�

    for (i = 0; i < height; i++)
    {
        LCD_SetCursor(sx, sy + i);  //���ù��λ��
        LCD_WriteRAM_Prepare();     //��ʼд��GRAM

        for (j = 0; j < width; j++)
        {
            if (lcddev.id == 0X7789)
            {
                LCD->LCD_RAM = color[i * width + j] >> 8;
                LCD->LCD_RAM = color[i * width + j];
            }
            else
            {
                LCD->LCD_RAM = color[i * width + j];
            }
						//LCD->LCD_RAM=color[i * width + j];  //д������
        }
    }
}

//����
//x1,y1:�������
//x2,y2:�յ�����  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1;              //������������
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0)incx = 1;       //���õ�������
    else if (delta_x == 0)incx = 0; //��ֱ��
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)incy = 1;
    else if (delta_y == 0)incy = 0; //ˮƽ��
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if ( delta_x > delta_y)distance = delta_x; //ѡȡ��������������
    else distance = delta_y;

    for (t = 0; t <= distance + 1; t++ )    //�������
    {
        LCD_DrawPoint(uRow, uCol); //����
        xerr += delta_x ;
        yerr += delta_y ;

        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

//������
//(x1,y1),(x2,y2):���εĶԽ�����
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//��ָ��λ�û�һ��ָ����С��Բ
//(x,y):���ĵ�
//r    :�뾶
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);       //�ж��¸���λ�õı�־

    while (a <= b)
    {
        LCD_DrawPoint(x0 + a, y0 - b);        //5
        LCD_DrawPoint(x0 + b, y0 - a);        //0
        LCD_DrawPoint(x0 + b, y0 + a);        //4
        LCD_DrawPoint(x0 + a, y0 + b);        //6
        LCD_DrawPoint(x0 - a, y0 + b);        //1
        LCD_DrawPoint(x0 - b, y0 + a);
        LCD_DrawPoint(x0 - a, y0 - b);        //2
        LCD_DrawPoint(x0 - b, y0 - a);        //7
        a++;

        //ʹ��Bresenham�㷨��Բ
        if (di < 0)di += 4 * a + 6;
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}


//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16/24
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���	
 	num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[num][t];	//����1608����
		else if(size==24)temp=asc2_2412[num][t];	//����2412����
		else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}   
//m^n����
//����ֵ:m^n�η�.
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//��ʾ����,��λΪ0,����ʾ
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//color:��ɫ
//num:��ֵ(0~4294967295);
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size)
{
    u8 t, temp;
    u8 enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                LCD_ShowChar(x + (size / 2)*t, y, ' ', size, 0);
                continue;
            }
            else enshow = 1;

        }

        LCD_ShowChar(x + (size / 2)*t, y, temp + '0', size, 0);
    }
}

//��ʾ����,��λΪ0,������ʾ
//x,y:�������
//num:��ֵ(0~999999999);	 
//len:����(��Ҫ��ʾ��λ��)
//size:�����С
//mode:
//[7]:0,�����;1,���0.
//[6:1]:����
//[0]:0,�ǵ�����ʾ;1,������ʾ.
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode)
{
    u8 t, temp;
    u8 enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                if (mode & 0X80)LCD_ShowChar(x + (size / 2)*t, y, '0', size, mode & 0X01);
                else LCD_ShowChar(x + (size / 2)*t, y, ' ', size, mode & 0X01);

                continue;
            }
            else enshow = 1;

        }

        LCD_ShowChar(x + (size / 2)*t, y, temp + '0', size, mode & 0X01);
    }
}

//��ʾ�ַ���
//x,y:�������
//width,height:�����С
//size:�����С
//*p:�ַ�����ʼ��ַ
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, u8 *p)
{
    u8 x0 = x;
    width += x;
    height += y;

    while ((*p <= '~') && (*p >= ' '))   //�ж��ǲ��ǷǷ��ַ�!
    {
        if (x >= width)
        {
            x = x0;
            y += size;
        }

        if (y >= height)break; //�˳�

        LCD_ShowChar(x, y, *p, size, 0);
        x += size / 2;
        p++;
    }  
}






























