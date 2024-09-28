#include "ctiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//WKS STM32F407VET6���İ�
//���ݴ�����-IIC ��������
//�汾��V1.0
//////////////////////////////////////////////////////////////////////////////////

//����I2C�ٶȵ���ʱ
void CT_Delay(void)
{
    delay_us(2);
}
//���ݴ���оƬIIC�ӿڳ�ʼ��
void CT_IIC_Init(void)
{
    RCC->AHB1ENR |= 1 << 1;    		//ʹ��PORTBʱ��
    GPIO_Set(GPIOB, PIN12, GPIO_MODE_OUT, GPIO_OTYPE_OD, GPIO_SPEED_100M, GPIO_PUPD_PU); 	//PB12����Ϊ�������
    GPIO_Set(GPIOB, PIN15, GPIO_MODE_OUT, GPIO_OTYPE_OD, GPIO_SPEED_100M, GPIO_PUPD_PU);  //PB15����Ϊ�������
}
//����IIC��ʼ�ź�
void CT_IIC_Start(void)
{
    CT_IIC_SDA = 1;
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SDA = 0; //��ʼ�ź�:SCLΪ�ߵ�ƽʱ,SDA�ɸߵ�ƽ��͵�ƽ���� 
    CT_Delay();
    CT_IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
    CT_Delay();
}
//����IICֹͣ�ź�
void CT_IIC_Stop(void)
{
    CT_IIC_SDA = 0; //�����ź�:SCLΪ�ߵ�ƽʱ��SDA�ɵ͵�ƽ��ߵ�ƽ����
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SDA = 1; //����I2C���߽����ź�
    CT_Delay();
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 CT_IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    u8 rack = 0;

    CT_IIC_SDA = 1;
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();

    while (CT_READ_SDA)
    {
        ucErrTime++;

        if (ucErrTime > 250)
        {
            CT_IIC_Stop();
            rack = 1;
            break;
        }

        CT_Delay();
    }

    CT_IIC_SCL = 0; //ʱ�����0
    CT_Delay();
    return rack;
}
//����ACKӦ��
void CT_IIC_Ack(void)
{
    CT_IIC_SDA = 0;
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SCL = 0;
    CT_Delay();
    CT_IIC_SDA = 1;
    CT_Delay();
}
//������ACKӦ��
void CT_IIC_NAck(void)
{
    CT_IIC_SDA = 1;
    CT_Delay();
    CT_IIC_SCL = 1;
    CT_Delay();
    CT_IIC_SCL = 0;
    CT_Delay();
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void CT_IIC_Send_Byte(u8 txd)
{
    u8 t;

    for (t = 0; t < 8; t++)
    {
        CT_IIC_SDA = (txd & 0x80) >> 7;
        CT_Delay();
        CT_IIC_SCL = 1;  //��SCL������ʱ���д���SDA�ϵ�����
        CT_Delay();
        CT_IIC_SCL = 0;
        txd <<= 1;
    }

    CT_IIC_SDA = 1;
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 CT_IIC_Read_Byte(unsigned char ack)
{
    u8 i, receive = 0;

    for (i = 0; i < 8; i++ )
    {
        receive <<= 1;
        CT_IIC_SCL = 1;
        CT_Delay();

        if (CT_READ_SDA)receive++;  //SCLΪ�ߵ�ƽ�ڼ䣬��ȡSDA�ϵ�����

        CT_IIC_SCL = 0;
        CT_Delay();
    }

    if (!ack)CT_IIC_NAck();//����nACK
    else CT_IIC_Ack(); //����ACK

    return receive;
}




























