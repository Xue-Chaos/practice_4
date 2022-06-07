/* ����ͷ�ļ� */
#include "ioCC2530.h"
#include <stdio.h>
#define LED1 P1_0     // P1_0����ΪP1_0  led�ƶ˿�
#define uint16 unsigned short
#define uint32 unsigned long



unsigned char SprintfAsciiResult[20];//���ת�����ASCII�ַ���
unsigned char ProtocolBuff[9];//���͵����ص�Э��֡
uint16  counter=0; //ͳ���������
unsigned char AsciiStr[8];//����һ�������СΪ8
unsigned char CHK(unsigned char *buf,int len);


void InitLED()
{
//�������LED�ĳ�ʼ��
    
    P1DIR |= 0x01;
    LED1 = 0;
   
}

void adc_Init(void)
{
  APCFG  |=0x01;        //ʹ��P0ģ�����蹦�ܣ�ָ��P0_0
  P0SEL  |=0x01;	//����P0_0 Ϊ�����
  P0DIR  &=~0x01;	//����p0_0 Ϊ�����
}

/************************************************************
* ����       get_adc
* ����       ��ȡADCͨ��0��ѹֵ
* ��ڲ���   ��
* ���ڲ���   16λ��ѹֵ���ֱ���Ϊ10mV
***************��ȡADCͨ��0��ѹֵ************************/

uint16 get_adc(void)
{
	uint32 value;
	ADCIF = 0;   //��ADC �жϱ�־
	//���û�׼��ѹavdd5:3.3V��ͨ��0������ADת��
	//�������ADC�ĳ�ʼ��
//=====���հ״���ʼ
    
    
    
    
//��=====�հ״�����
	// ADֵת���ɵ�ѹֵ
	// 0 ��ʾ 0V ��32768 ��ʾ 3.3V
	// ��ѹֵ = (value*3.3)/32768 ��V)
    value = (value * 330);// ���طֱ���Ϊ10mV�ĵ�ѹֵ
    value = value >> 15;   // ����32768
	// ���طֱ���Ϊ0.01V�ĵ�ѹֵ
    return (uint16)value;
}

/**********����ͨ�ų�ʼ��************************/
void initUART0(void)
{
	PERCFG = 0x00;	
	P0SEL = 0x3c;	
	//�ɿ������ô�����ز�����Ҫ������115200
    
    U0CSR |= 0x80;
    U0BAUD = 216;
    U0GCR = 11;

	UTX0IF = 0;  // ����UART0 TX�жϱ�־
	EA = 1;   //ʹ��ȫ���ж�
}

/*************************************************
* �������ƣ�initTimer1
* ��    �ܣ���ʼ����ʱ��T1����״̬�Ĵ���
******************��ʱ����ʼ��*****************************/
void initTimer1()
{
//�������LED�ĳ�ʼ��
//=====���հ״���ʼ
    
    CLKCONCMD &= 0x80;
    while (CLKCONCMD & 0x40);

    T1CTL = 0x0e;
    T1CCTL0 |= 0x04;
    T1CC0L = (50000 & 0xff);
    T1CC0H = (50000 & 0x00ff) >> 8;
    
    
//��=====�հ״�����
	
    T1IF=0;           //���timer1�жϱ�־(ͬIRCON &= ~0x02)
    T1STAT &= ~0x01;  //���ͨ��0�жϱ�־
    TIMIF &= ~0x40;  //��������ʱ��1������ж�
   //��ʱ��1��ͨ��0���ж�ʹ��T1CCTL0.IMĬ��ʹ��
    IEN1 |= 0x02;    //ʹ�ܶ�ʱ��1���ж�
    EA = 1;        //ʹ��ȫ���ж�
}


/**************��Ƭ������Э��֡���ݵ�����******************/
void UART0SendData(unsigned char *str,int len )
{
  for(int i=0;i<=len;i++)
  {
    U0DBUF = str[i];   // ��Ҫ���͵�1�ֽ�����д��U0DBUF
    while (!UTX0IF) ;  // �ȴ�TX�жϱ�־����U0DBUF����
    UTX0IF = 0;        // ����TX�жϱ�־UART0SendByte(*str++); 
  }
}

/**************�������͵�ѹֵ�����������ص�Э��֡******************/
void BuildProtocolFrame(uint16 voltage)
{
  ProtocolBuff[0]=0xDD;
  ProtocolBuff[1]=0x03;
  ProtocolBuff[2]=0x00;
  ProtocolBuff[3]=0x01;
  ProtocolBuff[4]=0x09;
  ProtocolBuff[5]=0x04;
  ProtocolBuff[6]=voltage>>8;
  ProtocolBuff[7]=voltage&0xFF;
  ProtocolBuff[8]=CHK(ProtocolBuff,8);
}

/**************��У�麯��******************/
 unsigned char CHK(unsigned char *buf,int len)
{
    unsigned char  RX_CHX=0;
     while(len--) {
        RX_CHX+= *buf;
        buf++;
    }

    return RX_CHX&=0xff;
}

/**************��ȡ��ѹֵ����������******************/
void Get_AsciiValue()
{
  uint16 sensor_val;
  sensor_val=get_adc();
  AsciiStr[0]=sensor_val/100+'0';
  AsciiStr[1]='.';
  AsciiStr[2]=sensor_val/10%10+'0';
  AsciiStr[3]=sensor_val%10+'0';
  AsciiStr[4]='V';
  AsciiStr[5]='\n';
 }

/******************************************
* ��    �ܣ���ʱ��T1�жϷ����ӳ���
************************************/
#pragma vector = T1_VECTOR //�жϷ����ӳ���
__interrupt void T1_ISR(void)
{
	EA = 0;   //��ֹȫ���ж�
	counter++;
	T1STAT &= ~0x01;  //���ͨ��0�жϱ�־
	EA = 1;   //ʹ��ȫ���ж�
}
//LED1��˸ʱ���ӳ�
void delay(int time)
{
   for(int i=0;i<=time;i++)
   {
     for(int j=0;j<=240;j++);
   }


}
/******************************************
* �������ƣ�main
* ��    �ܣ�main�������
* ��ڲ�������
* ���ڲ�������
* �� �� ֵ����
**************************************************/
void main(void)
{
    InitLED();
    initTimer1();  //��ʼ��Timer1
    initUART0();  // UART0��ʼ��
    adc_Init(); // ADC��ʼ��
   while(1)
	{
	if(counter>=10)     //��ʱ��ÿ0.2S����жϼƴ�
	{
	  counter=0;       //���־λ
	  LED1 = 1;    //ָʾ�Ƶ���
	  Get_AsciiValue();
#ifdef debug
	//�ɿ�������򴮿ڷ��Ϳ�ȼ����ֵASCII�ַ���
    //=====���հ״���ʼ
    
      
    
    //��=====�հ״�����
#else
     //�ɿ�������򴮿ڷ��Ϳ�ȼ����Э��֡
    //=====���հ״���ʼ
    
      
    
    //��=====�հ״�����
     delay(500);
                        
#endif
                        
    LED1 = 0;    //ָʾ��Ϩ��
      }
  }
}
