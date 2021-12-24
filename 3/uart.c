/*
 * c6748_uart.c
 *
 *  Created on: 2019��9��9��
 *      Author: lh03061238
 */

// ע�⣺DSP ports, Shared RAM, UART0, EDMA, SPI0, MMC/SDs,
//       VPIF, LCDC, SATA, uPP, DDR2/mDDR (bus ports), USB2.0, HPI, PRU
//       ��Щ����ʹ�õ�ʱ����ԴΪ PLL0_SYSCLK2 Ĭ��Ƶ��Ϊ CPU Ƶ�ʵĶ���֮һ
//       ���ǣ�ECAPs, UART1/2, Timer64P2/3, eHRPWMs,McBSPs, McASP0, SPI1
//       ��Щ�����ʱ����Դ������ PLL0_SYSCLK2 �� PLL1_SYSCLK2 ��ѡ��
//       ͨ���޸� System Configuration (SYSCFG) Module
//       �Ĵ��� Chip Configuration 3 Register (CFGCHIP3) ����λ ASYNC3_CLKSRC
//       ����ʱ����Դ
//       ��Ĭ��ֵ�� 0 ��Դ�� PLL0_SYSCLK2
//                  1 ��Դ�� PLL1_SYSCLK2
//       �������Ϊ�˽��͹��ģ��������޸����ֵ������Ӱ��������������ʱ��Ƶ��

#include "TL6748.h"                 // ���� DSP6748 �������������

#include "hw_types.h"               // ������
#include "hw_syscfg0_C6748.h"       // ϵͳ����ģ��Ĵ���
#include "soc_C6748.h"              // DSP C6748 ����Ĵ���

#include "psc.h"                    // ��Դ��˯�߿��ƺ꼰�豸����㺯������
#include "gpio.h"                   // ͨ����������ں꼰�豸����㺯������
#include "uart.h"                   // ͨ���첽���ں꼰�豸����㺯������
#include "interrupt.h"              // DSP C6748 �ж����Ӧ�ó���ӿں���������ϵͳ�¼��Ŷ���
#include "global_def.h"


char txArray[] = "Tronlong UART2 Application......\n\r";


#pragma  CODE_SECTION(usart1_process, ".MIPSUART")

#pragma  CODE_SECTION(UARTIsr1, ".MIPSUART")


extern unsigned int Time34;
extern unsigned int timercnt,timercnt1,timercnt2;






// UART ��ʼ��
void UARTInit(void);

// UART �жϳ�ʼ��
void UARTInterruptInit();
// UART ���崦����
unsigned long  usart1_process(void);

// UART �жϷ�����
void UARTIsr1(void);
void UARTIsr2(void);

extern unsigned int crc16(unsigned char *buf, int len);



/****************************************************************************/
/*                                                                          */
/*              UART ��ʼ��                                                 */
/*                                                                          */
/****************************************************************************/
void UARTInit(void)
{

	/**********************UART1��ʼ��**********************/
		// ���� UART1����
		// ������ 460800 ����λ 8 ֹͣλ 1 ��У��λ
		UARTConfigSetExpClk(SOC_UART_1_REGS, UART_1_FREQ, 460800,
							  UART_WORDL_8BITS, UART_OVER_SAMP_RATE_16);
		// ʹ�� UART2
		UARTEnable(SOC_UART_1_REGS);
	
		// ʹ�ܽ��� / ���� FIFO
		UARTFIFOEnable(SOC_UART_1_REGS);
	
		// ���� FIFO ����
		UARTFIFOLevelSet(SOC_UART_1_REGS, UART_RX_TRIG_LEVEL_1);
	/****************************************************************/	

    /**********************UART2��ʼ��**********************/
    // ���� UART2 ����
    // ������ 460800 ����λ 8 ֹͣλ 1 ��У��λ
    UARTConfigSetExpClk(SOC_UART_2_REGS, UART_2_FREQ, BAUD_115200,
                          UART_WORDL_8BITS, UART_OVER_SAMP_RATE_16);
    // ʹ�� UART2
    UARTEnable(SOC_UART_2_REGS);

    // ʹ�ܽ��� / ���� FIFO
    UARTFIFOEnable(SOC_UART_2_REGS);

    // ���� FIFO ����
    UARTFIFOLevelSet(SOC_UART_2_REGS, UART_RX_TRIG_LEVEL_1);
/****************************************************************/	
}

/****************************************************************************/
/*                                                                          */
/*              UART �жϳ�ʼ��                                             */
/*                                                                          */
/****************************************************************************/
void UARTInterruptInit(void)
{
/**********SYS_INT_UART1_INT   ʹ��***************************/
	IntRegister(C674X_MASK_INT6, UARTIsr1);
	IntEventMap(C674X_MASK_INT6, SYS_INT_UART1_INT);
	IntEnable(C674X_MASK_INT6);
	
	// ʹ���ж�
	unsigned int intFlags1 = 0;
	intFlags1 |=   (UART_INT_LINE_STAT	|  \
	                       UART_INT_RXDATA_CTI);
	UARTIntEnable(SOC_UART_1_REGS, intFlags1);
/******************************************************************/



 
/**********SYS_INT_UART2_INT   ʹ��***************************/
    IntRegister(C674X_MASK_INT7, UARTIsr2);
    IntEventMap(C674X_MASK_INT7, SYS_INT_UART2_INT);
    IntEnable(C674X_MASK_INT7);

    // ʹ���ж�
    unsigned int intFlags2 = 0;
    intFlags2 |=   (UART_INT_LINE_STAT  |  \
                          UART_INT_RXDATA_CTI);
    UARTIntEnable(SOC_UART_2_REGS, intFlags2);
/******************************************************************/

	
}
/*
unsigned long  usart1_process_1(void)
	{
				short unsigned int	i,check;
				short unsigned int flag=0;//�����־
				unsigned long p_c0,data_len;
				unsigned long uart1_endbak;
				int crc_len;
			 
			 //ɨ�軺��
			 p_c0=flag=0;
			 i=usart1_start;
			 uart1_endbak=usart1_end;
			 check=0;	
			 crc_len=0;
			 while(i!=uart1_endbak)
			  {
				 switch(check)
				{
					 case 0:
					  {
						 if(USART1_RX_BUF[i]==0x7e)//�ҵ���ͷ�ĵ�һ�ֽ�
							{
								check++;
								 main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++;
							}
							else
							{
								  check=0;
							}				 
					  };break;		 
					 case 1:
					  {
						 if(USART1_RX_BUF[i]==0x7f)//�ҵ���ͷ�ĵڶ��ֽ�
							  {
								  check++;
								  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++;
							   }
						 else	 
							  {
								  check=0; //����
								p_c0=0;  
								 }				 
					   };break;
					case 2:
					  {
						 if(p_c0==(data_len+cmd_rear_offset))//�ҵ���β
							  {
								 
								if(USART1_RX_BUF[i]==0x9e) //�ж��Ƿ��ǰ�β
									{
										   check++;
											main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
		
									}
								else		
									{
										 check=0; //����
											p_c0=0;  
									}
							  }
						 else	 
							  {
		
								  check=2; 
								  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
								  if(p_c0==cmd_data_bit)  //���յ�len�ֶ�ʱ����¼����
									 {
										data_len=main_recbuffer[p_c0-2]*256+main_recbuffer[p_c0-1];
									 }
							  } 			 
					  };break;
					 case 3:
					 {
						 if(USART1_RX_BUF[i]==0x9f)//�ҵ���β�ĵڶ��ֽ�
							  {
								  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
							          
								  flag=1;	
							  }
						 else	 
							  {
								  check=0;	 //����
									p_c0=0;   
							  } 			 
						};break;
				}	
			   i++;
			   if(i>=USART_BUFFERSIZE){i=0;};
			   if(flag==1){break;};
			  }
			 
			 if(flag)//�ҵ�һ����Чָ��
				{
				////////////////////////////////////////////////////
				
					 crcbakrx = &main_recbuffer[2]; 
					 crc_len=data_len+7;			  
					 crctest=crc16(crcbakrx,crc_len);
						  if((main_recbuffer[p_c0-4]!=(unsigned char)((crctest&0xff00)>>8))||(main_recbuffer[p_c0-3]!=(unsigned char)(crctest&0x00ff)))//У��λ�ȶ�
								  {
									//printf("CRC ERROR\n") ;
							               usart1_start=i;//����usart2_startָ��
								  } 
				               else 	   
						                {
					   ////////////////////////////////////////////////////
									usart1_start=i;//����usart2_startָ��
								       return(p_c0);    
						                 }
							  
				}
			 return(0);
			}
*/
unsigned long  usart1_process(void)
	{
			short unsigned int	i,check;
			short unsigned int flag=0;//�����־
			unsigned long p_c0,data_len;
			unsigned long uart1_endbak;
			int	crc_len;
		 
		 //ɨ�軺��
		 p_c0=flag=0;
		 i=usart1_start;
		 uart1_endbak=usart1_end;
		 check=0;	
		 crc_len=0;
		 while(i!=uart1_endbak)
		  {
			 switch(check)
			{
				 case 0:
				  {
					 if(USART1_RX_BUF[i]==0x7e)//�ҵ���ͷ�ĵ�һ�ֽ�
						{
							check++;
							 main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++;
						}
						else
						{
							  check=0;
						}				 
				  };break;		 
				 case 1:
				  {
					 if(USART1_RX_BUF[i]==0x7f)//�ҵ���ͷ�ĵڶ��ֽ�
						  {
							  check++;
							  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++;
						   }
					 else	 
						  {
							   check=0; //������
							    p_c0=0;  
							    usart1_start=i ;    //����֮ǰ����
						   }				 
				   };break;
				case 2:
				  {
					 if(p_c0==(data_len+cmd_rear_offset))//�ҵ���β
						  {
							 
							if(USART1_RX_BUF[i]==0x9e) //�ж��Ƿ��ǰ�β
								{
									   check++;
										main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
	
								}
							else		
								{
									      check=0; //������
										p_c0=0;  
										 usart1_start=i ;    //����֮ǰ����
								}
						  }
					 else	 
						  {
	
							  check=2; 
							  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
							  if(p_c0==cmd_data_bit)  //���յ�len�ֶ�ʱ����¼����
								 {
									data_len=main_recbuffer[p_c0-2]*256+main_recbuffer[p_c0-1];
									if(data_len>410)  //�ж��������ȣ���ֹ���ȴ�������������
										{
										      check=0;  //������
										      p_c0=0;  
										      usart1_start=i ;    //����֮ǰ����
										}
								 }
						  } 			 
				  };break;
				 case 3:
				 {
					 if(USART1_RX_BUF[i]==0x9f)//�ҵ���β�ĵڶ��ֽ�
						  {
							  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
						
							  flag=1;	
						  }
					 else	 
						  {
							       check=0;	  //������
								p_c0=0;   
							       usart1_start=i ;    //����֮ǰ����
						  } 			 
					};break;
			}	
		   i++;
		   if(i>=USART_BUFFERSIZE){i=0;};
		   if(flag==1){break;};
		  }
		 
		 if(flag)//�ҵ�һ����Чָ��
			{
			////////////////////////////////////////////////////
			
				 crcbakrx = &main_recbuffer[2];
				 crc_len=data_len+7;			  
				 crctest=crc16(crcbakrx,crc_len);
		     	   if((main_recbuffer[p_c0-4]!=(unsigned char)((crctest&0xff00)>>8))||(main_recbuffer[p_c0-3]!=(unsigned char)(crctest&0x00ff)))//У��λ�ȶ�
				   {
				       usart1_start=i;//����usart2_startָ��
				//	printf("CRC ERROR\n") ;
						
				    } 
			       else 	   
				   {
				   ////////////////////////////////////////////////////
								usart1_start=i;//����usart2_startָ��
									   return(p_c0);
				   }
						  
			}
		 return(0);
		}












/****************************************************************************/
/*                                                                          */
/*              UART �жϷ�����                                           */
/*                                                                          */
/****************************************************************************/
void UARTIsr1()
{
 //   static unsigned int length = sizeof(txArray);
 //   static unsigned int count = 0;
//    unsigned char rxData1 = 0;
    unsigned int int_id = 0;

    // ȷ���ж�Դ
  int_id = UARTIntStatus(SOC_UART_1_REGS);
//   INT_ID=int_id;
//   INT_ID_Sum =INT_ID_Sum+INT_ID;
//   INT_NUM++;

    // ��� UART2 ϵͳ�ж�
    IntEventClear(SYS_INT_UART1_INT);
   //��ֹ UART  �ж�
   UARTIntDisable(SOC_UART_1_REGS, (UART_INT_LINE_STAT	|  UART_INT_RXDATA_CTI));



// ȷ���ж�Դ
//   int_id = UARTIntStatus(SOC_UART_1_REGS);

    // ���մ���
    if(UART_INTID_RX_LINE_STAT == int_id)
    {

        while(UARTRxErrorGet(SOC_UART_1_REGS))

        {
            // �� RBR ��һ���ֽ�
            UARTCharGetNonBlocking(SOC_UART_1_REGS);
        }
    }

	/*
    // �����ж�
    if(UART_INTID_TX_EMPTY == int_id)
    {
        if(0 < length)
        {
            // дһ���ֽڵ� THR
            UARTCharPutNonBlocking(SOC_UART_1_REGS, txArray[count]);
            length--;
            count++;
        }
        if(0 == length)
        {
            // ���÷����ж�
            UARTIntDisable(SOC_UART_1_REGS, UART_INT_TX_EMPTY);
        }
     }
*/
	
	// �����ж�
		if(UART_INTID_RX_DATA == int_id)
		{

	   
	   
	USART1_RX_BUF[usart1_end] =UARTCharGetNonBlocking(SOC_UART_1_REGS);
 
    //   if(USART1_RX_BUF[usart1_end] ==0x7E)
     //  	{
    //   	      timercnt1 =	Time34; 
     //  	}

		  usart1_end++;  
		   if(usart1_end >= USART_BUFFERSIZE)
			  {
				 usart1_end = 0;
			  }

		
		}


/*
		
		if( int_id!=UART_INTID_RX_DATA)
			{
			        UARTCharGetNonBlocking(SOC_UART_1_REGS);
			}

*/
		

    // ��� UART2 ϵͳ�ж�
//    IntEventClear(SYS_INT_UART1_INT);

   // ʹ�� UART2
  // UARTEnable(SOC_UART_1_REGS);

// ʹ�� UART2 �ж�
 UARTIntEnable(SOC_UART_1_REGS, (UART_INT_LINE_STAT	|  UART_INT_RXDATA_CTI));

    return;
}

void UARTIsr2()
{
    static unsigned int length = sizeof(txArray);
    static unsigned int count = 0;
    unsigned char rxData2 = 0;
    unsigned int int_id = 0;

    // ȷ���ж�Դ
    int_id = UARTIntStatus(SOC_UART_2_REGS);

    // ��� UART2 ϵͳ�ж�
    IntEventClear(SYS_INT_UART2_INT);

    // �����ж�
    if(UART_INTID_TX_EMPTY == int_id)
    {
        if(0 < length)
        {
            // дһ���ֽڵ� THR
            UARTCharPutNonBlocking(SOC_UART_2_REGS, txArray[count]);
            length--;
            count++;
        }
        if(0 == length)
        {
            // ���÷����ж�
            UARTIntDisable(SOC_UART_2_REGS, UART_INT_TX_EMPTY);
        }
     }

    // �����ж�
    if(UART_INTID_RX_DATA == int_id)
    {
        rxData2= UARTCharGetNonBlocking(SOC_UART_2_REGS);
        UARTCharPutNonBlocking(SOC_UART_2_REGS, rxData2);
    }

    // ���մ���
    if(UART_INTID_RX_LINE_STAT == int_id)
    {
         INT_ID=UARTRxErrorGet(SOC_UART_2_REGS);
        while(UARTRxErrorGet(SOC_UART_2_REGS))
        {
            // �� RBR ��һ���ֽ�
            UARTCharGetNonBlocking(SOC_UART_2_REGS);
        }
    }

    return;
}


