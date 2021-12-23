/*
 * c6748_uart.c
 *
 *  Created on: 2019年9月9日
 *      Author: lh03061238
 */

// 注意：DSP ports, Shared RAM, UART0, EDMA, SPI0, MMC/SDs,
//       VPIF, LCDC, SATA, uPP, DDR2/mDDR (bus ports), USB2.0, HPI, PRU
//       这些外设使用的时钟来源为 PLL0_SYSCLK2 默认频率为 CPU 频率的二分之一
//       但是，ECAPs, UART1/2, Timer64P2/3, eHRPWMs,McBSPs, McASP0, SPI1
//       这些外设的时钟来源可以在 PLL0_SYSCLK2 和 PLL1_SYSCLK2 中选择
//       通过修改 System Configuration (SYSCFG) Module
//       寄存器 Chip Configuration 3 Register (CFGCHIP3) 第四位 ASYNC3_CLKSRC
//       配置时钟来源
//       （默认值） 0 来源于 PLL0_SYSCLK2
//                  1 来源于 PLL1_SYSCLK2
//       如果不是为了降低功耗，不建议修改这个值，它会影响所有相关外设的时钟频率

#include "TL6748.h"                 // 创龙 DSP6748 开发板相关声明

#include "hw_types.h"               // 宏命令
#include "hw_syscfg0_C6748.h"       // 系统配置模块寄存器
#include "soc_C6748.h"              // DSP C6748 外设寄存器

#include "psc.h"                    // 电源与睡眠控制宏及设备抽象层函数声明
#include "gpio.h"                   // 通用输入输出口宏及设备抽象层函数声明
#include "uart.h"                   // 通用异步串口宏及设备抽象层函数声明
#include "interrupt.h"              // DSP C6748 中断相关应用程序接口函数声明及系统事件号定义
#include "global_def.h"


char txArray[] = "Tronlong UART2 Application......\n\r";


#pragma  CODE_SECTION(usart1_process, ".MIPSUART")

#pragma  CODE_SECTION(UARTIsr1, ".MIPSUART")


extern unsigned int Time34;
extern unsigned int timercnt,timercnt1,timercnt2;






// UART 初始化
void UARTInit(void);

// UART 中断初始化
void UARTInterruptInit();
// UART 缓冲处理函数
unsigned long  usart1_process(void);

// UART 中断服务函数
void UARTIsr1(void);
void UARTIsr2(void);

extern unsigned int crc16(unsigned char *buf, int len);



/****************************************************************************/
/*                                                                          */
/*              UART 初始化                                                 */
/*                                                                          */
/****************************************************************************/
void UARTInit(void)
{

	/**********************UART1初始化**********************/
		// 配置 UART1参数
		// 波特率 460800 数据位 8 停止位 1 无校验位
		UARTConfigSetExpClk(SOC_UART_1_REGS, UART_1_FREQ, 460800,
							  UART_WORDL_8BITS, UART_OVER_SAMP_RATE_16);
		// 使能 UART2
		UARTEnable(SOC_UART_1_REGS);
	
		// 使能接收 / 发送 FIFO
		UARTFIFOEnable(SOC_UART_1_REGS);
	
		// 设置 FIFO 级别
		UARTFIFOLevelSet(SOC_UART_1_REGS, UART_RX_TRIG_LEVEL_1);
	/****************************************************************/	

    /**********************UART2初始化**********************/
    // 配置 UART2 参数
    // 波特率 460800 数据位 8 停止位 1 无校验位
    UARTConfigSetExpClk(SOC_UART_2_REGS, UART_2_FREQ, BAUD_115200,
                          UART_WORDL_8BITS, UART_OVER_SAMP_RATE_16);
    // 使能 UART2
    UARTEnable(SOC_UART_2_REGS);

    // 使能接收 / 发送 FIFO
    UARTFIFOEnable(SOC_UART_2_REGS);

    // 设置 FIFO 级别
    UARTFIFOLevelSet(SOC_UART_2_REGS, UART_RX_TRIG_LEVEL_1);
/****************************************************************/	
}

/****************************************************************************/
/*                                                                          */
/*              UART 中断初始化                                             */
/*                                                                          */
/****************************************************************************/
void UARTInterruptInit(void)
{
/**********SYS_INT_UART1_INT   使能***************************/
	IntRegister(C674X_MASK_INT6, UARTIsr1);
	IntEventMap(C674X_MASK_INT6, SYS_INT_UART1_INT);
	IntEnable(C674X_MASK_INT6);
	
	// 使能中断
	unsigned int intFlags1 = 0;
	intFlags1 |=   (UART_INT_LINE_STAT	|  \
	                       UART_INT_RXDATA_CTI);
	UARTIntEnable(SOC_UART_1_REGS, intFlags1);
/******************************************************************/



 
/**********SYS_INT_UART2_INT   使能***************************/
    IntRegister(C674X_MASK_INT7, UARTIsr2);
    IntEventMap(C674X_MASK_INT7, SYS_INT_UART2_INT);
    IntEnable(C674X_MASK_INT7);

    // 使能中断
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
				short unsigned int flag=0;//处理标志
				unsigned long p_c0,data_len;
				unsigned long uart1_endbak;
				int crc_len;
			 
			 //扫描缓冲
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
						 if(USART1_RX_BUF[i]==0x7e)//找到包头的第一字节
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
						 if(USART1_RX_BUF[i]==0x7f)//找到包头的第二字节
							  {
								  check++;
								  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++;
							   }
						 else	 
							  {
								  check=0; //丢弃
								p_c0=0;  
								 }				 
					   };break;
					case 2:
					  {
						 if(p_c0==(data_len+cmd_rear_offset))//找到包尾
							  {
								 
								if(USART1_RX_BUF[i]==0x9e) //判断是否是包尾
									{
										   check++;
											main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
		
									}
								else		
									{
										 check=0; //丢弃
											p_c0=0;  
									}
							  }
						 else	 
							  {
		
								  check=2; 
								  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
								  if(p_c0==cmd_data_bit)  //接收到len字段时，记录包长
									 {
										data_len=main_recbuffer[p_c0-2]*256+main_recbuffer[p_c0-1];
									 }
							  } 			 
					  };break;
					 case 3:
					 {
						 if(USART1_RX_BUF[i]==0x9f)//找到包尾的第二字节
							  {
								  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
							          
								  flag=1;	
							  }
						 else	 
							  {
								  check=0;	 //丢弃
									p_c0=0;   
							  } 			 
						};break;
				}	
			   i++;
			   if(i>=USART_BUFFERSIZE){i=0;};
			   if(flag==1){break;};
			  }
			 
			 if(flag)//找到一包有效指令
				{
				////////////////////////////////////////////////////
				
					 crcbakrx = &main_recbuffer[2]; 
					 crc_len=data_len+7;			  
					 crctest=crc16(crcbakrx,crc_len);
						  if((main_recbuffer[p_c0-4]!=(unsigned char)((crctest&0xff00)>>8))||(main_recbuffer[p_c0-3]!=(unsigned char)(crctest&0x00ff)))//校验位比对
								  {
									//printf("CRC ERROR\n") ;
							               usart1_start=i;//更新usart2_start指针
								  } 
				               else 	   
						                {
					   ////////////////////////////////////////////////////
									usart1_start=i;//更新usart2_start指针
								       return(p_c0);    
						                 }
							  
				}
			 return(0);
			}
*/
unsigned long  usart1_process(void)
	{
			short unsigned int	i,check;
			short unsigned int flag=0;//处理标志
			unsigned long p_c0,data_len;
			unsigned long uart1_endbak;
			int	crc_len;
		 
		 //扫描缓冲
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
					 if(USART1_RX_BUF[i]==0x7e)//找到包头的第一字节
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
					 if(USART1_RX_BUF[i]==0x7f)//找到包头的第二字节
						  {
							  check++;
							  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++;
						   }
					 else	 
						  {
							   check=0; //重新找
							    p_c0=0;  
							    usart1_start=i ;    //丢掉之前数据
						   }				 
				   };break;
				case 2:
				  {
					 if(p_c0==(data_len+cmd_rear_offset))//找到包尾
						  {
							 
							if(USART1_RX_BUF[i]==0x9e) //判断是否是包尾
								{
									   check++;
										main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
	
								}
							else		
								{
									      check=0; //重新找
										p_c0=0;  
										 usart1_start=i ;    //丢掉之前数据
								}
						  }
					 else	 
						  {
	
							  check=2; 
							  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
							  if(p_c0==cmd_data_bit)  //接收到len字段时，记录包长
								 {
									data_len=main_recbuffer[p_c0-2]*256+main_recbuffer[p_c0-1];
									if(data_len>410)  //判断最大包长度，防止长度错误引发的问题
										{
										      check=0;  //重新找
										      p_c0=0;  
										      usart1_start=i ;    //丢掉之前数据
										}
								 }
						  } 			 
				  };break;
				 case 3:
				 {
					 if(USART1_RX_BUF[i]==0x9f)//找到包尾的第二字节
						  {
							  main_recbuffer[p_c0]=USART1_RX_BUF[i];p_c0++; 
						
							  flag=1;	
						  }
					 else	 
						  {
							       check=0;	  //重新找
								p_c0=0;   
							       usart1_start=i ;    //丢掉之前数据
						  } 			 
					};break;
			}	
		   i++;
		   if(i>=USART_BUFFERSIZE){i=0;};
		   if(flag==1){break;};
		  }
		 
		 if(flag)//找到一包有效指令
			{
			////////////////////////////////////////////////////
			
				 crcbakrx = &main_recbuffer[2];
				 crc_len=data_len+7;			  
				 crctest=crc16(crcbakrx,crc_len);
		     	   if((main_recbuffer[p_c0-4]!=(unsigned char)((crctest&0xff00)>>8))||(main_recbuffer[p_c0-3]!=(unsigned char)(crctest&0x00ff)))//校验位比对
				   {
				       usart1_start=i;//更新usart2_start指针
				//	printf("CRC ERROR\n") ;
						
				    } 
			       else 	   
				   {
				   ////////////////////////////////////////////////////
								usart1_start=i;//更新usart2_start指针
									   return(p_c0);
				   }
						  
			}
		 return(0);
		}












/****************************************************************************/
/*                                                                          */
/*              UART 中断服务函数                                           */
/*                                                                          */
/****************************************************************************/
void UARTIsr1()
{
 //   static unsigned int length = sizeof(txArray);
 //   static unsigned int count = 0;
//    unsigned char rxData1 = 0;
    unsigned int int_id = 0;

    // 确定中断源
  int_id = UARTIntStatus(SOC_UART_1_REGS);
//   INT_ID=int_id;
//   INT_ID_Sum =INT_ID_Sum+INT_ID;
//   INT_NUM++;

    // 清除 UART2 系统中断
    IntEventClear(SYS_INT_UART1_INT);
   //禁止 UART  中断
   UARTIntDisable(SOC_UART_1_REGS, (UART_INT_LINE_STAT	|  UART_INT_RXDATA_CTI));



// 确定中断源
//   int_id = UARTIntStatus(SOC_UART_1_REGS);

    // 接收错误
    if(UART_INTID_RX_LINE_STAT == int_id)
    {

        while(UARTRxErrorGet(SOC_UART_1_REGS))

        {
            // 从 RBR 读一个字节
            UARTCharGetNonBlocking(SOC_UART_1_REGS);
        }
    }

	/*
    // 发送中断
    if(UART_INTID_TX_EMPTY == int_id)
    {
        if(0 < length)
        {
            // 写一个字节到 THR
            UARTCharPutNonBlocking(SOC_UART_1_REGS, txArray[count]);
            length--;
            count++;
        }
        if(0 == length)
        {
            // 禁用发送中断
            UARTIntDisable(SOC_UART_1_REGS, UART_INT_TX_EMPTY);
        }
     }
*/
	
	// 接收中断
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
		

    // 清除 UART2 系统中断
//    IntEventClear(SYS_INT_UART1_INT);

   // 使能 UART2
  // UARTEnable(SOC_UART_1_REGS);

// 使能 UART2 中断
 UARTIntEnable(SOC_UART_1_REGS, (UART_INT_LINE_STAT	|  UART_INT_RXDATA_CTI));

    return;
}

void UARTIsr2()
{
    static unsigned int length = sizeof(txArray);
    static unsigned int count = 0;
    unsigned char rxData2 = 0;
    unsigned int int_id = 0;

    // 确定中断源
    int_id = UARTIntStatus(SOC_UART_2_REGS);

    // 清除 UART2 系统中断
    IntEventClear(SYS_INT_UART2_INT);

    // 发送中断
    if(UART_INTID_TX_EMPTY == int_id)
    {
        if(0 < length)
        {
            // 写一个字节到 THR
            UARTCharPutNonBlocking(SOC_UART_2_REGS, txArray[count]);
            length--;
            count++;
        }
        if(0 == length)
        {
            // 禁用发送中断
            UARTIntDisable(SOC_UART_2_REGS, UART_INT_TX_EMPTY);
        }
     }

    // 接收中断
    if(UART_INTID_RX_DATA == int_id)
    {
        rxData2= UARTCharGetNonBlocking(SOC_UART_2_REGS);
        UARTCharPutNonBlocking(SOC_UART_2_REGS, rxData2);
    }

    // 接收错误
    if(UART_INTID_RX_LINE_STAT == int_id)
    {
         INT_ID=UARTRxErrorGet(SOC_UART_2_REGS);
        while(UARTRxErrorGet(SOC_UART_2_REGS))
        {
            // 从 RBR 读一个字节
            UARTCharGetNonBlocking(SOC_UART_2_REGS);
        }
    }

    return;
}


