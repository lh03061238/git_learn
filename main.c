/****************************************************************************/
/*                                                                          */
/*              通用异步串口2测试（中断方式）                               */
/*                                                                          */
/*              2014年04月20日                                              */
/*                                                                          */
/****************************************************************************/
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


#define GLOBAL_VAR

//#include "mdtapi.h"

#include "global_def.h"
#include "TL6748.h"                 // 创龙 DSP6748 开发板相关声明
#include "hw_types.h"               // 宏命令
#include "hw_syscfg0_C6748.h"       // 系统配置模块寄存器
#include "soc_C6748.h"              // DSP C6748 外设寄存器
#include "psc.h"                    // 电源与睡眠控制宏及设备抽象层函数声明
#include "gpio.h"                   // 通用输入输出口宏及设备抽象层函数声明
#include "uart.h"                   // 通用异步串口宏及设备抽象层函数声明
#include "interrupt.h"              // DSP C6748 中断相关应用程序接口函数声明及系统事件号定义
#include "edma.h"                   // 直接内存访问宏及设备抽象层函数声明
#include "edma_event.h"             // 直接内存访问事件
#include "uartStdio.h"              // 串口标准输入输出终端函数声明
#include "string.h"
#include "i2c.h"                    // I2C 宏及设备抽象层函数声明
//#include "opusvad.h"                //静音检测





/****************************************************************************/
/*                                                                          */
/*              宏定义                                                      */
/*                                                                          */
/****************************************************************************/


// 软件断点
#define SW_BREAKPOINT     asm(" SWBP 0 ");
//#define MDT_previously




//#pragma  DATA_SECTION(usart1_end, ".MIPSUART")

//#pragma  DATA_SECTION(usart1_start, ".MIPSUART")

//#pragma  DATA_SECTION(USART1_RX_BUF, ".MIPSUART")



/* 设置编解码结构体数组在MIPStable 段*/
#pragma DATA_SECTION(VocEnDataMem, ".MIPStable")
#pragma DATA_SECTION(VocDeDataMem, ".MIPStable")
/* 设置编解码参数偶数地址对齐*/
#pragma DATA_ALIGN (VocEnDataMem, 2);
#pragma DATA_ALIGN (VocDeDataMem, 2);






/****************************************************************************/
/*                                                                          */
/*              全局变量                                                    */
/*                                                                          */
/****************************************************************************/


/****************************************************************************/
/*                                                                          */
/*              函数声明                                                    */
/*                                                                          */
/****************************************************************************/
// 外设使能配置
void PSCInit(void);

//  管脚复用配置
void PinMuxSet();
// GPIO 管脚复用配置
void GPIOBankPinMuxSet();
void GPIOPinDIRInit(void);

// GPIO 管脚初始化
void GPIOBankPinInit();
// DSP 中断初始化
void InterruptInit(void);

// UART 初始化
extern void UARTInit(void);

// UART 中断初始化
extern void UARTInterruptInit();
// UART 中断服务函数
extern void UARTIsr(void);
extern unsigned long  usart1_process(void);

// EDMA3 中断初始化
extern void EDMA3InterruptInit(void);

// UARTDMA发送数据     
extern void UartDmaTransmitData(unsigned int baseAdd,unsigned int tccNum, unsigned int chNum,
								unsigned char *buffer, unsigned int buffLength);

// UARTDMA填充发送数据
//extern void UartTransmitData(unsigned int tccNum, unsigned int chNum,
//                       volatile char *buffer, unsigned int buffLength);
// 串口接收数据
extern void UartReceiveData(unsigned int tccNum, unsigned int chNum,
                       volatile char *buffer);
// 回调函数
extern void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc, unsigned int status);
extern void callback(unsigned int tccNum, unsigned int status);

// UART EDMA3 初始化
extern void EDMA3UARTInit();
// EDMA3 初始化
extern void EDMA3Initialize(void);
// EDMA3 中断服务函数
extern void Edma3ComplHandlerIsr(void);
extern void Edma3CCErrHandlerIsr(void);
//延时函数
extern void delay_ms(unsigned int ms);
extern void delay_us(unsigned int us);

// 定时器 / 计数器初始化
extern void TimerInit(void);
// 定时器 / 计数器中断初始化
extern void TimerInterruptInit(void);
extern void main_cmd_receive_process(void);


extern void IICInit(void);
extern void IICInterruptInit(void);
extern unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo);
extern unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo);

extern void TimerWatchDogInit(void);

extern void SPIInterruptInit(void);
extern void SPIInit(void);

extern void SPIPinMuxSet(void);

extern int do_mem_mtest(unsigned long start_addr, unsigned long end_addr, unsigned int iteration_limit);

extern void GPIOBank6Pin0PinMuxSetup(void);
extern void GPIOBank7Pin12PinMuxSetup(void);
extern void GPIOBank7Pin13PinMuxSetup(void);

extern void GPIOBank7Pin10PinMuxSetup(void);
extern void GPIOBank7Pin11PinMuxSetup(void);



volatile char enter[] = "Tronlong UART2 EDMA3 Application......\n\rPlease Enter 20 bytes from keyboard\r\n";
volatile char buffer[RX_BUFFER_SIZE];
unsigned int buffLength = 0;
unsigned int   frame_sum;


char infilename[50] = "mix2.pcm";
char outfilename[50] = "mix2_2400.pcm";
char packname[50] = "code_MIX2_2400.o";


int vod_on =10;
int vod_off=1;
int vod_en=1;

extern unsigned int Time34;
extern unsigned int timercnt,timercnt1,timercnt2;




void Printintofile(
 FILE *p,
 short *y,//数据流
 short len,//数据长度
 short b,//帧数
 short n//单行输出个数
 )
{
 static short comparecnt=0;
 short d,j;
 
 if(comparecnt < b)
 { 
  comparecnt++;
  fprintf(p,"//0000%d\n",comparecnt);
  for(d=0;d<len;d+=n)
  {
   for(j=d;j<(n+d);j++)
   {    
    fprintf(p,"0x%08x,",y[j]);
   }   
   fprintf(p,"\n");
  }
 }else
 {
  fclose(p);
  printf("oooooooooooooooooover!\n");
  //while(1);
 }
}


//int vod_test(int vod_on ,int vod_off)
/*
vod_test(    const short            pIn[]         ,                  // I    PCM input                                   
                              int         frame_length_ms
                        )
{

static int vad_j=0;
static int vad_i=0;
static int vad_flag=0;

     if(silk_VAD_Get(pIn,frame_length_ms)==0)
          {
              vad_j=0; 
              vad_i++;
              if(vad_i>=vod_on)
              {
                vad_flag=1;
                
                 
              }
           }
          else
          {
              if(vad_flag==1)
              {
                 vad_j++;
                 if(vad_j==1)
                   { 
                    vad_flag=0;
                    vad_j = 0; 
                    vad_i = 0;
                   }
              }
              else
              {
                
                 if(vad_i>0)
                 {
                   
                    vad_j++;
                    if(vad_j==vod_off)
                   { 
                     
                     vad_i = 0;
                     vad_j = 0;
                   }
                 }
              }
                
          }
          
          
     return   (vad_flag);
      
}

*/


/****************************************************************************/
/*                                                                          */
/*              主函数                                                      */
/*                                                                          */
/****************************************************************************/
int main(void)
{
// ST
//	short int rateselect,i;
	
//	rateselect=0;
	short int  i;

	
	PSCInit();                              // 外设使能配置
	
	PinMuxSet();                        // GPIO 管脚复用配置

	InterruptInit();                    // DSP 中断初始化

	UARTInit();                         // UART 初始化

	UARTInterruptInit();         // UART 中断初始化6、7

	TimerInit();                          // 定时器0 计数器初始化

	TimerInterruptInit();            // 定时器 0 计数器中断初始化  11、12
	
	EDMA3InterruptInit();      // EDMA3 中断初始化   4、5

	EDMA3UARTInit();			// EDMA3 初始化

	IICInterruptInit();    //IIC中断初始化  10
	                       
	IICInit();      // IIC初始化

	SPIInterruptInit();//SPI中断初始化  8

	SPIInit();  //spi初始化

//	TimerWatchDogInit();  //看门狗初始化

	send_finish_flag=1;


	// 申请串口 EDMA3 发送通道
	   EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
							 EDMA3_CHA_UART1_TX, EDMA3_CHA_UART1_TX,
							 EVT_QUEUE_NUM);
	
	   // 注册回调函数
	   cb_Fxn[EDMA3_CHA_UART1_TX] = &callback;



	   usart1_end=usart1_start=0;



//delay_ms(500);
GPIOPinWrite(SOC_GPIO_0_REGS, 97, GPIO_PIN_HIGH);	  // LED

					for(i=0;i<50;i++)
			
						{
						        main_recbuffer[i]=i;
							// 写一个字节到 THR
							UARTCharPut(SOC_UART_1_REGS, main_recbuffer[i]);
						}

/***********************************************************************/
selp_encoder_init( 0);
selp_decoder_init( 0);

rx_length = NVOC_VoEncoder_Init(VocEnDataMem, 0);
rx_length = NVOC_VoDecoder_Init(VocDeDataMem, 0);


                                          





//MDT加密芯片初始化
				   {
					   softdata[0] = 0x0f;//
					   softdata[1] = 0x00;
					   softdata[2] = 0x00; 
					   i2c_error = _i2c_write(0x3d, 0x30, (unsigned char *)softdata, 3);
					   delay_ms(1);    //延时1ms
					   i2c_error = _i2c_read(0x3d, 0x76, (unsigned char *)softdata, 8);
					   
					   delay_ms(1);    //延时1ms
					   
					   for(i=0;i<8;i++)
					   {   xmt[i] = 0;}
			   
					   i2c_error = _i2c_write(0x3d, 0x80, (unsigned char *)xmt, 8);
					   delay_ms(1);    //延时1ms
					   i2c_error = _i2c_read(0x3d, 0x80, (unsigned char *)rcv, 10);
					   
				} 

	   while(1)
	   {   
	   
	 
	//if((usart1_end-usart1_start)>cmd_rear_offset)
	    if(usart1_end!=usart1_start)
		   {

	         rx_length=usart1_process();
			 if(rx_length!=0)
			   {
/*
		               while(send_finish_flag == 0);
	                      send_finish_flag = 0;

	                       UartDmaTransmitData(SOC_UART_1_REGS,EDMA3_CHA_UART1_TX, EDMA3_CHA_UART1_TX,
							   main_recbuffer, rx_length);	   
*/

/*
			   for(i=0;i<rx_length;i++)
			   
						   {
							   // 写一个字节到 THR
							   UARTCharPut(SOC_UART_1_REGS, main_recbuffer[i]);
						   }

*/

			        main_cmd_receive_process();
                      


 
			    }

			}
	if(senddata2out[0]==1)	//检测到senddata2out[0]为1，下面操作需要在60帧内完成
				{
				
					  senddata2out[0] = 0;
					  //返回值error_code为0表示读写正确
						i2c_error = _i2c_write(0x3d, (unsigned char)senddata2out[1], (unsigned char *)&senddata2out[2], 8);				
						delay_ms(1);//延时1ms	
						i2c_error = _i2c_read(0x3d, (unsigned char)senddata2out[1], (unsigned char *)&getdata2out[1], 10);
						getdata2out[0]=1;
					//	IICNum++;

					
				}	

	            

	   	}



	

	   // (ST)释放 EDMA3 通道  
/*	   EDMA3FreeChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
						  EDMA3_CHA_UART1_TX, EDMA3_TRIG_MODE_EVENT,
						  EDMA3_CHA_UART1_TX, EVT_QUEUE_NUM);
	
*/







}

/****************************************************************************/
/*                                                                          */
/*              PSC 初始化                                                  */
/*                                                                          */
/****************************************************************************/
void PSCInit(void)
{
	// 对相应外设模块的使能也可以在 BootLoader 中完成

  // 使能 GPIO 模块
   PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
   // 使能 UART1 模块
   PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_UART1, PSC_POWERDOMAIN_ALWAYS_ON,PSC_MDCTL_NEXT_ENABLE);
    // 使能 UART2 模块
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_UART2, PSC_POWERDOMAIN_ALWAYS_ON,PSC_MDCTL_NEXT_ENABLE);

   // 使能 EDMA3CC_0
    PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_CC0, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
    // 使能 EDMA3TC_0
    PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_TC0, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);

     // 使能 SPI 模块
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_SPI1, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
}



/****************************************************************************/
/*                                                                          */
/*              GPIO 管脚复用配置                                           */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinMuxSet(void)
{

GPIOBank6Pin0PinMuxSetup();   //GP6[0]                      LED
GPIOBank7Pin12PinMuxSetup(); // GP7[12]                  PDT_SDA
GPIOBank7Pin13PinMuxSetup(); // GP7[13]                 PDT_SCL

GPIOBank7Pin10PinMuxSetup(); // GP7[10]                 rev2
GPIOBank7Pin11PinMuxSetup(); // GP7[11]                 rev1

	
}


/****************************************************************************/
/*                                                                          */
/*              GPIO 管脚初始化                                             */
/*                                                                          */
/****************************************************************************/
void GPIOPinDIRInit(void)
{
	// 配置 LED 对应管脚为输出管脚
    // OMAPL138 及 DSP C6748 共有 144 个 GPIO
	// 以下为各组 GPIO BANK 起始管脚对应值
    // 范围 1-144
	// GPIO0[0] 1
    // GPIO1[0] 17
	// GPIO2[0] 33
    // GPIO3[0] 49
	// GPIO4[0] 65
    // GPIO5[0] 81
	// GPIO6[0] 97
	// GPIO7[0] 113
	// GPIO8[0] 129

    GPIODirModeSet(SOC_GPIO_0_REGS, 97, GPIO_DIR_OUTPUT);  //GP6[0]                      LED
//    GPIODirModeSet(SOC_GPIO_0_REGS, 125, GPIO_DIR_OUTPUT);  // GP7[12]                  PDT_SDA
    GPIODirModeSet(SOC_GPIO_0_REGS, 126, GPIO_DIR_OUTPUT);    // GP7[13]                 PDT_SCL

    GPIODirModeSet(SOC_GPIO_0_REGS, 123, GPIO_DIR_OUTPUT);    // GP7[10]                 rev2	
    GPIODirModeSet(SOC_GPIO_0_REGS, 124, GPIO_DIR_OUTPUT);    // GP7[11]                rev1
}








/****************************************************************************/
/*                                                                          */
/*             管脚复用配置                                           */
/*                                                                          */
/****************************************************************************/
void PinMuxSet(void)
{
	// GPIO 管脚复用配置
	GPIOBankPinMuxSet();
	GPIOPinDIRInit();
	// UART2 禁用流控
	UARTPinMuxSetup(1, FALSE);
	UARTPinMuxSetup(2, FALSE);	
	//I2C管脚配置
	I2CPinMuxSetup(0);
	//SPI1管脚配置
	SPIPinMuxSet();
}

/****************************************************************************/
/*                                                                          */
/*              DSP 中断初始化                                              */
/*                                                                          */
/****************************************************************************/
void InterruptInit(void)
{
	// 初始化 DSP 中断控制器
	IntDSPINTCInit();

	// 使能 DSP 全局中断
	IntGlobalEnable();
}

/****************************************************************************/
/*                                                                          */
/*              延时                                                        */
/*                                                                          */
/****************************************************************************/
void Delay(volatile unsigned int delay)
{
    while(delay--);
}

//123456

