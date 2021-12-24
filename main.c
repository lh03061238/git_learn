/****************************************************************************/
/*                                                                          */
/*              ͨ���첽����2���ԣ��жϷ�ʽ��                               */
/*                                                                          */
/*              2014��04��20��                                              */
/*                                                                          */
/****************************************************************************/
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


#define GLOBAL_VAR

//#include "mdtapi.h"

#include "global_def.h"
#include "TL6748.h"                 // ���� DSP6748 �������������
#include "hw_types.h"               // ������
#include "hw_syscfg0_C6748.h"       // ϵͳ����ģ��Ĵ���
#include "soc_C6748.h"              // DSP C6748 ����Ĵ���
#include "psc.h"                    // ��Դ��˯�߿��ƺ꼰�豸����㺯������
#include "gpio.h"                   // ͨ����������ں꼰�豸����㺯������
#include "uart.h"                   // ͨ���첽���ں꼰�豸����㺯������
#include "interrupt.h"              // DSP C6748 �ж����Ӧ�ó���ӿں���������ϵͳ�¼��Ŷ���
#include "edma.h"                   // ֱ���ڴ���ʺ꼰�豸����㺯������
#include "edma_event.h"             // ֱ���ڴ�����¼�
#include "uartStdio.h"              // ���ڱ�׼��������ն˺�������
#include "string.h"
#include "i2c.h"                    // I2C �꼰�豸����㺯������
//#include "opusvad.h"                //�������





/****************************************************************************/
/*                                                                          */
/*              �궨��                                                      */
/*                                                                          */
/****************************************************************************/


// ����ϵ�
#define SW_BREAKPOINT     asm(" SWBP 0 ");
//#define MDT_previously




//#pragma  DATA_SECTION(usart1_end, ".MIPSUART")

//#pragma  DATA_SECTION(usart1_start, ".MIPSUART")

//#pragma  DATA_SECTION(USART1_RX_BUF, ".MIPSUART")



/* ���ñ����ṹ��������MIPStable ��*/
#pragma DATA_SECTION(VocEnDataMem, ".MIPStable")
#pragma DATA_SECTION(VocDeDataMem, ".MIPStable")
/* ���ñ�������ż����ַ����*/
#pragma DATA_ALIGN (VocEnDataMem, 2);
#pragma DATA_ALIGN (VocDeDataMem, 2);






/****************************************************************************/
/*                                                                          */
/*              ȫ�ֱ���                                                    */
/*                                                                          */
/****************************************************************************/


/****************************************************************************/
/*                                                                          */
/*              ��������                                                    */
/*                                                                          */
/****************************************************************************/
// ����ʹ������
void PSCInit(void);

//  �ܽŸ�������
void PinMuxSet();
// GPIO �ܽŸ�������
void GPIOBankPinMuxSet();
void GPIOPinDIRInit(void);

// GPIO �ܽų�ʼ��
void GPIOBankPinInit();
// DSP �жϳ�ʼ��
void InterruptInit(void);

// UART ��ʼ��
extern void UARTInit(void);

// UART �жϳ�ʼ��
extern void UARTInterruptInit();
// UART �жϷ�����
extern void UARTIsr(void);
extern unsigned long  usart1_process(void);

// EDMA3 �жϳ�ʼ��
extern void EDMA3InterruptInit(void);

// UARTDMA��������     
extern void UartDmaTransmitData(unsigned int baseAdd,unsigned int tccNum, unsigned int chNum,
								unsigned char *buffer, unsigned int buffLength);

// UARTDMA��䷢������
//extern void UartTransmitData(unsigned int tccNum, unsigned int chNum,
//                       volatile char *buffer, unsigned int buffLength);
// ���ڽ�������
extern void UartReceiveData(unsigned int tccNum, unsigned int chNum,
                       volatile char *buffer);
// �ص�����
extern void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc, unsigned int status);
extern void callback(unsigned int tccNum, unsigned int status);

// UART EDMA3 ��ʼ��
extern void EDMA3UARTInit();
// EDMA3 ��ʼ��
extern void EDMA3Initialize(void);
// EDMA3 �жϷ�����
extern void Edma3ComplHandlerIsr(void);
extern void Edma3CCErrHandlerIsr(void);
//��ʱ����
extern void delay_ms(unsigned int ms);
extern void delay_us(unsigned int us);

// ��ʱ�� / ��������ʼ��
extern void TimerInit(void);
// ��ʱ�� / �������жϳ�ʼ��
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
 short *y,//������
 short len,//���ݳ���
 short b,//֡��
 short n//�����������
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
/*              ������                                                      */
/*                                                                          */
/****************************************************************************/
int main(void)
{
// ST
//	short int rateselect,i;
	
//	rateselect=0;
	short int  i;

	
	PSCInit();                              // ����ʹ������
	
	PinMuxSet();                        // GPIO �ܽŸ�������

	InterruptInit();                    // DSP �жϳ�ʼ��

	UARTInit();                         // UART ��ʼ��

	UARTInterruptInit();         // UART �жϳ�ʼ��6��7

	TimerInit();                          // ��ʱ��0 ��������ʼ��

	TimerInterruptInit();            // ��ʱ�� 0 �������жϳ�ʼ��  11��12
	
	EDMA3InterruptInit();      // EDMA3 �жϳ�ʼ��   4��5

	EDMA3UARTInit();			// EDMA3 ��ʼ��

	IICInterruptInit();    //IIC�жϳ�ʼ��  10
	                       
	IICInit();      // IIC��ʼ��

	SPIInterruptInit();//SPI�жϳ�ʼ��  8

	SPIInit();  //spi��ʼ��

//	TimerWatchDogInit();  //���Ź���ʼ��

	send_finish_flag=1;


	// ���봮�� EDMA3 ����ͨ��
	   EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
							 EDMA3_CHA_UART1_TX, EDMA3_CHA_UART1_TX,
							 EVT_QUEUE_NUM);
	
	   // ע��ص�����
	   cb_Fxn[EDMA3_CHA_UART1_TX] = &callback;



	   usart1_end=usart1_start=0;



//delay_ms(500);
GPIOPinWrite(SOC_GPIO_0_REGS, 97, GPIO_PIN_HIGH);	  // LED

					for(i=0;i<50;i++)
			
						{
						        main_recbuffer[i]=i;
							// дһ���ֽڵ� THR
							UARTCharPut(SOC_UART_1_REGS, main_recbuffer[i]);
						}

/***********************************************************************/
selp_encoder_init( 0);
selp_decoder_init( 0);

rx_length = NVOC_VoEncoder_Init(VocEnDataMem, 0);
rx_length = NVOC_VoDecoder_Init(VocDeDataMem, 0);


                                          





//MDT����оƬ��ʼ��
				   {
					   softdata[0] = 0x0f;//
					   softdata[1] = 0x00;
					   softdata[2] = 0x00; 
					   i2c_error = _i2c_write(0x3d, 0x30, (unsigned char *)softdata, 3);
					   delay_ms(1);    //��ʱ1ms
					   i2c_error = _i2c_read(0x3d, 0x76, (unsigned char *)softdata, 8);
					   
					   delay_ms(1);    //��ʱ1ms
					   
					   for(i=0;i<8;i++)
					   {   xmt[i] = 0;}
			   
					   i2c_error = _i2c_write(0x3d, 0x80, (unsigned char *)xmt, 8);
					   delay_ms(1);    //��ʱ1ms
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
							   // дһ���ֽڵ� THR
							   UARTCharPut(SOC_UART_1_REGS, main_recbuffer[i]);
						   }

*/

			        main_cmd_receive_process();
                      


 
			    }

			}
	if(senddata2out[0]==1)	//��⵽senddata2out[0]Ϊ1�����������Ҫ��60֡�����
				{
				
					  senddata2out[0] = 0;
					  //����ֵerror_codeΪ0��ʾ��д��ȷ
						i2c_error = _i2c_write(0x3d, (unsigned char)senddata2out[1], (unsigned char *)&senddata2out[2], 8);				
						delay_ms(1);//��ʱ1ms	
						i2c_error = _i2c_read(0x3d, (unsigned char)senddata2out[1], (unsigned char *)&getdata2out[1], 10);
						getdata2out[0]=1;
					//	IICNum++;

					
				}	

	            

	   	}



	

	   // (ST)�ͷ� EDMA3 ͨ��  
/*	   EDMA3FreeChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
						  EDMA3_CHA_UART1_TX, EDMA3_TRIG_MODE_EVENT,
						  EDMA3_CHA_UART1_TX, EVT_QUEUE_NUM);
	
*/







}

/****************************************************************************/
/*                                                                          */
/*              PSC ��ʼ��                                                  */
/*                                                                          */
/****************************************************************************/
void PSCInit(void)
{
	// ����Ӧ����ģ���ʹ��Ҳ������ BootLoader �����

  // ʹ�� GPIO ģ��
   PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
   // ʹ�� UART1 ģ��
   PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_UART1, PSC_POWERDOMAIN_ALWAYS_ON,PSC_MDCTL_NEXT_ENABLE);
    // ʹ�� UART2 ģ��
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_UART2, PSC_POWERDOMAIN_ALWAYS_ON,PSC_MDCTL_NEXT_ENABLE);

   // ʹ�� EDMA3CC_0
    PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_CC0, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
    // ʹ�� EDMA3TC_0
    PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_TC0, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);

     // ʹ�� SPI ģ��
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_SPI1, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
}



/****************************************************************************/
/*                                                                          */
/*              GPIO �ܽŸ�������                                           */
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
/*              GPIO �ܽų�ʼ��                                             */
/*                                                                          */
/****************************************************************************/
void GPIOPinDIRInit(void)
{
	// ���� LED ��Ӧ�ܽ�Ϊ����ܽ�
    // OMAPL138 �� DSP C6748 ���� 144 �� GPIO
	// ����Ϊ���� GPIO BANK ��ʼ�ܽŶ�Ӧֵ
    // ��Χ 1-144
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
/*             �ܽŸ�������                                           */
/*                                                                          */
/****************************************************************************/
void PinMuxSet(void)
{
	// GPIO �ܽŸ�������
	GPIOBankPinMuxSet();
	GPIOPinDIRInit();
	// UART2 ��������
	UARTPinMuxSetup(1, FALSE);
	UARTPinMuxSetup(2, FALSE);	
	//I2C�ܽ�����
	I2CPinMuxSetup(0);
	//SPI1�ܽ�����
	SPIPinMuxSet();
}

/****************************************************************************/
/*                                                                          */
/*              DSP �жϳ�ʼ��                                              */
/*                                                                          */
/****************************************************************************/
void InterruptInit(void)
{
	// ��ʼ�� DSP �жϿ�����
	IntDSPINTCInit();

	// ʹ�� DSP ȫ���ж�
	IntGlobalEnable();
}

/****************************************************************************/
/*                                                                          */
/*              ��ʱ                                                        */
/*                                                                          */
/****************************************************************************/
void Delay(volatile unsigned int delay)
{
    while(delay--);
}

//123456

