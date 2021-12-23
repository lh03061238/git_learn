#include "global_def.h"
//#include "nvocapi.h"
#include "TL6748.h"                 // 创龙 DSP6748 开发板相关声明
#include "hw_types.h"               // 宏命令
#include "hw_syscfg0_C6748.h"       // 系统配置模块寄存器
#include "soc_C6748.h"              // DSP C6748 外设寄存器
#include "edma_event.h"             // 直接内存访问事件
//#include "opusvad.h"                //静音检测
#include "gpio.h"                   // 通用输入输出口宏及设备抽象层函数声明

//void main_cmd_send(unsigned char CmdID, short int len,unsigned char *sendbuf);  for ST
 void main_cmd_send(unsigned char Cmd, short int len,unsigned char *sendbuf);

void main_cmd_receive_process(void);
//u8 wait_main_cmd_respond(void);
unsigned int crc16(unsigned char *buf, int len);
extern void UartDmaTransmitData(unsigned int baseAdd,unsigned int tccNum, unsigned int chNum,
								unsigned char *buffer, unsigned int buffLength);
//st
//int Vod_On =10;
//int Vod_Off=1;
//int Vod_En=0;

extern unsigned int Time34;
unsigned int timercnt,timercnt1,timercnt2;



static const short int crc16tab[256]= 
{
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};
//CRC校验
unsigned int crc16(unsigned char *buf, int len)
{
	int counter;
	short int crc = 0;
	for (counter = 0; counter < len; counter++)
	{
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++)&0x00FF];
	}
	return crc;
}


/////////////////////////////////////////////////////////
//  处理发送的命令
/////////////////////////////////////////////////////////

void main_cmd_send( unsigned char Cmd, short int len,unsigned char *sendbuf)
	{
	
	  short int i,counter = 0;
	//  unsigned char  sum = 0;
//	  unsigned long freq;
	   short int crcbak;
          
 	  while(send_finish_flag == 0);
	   send_finish_flag = 0;	               //确保DMA发送完成
	  
	/*
	  //命令字合法性判断 0000~04ff
	  if (CmdID >= 0x1fff)
	  {
		return ;
	  }
	*/
	  //标志字合法性判断 
	  /*
	  if (!((CmdFlag ==0x0A) || (CmdFlag ==0xff)|| (CmdFlag ==0x0f)|| (CmdFlag ==0x80)|| (CmdFlag ==0xf0)))
	   {
		  return ;
	   }
	*/
//	  cmd_sort=CmdFlag;
//	  cmdid_recv = 0x00; //接收到的ID初始化
	
	  //Head_H
	  main_sendbuffer[counter++] = 0x7e;
	  //head_L 
	  main_sendbuffer[counter++] = 0x7f;
	  //CmdACK
	  main_sendbuffer[counter++] = Cmd;
	  //chn
	  main_sendbuffer[counter++] =chn;
	  //in_type
	  main_sendbuffer[counter++] = out_type;
	  //out_type
	  main_sendbuffer[counter++] =in_type;
	  //frame
	  main_sendbuffer[counter++] =frame;
	  //len high
           main_sendbuffer[counter++] =((len&0xff00)>>8);
         //len low 
           main_sendbuffer[counter++] =(len&0x00ff);
	  //信令分类填充处理date
         for(i=0;i<len;i++)
         	{
         	    main_sendbuffer[counter++] =sendbuf[i];
         	}
	  
	  main_sendbuffer[counter++] = 0x00;  //crc high 暂时填0
	  main_sendbuffer[counter++] = 0x00;  //crc low  暂时填0
	  main_sendbuffer[counter++] = 0x9e;  //Rear_H
	  main_sendbuffer[counter++] = 0x9f;  //Rear_L
	
	
         crcbaktx=&main_sendbuffer[2];
	  crcbak=crc16(crcbaktx,counter-6); //crc
	  main_sendbuffer[counter-4]=(unsigned char)((crcbak&0xff00)>>8);//crc high
	  main_sendbuffer[counter-3]=(unsigned char)(crcbak&0x00ff);//crc low 
	  


//	  while(send_finish_flag == 0);
//	   send_finish_flag = 0;

	  UartDmaTransmitData(SOC_UART_1_REGS,EDMA3_CHA_UART1_TX, EDMA3_CHA_UART1_TX,
							   main_sendbuffer, counter);

/*	
          for(i=0;i<counter;i++)
	            {
				  // 写一个字节到 THR
				 UARTCharPut(SOC_UART_1_REGS, main_sendbuffer[i]);
			//	UARTCharPutNonBlocking(SOC_UART_1_REGS, main_sendbuffer[i]);		  
		     }
      
*/

//	  UsSend(1,main_sendbuffer,counter); 


	}


short int NVOC2FEC(short int *pcodeword)
{

    short int i;
             NVOC_FecEncoder(pcodeword, FECCodeWordTX);
	     for(i=0;i<FEC_SIZE;i++)
   	          {
   	             sendbuf[2*i] =(unsigned char)( (FECCodeWordTX[i]>>8) & 0x00ff);
	             sendbuf[2*i+1] =(unsigned char)(FECCodeWordTX[i]& 0x00ff);
   	          }
		 buf_len=144;
		 main_cmd_send(NVOC_FecEncoder_Ack, buf_len,sendbuf);
	        return 0;

   
			
}
short int NVOC2FEC_Opt(short int *pcodeword)
{
/*
    short int i;
             NVOC_FecEncoder(pcodeword, FECCodeWordTX);
	     for(i=0;i<FEC_SIZE;i++)
   	          {
   	             sendbuf[2*i] =(unsigned char)( (FECCodeWordTX[i]>>8) & 0x00ff);
	             sendbuf[2*i+1] =(unsigned char)(FECCodeWordTX[i]& 0x00ff);
   	          }
		 buf_len=144;
		 main_cmd_send(NVOC_FecEncoder_Ack, buf_len,sendbuf);
	        return 0;
*/
      short int i,j;
   //   unsigned char  Fec_code[9];
	  	
             NVOC_FecEncoder(pcodeword, FECCodeWordTX);

            for(i=0;i<FEC_byte;i++)
              {
                    sendbuf[i]=0;
                   for(j=0;j<8;j++)
                   	{
                           sendbuf[i]|=(unsigned char)((FECCodeWordTX[i*8+j]&0x0001)<<(7-j));
		          //  if(j<7)
		          //  	{
		          //        sendbuf[i]<<=1;
		           // 	}
		     	           
                   	}
            	}
 
		 buf_len=FEC_byte;
		 main_cmd_send(NVOC_FecEncoder_Ack_Opt, buf_len,sendbuf);
	        return 0;
}
short int NVOC2FEC_Opt_60(short int *pcodeword)
{

      short int i,j,k;
   //   unsigned char  Fec_code[9];
	
   for(k=0;k<3;k++)
   	{
          //  NVOC_FecEncoder((pcodeword+3*k), FECCodeWordTX);   for st
	        NVOC_FecEncoder(&(pcodeword[3*k]), FECCodeWordTX);
            for(i=0;i<FEC_byte;i++)
              {
                    sendbuf[i+FEC_byte*k]=0;
                   for(j=0;j<8;j++)
                   	{
                           sendbuf[i+FEC_byte*k]|=(unsigned char)((FECCodeWordTX[i*8+j]&0x0001)<<(7-j));
		       
		     	           
                   	}
            	}
   	}
		 buf_len=FEC_byte*3;
		 main_cmd_send(NVOC_FecEncoder_Ack_Opt_60, buf_len,sendbuf);
	        return 0;
}

short int FEC2NVOC(short int *pFECCodeWordRX)
{
     short int i;
     short int status;
     status=NVOC_FecDecoder(pFECCodeWordRX, decodeword); 
       sendbuf[0] =(unsigned char)( (status>>8) & 0x00ff);
       sendbuf[1] =(unsigned char)(status& 0x00ff);
	for(i=0;i<3;i++)
   	     {
   	           sendbuf[2+2*i] =(unsigned char)( (decodeword[i]>>8) & 0x00ff);
	           sendbuf[2+2*i+1] =(unsigned char)(decodeword[i]& 0x00ff);
   	     }
	buf_len=8;
       main_cmd_send(NVOC_FecDecoder_Ack, buf_len,sendbuf);
	return 0;

}
short int FEC2NVOC_Opt(short int *pFECCodeWordRX)
{
     short int i;
     short int status;
     status=NVOC_FecDecoder(pFECCodeWordRX, decodeword); 
       sendbuf[0] =(unsigned char)( (status>>8) & 0x00ff);
       sendbuf[1] =(unsigned char)(status& 0x00ff);
	for(i=0;i<3;i++)
   	     {
   	           sendbuf[2+2*i] =(unsigned char)( (decodeword[i]>>8) & 0x00ff);
	           sendbuf[2+2*i+1] =(unsigned char)(decodeword[i]& 0x00ff);
   	     }
	buf_len=8;
       main_cmd_send(NVOC_FecDecoder_Ack_Opt, buf_len,sendbuf);
	return 0;

}

short int FEC2NVOC_Opt_60(short int *pFECCodeWordRX)
{
     short int i,k;
     short int status;

	for(k=0;k<3;k++)
   {
     //status=NVOC_FecDecoder((pFECCodeWordRX+72*k), decodeword);        for st
     
	   status=NVOC_FecDecoder(&(pFECCodeWordRX[72*k]), decodeword);  
       sendbuf[2*k] =(unsigned char)( (status>>8) & 0x00ff);
       sendbuf[2*k+1] =(unsigned char)(status& 0x00ff);
	for(i=0;i<3;i++)
   	     {
   	           sendbuf[6+6*k+2*i] =(unsigned char)( (decodeword[i]>>8) & 0x00ff);
	           sendbuf[6+6*k+2*i+1] =(unsigned char)(decodeword[i]& 0x00ff);
   	     }

	}
	
	buf_len=24;
       main_cmd_send(NVOC_FecDecoder_Ack_Opt_60, buf_len,sendbuf);
	return 0;

}


short int Encoder_Fun(short int *pPcmIn, short int Mode, unsigned char encode_type)
{
   short int  i;
   short int status;
  cmd_ack=Encoder_Ack;
  
 

   status= NVOC_VoEncoder(VocEnDataMem, pPcmIn, codeword, Mode, NVOC_EncoderRate, &DtmfTone);
	 


       /*
	  if((encode_type==nvoc_2_2K)||(NVOC_EncoderRate==RATE2150))// 加密速率
   	      {
   	         //codeword 加密信息添加
   	         //codeword...
   	    
   	       }
	*/  
	  if((encode_type==nvoc_2_4K)||(encode_type==nvoc_2_2K))
	  {
	    sendbuf[0] =(unsigned char)( (DtmfTone>>8) & 0x00ff);
           sendbuf[1] =(unsigned char)(DtmfTone& 0x00ff);
           sendbuf[2] =(unsigned char)( (status>>8) & 0x00ff);
           sendbuf[3] =(unsigned char)(status& 0x00ff);
            for(i=0;i<3;i++)
   	        {
   	             sendbuf[4+2*i] =(unsigned char)( (codeword[i]>>8) & 0x00ff);
	             sendbuf[4+2*i+1] =(unsigned char)(codeword[i]& 0x00ff);
   	        }
             buf_len=10;
	     main_cmd_send(Encoder_Ack, buf_len,sendbuf);
	     return 0;
		
   	    }
          else if(encode_type==nvoc_3_6K) //  FEC encoder
  	    {

             NVOC_FecEncoder(codeword, FECCodeWordTX);
	 
	     for(i=0;i<FEC_SIZE;i++)
   	          {
   	             sendbuf[2*i] =(unsigned char)( (FECCodeWordTX[i]>>8) & 0x00ff);
	             sendbuf[2*i+1] =(unsigned char)(FECCodeWordTX[i]& 0x00ff);
   	          }
		 buf_len=144;
		 main_cmd_send(Encoder_Ack, buf_len,sendbuf);
    
	 
	        return 0;

  	     }
	else
	   {
		return 0;
	   }
		  
}


short int Encoder_Fun_Opt(short int *pPcmIn, short int Mode, unsigned char encode_type)
{
   short int  i,j;
   short int status;
  cmd_ack=Encoder_Ack;
  
 

   status= NVOC_VoEncoder(VocEnDataMem, pPcmIn, codeword, Mode, NVOC_EncoderRate, &DtmfTone);
	 


        /*
	  if((encode_type==nvoc_2_2K)||(NVOC_EncoderRate==RATE2150))// 加密速率
   	      {
   	         //codeword 加密信息添加
   	         //codeword...
   	    
   	       }
	  */
	  if((encode_type==nvoc_2_4K)||(encode_type==nvoc_2_2K))
	  {
	    sendbuf[0] =(unsigned char)( (DtmfTone>>8) & 0x00ff);
           sendbuf[1] =(unsigned char)(DtmfTone& 0x00ff);
           sendbuf[2] =(unsigned char)( (status>>8) & 0x00ff);
           sendbuf[3] =(unsigned char)(status& 0x00ff);
            for(i=0;i<3;i++)
   	        {
   	             sendbuf[4+2*i] =(unsigned char)( (codeword[i]>>8) & 0x00ff);
	             sendbuf[4+2*i+1] =(unsigned char)(codeword[i]& 0x00ff);
   	        }
             buf_len=10;
	     main_cmd_send(Pdt_Encoder_Ack_Opt, buf_len,sendbuf);
	     return 0;
		
   	    }
          else if(encode_type==nvoc_3_6K) //  FEC encoder
  	    {

             NVOC_FecEncoder(codeword, FECCodeWordTX);
	/*	 
	     for(i=0;i<FEC_SIZE;i++)
   	          {
   	             sendbuf[2*i] =(unsigned char)( (FECCodeWordTX[i]>>8) & 0x00ff);
	             sendbuf[2*i+1] =(unsigned char)(FECCodeWordTX[i]& 0x00ff);
   	          }
		 buf_len=144;
		 main_cmd_send(Encoder_Ack, buf_len,sendbuf);
    */
              for(i=0;i<FEC_byte;i++)
              {
                    sendbuf[i]=0;
                   for(j=0;j<8;j++)
                   	{
                           sendbuf[i]|=(unsigned char)((FECCodeWordTX[i*8+j]&0x0001)<<(7-j));
		         //   if(j<7)
		         //   	{
		          //        sendbuf[i]<<=1;
		          //  	}
		     	           
                   	}
            	}
 
		 buf_len=FEC_byte;
		 main_cmd_send(Pdt_Encoder_Ack_Opt, buf_len,sendbuf);

		 
	        return 0;

  	     }
	else
	   {
		return 0;
	   }
		  
}




short int NVOC2PCM(short int fec_status, short int *pdecodeword,unsigned char decode_type)
{
	  short int i;
	  short int status;
	  cmd_ack=Decoder_Ack;
/*
	   if((decode_type==nvoc_2_2K)||(NVOC_DecoderRate==RATE2150))// 加密速率
		{
					//pNvocin 加密信息添加
					//pNvocin...
			   
		}
*/
	   status=NVOC_VoDecoder(VocDeDataMem, pdecodeword, PcmOut, NVOC_DecoderRate, fec_status, &DtmfTone);

	    sendbuf[0] =(unsigned char)( (DtmfTone>>8) & 0x00ff);
           sendbuf[1] =(unsigned char)(DtmfTone& 0x00ff);
           sendbuf[2] =(unsigned char)( (status>>8) & 0x00ff);
           sendbuf[3] =(unsigned char)(status& 0x00ff);
	   for(i=0;i<FRAMESIZE;i++)
	         {
			sendbuf[4+2*i] =(unsigned char)( (PcmOut[i]>>8) & 0x00ff);
			sendbuf[4+2*i+1] =(unsigned char)(PcmOut[i]& 0x00ff);
		  }

	   buf_len=324;
	   main_cmd_send(Decoder_Ack, buf_len,sendbuf);
	   return 0;

}
short int NVOC2PCM_Opt(short int fec_status, short int *pdecodeword,unsigned char decode_type)
{
	  short int i;
	  short int status;
	  cmd_ack=Decoder_Ack;
/*
	   if((decode_type==nvoc_2_2K)||(NVOC_DecoderRate==RATE2150))// 加密速率
		{
					//pNvocin 加密信息添加
					//pNvocin...
			   
		}
*/
	   status=NVOC_VoDecoder(VocDeDataMem, pdecodeword, PcmOut, NVOC_DecoderRate, fec_status, &DtmfTone);

	   sendbuf[0] =(unsigned char)( (DtmfTone>>8) & 0x00ff);
           sendbuf[1] =(unsigned char)(DtmfTone& 0x00ff);
           sendbuf[2] =(unsigned char)( (status>>8) & 0x00ff);
           sendbuf[3] =(unsigned char)(status& 0x00ff);
	   for(i=0;i<FRAMESIZE;i++)
	         {
			sendbuf[4+2*i] =(unsigned char)( (PcmOut[i]>>8) & 0x00ff);
			sendbuf[4+2*i+1] =(unsigned char)(PcmOut[i]& 0x00ff);
		  }

	   buf_len=324;
	   main_cmd_send(Pdt_Decoder_Ack_Opt, buf_len,sendbuf);
	   return 0;

}

short int FEC2PCM(short int *pFECCodeWordRX, unsigned char decode_type)
{
      	
	  ChanState=NVOC_FecDecoder(pFECCodeWordRX, decodeword);
	  NVOC2PCM(ChanState,decodeword, decode_type);
     
	  return 0;
	
}
short int FEC2PCM_Opt(short int *pFECCodeWordRX, unsigned char decode_type)
{
      	
	  ChanState=NVOC_FecDecoder(pFECCodeWordRX, decodeword);
	  NVOC2PCM_Opt(ChanState,decodeword, decode_type);
     
	  return 0;
	
}

/*
int Vod_Test(    const short            pIn[]         ,                  
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
              if(vad_i>=Vod_On)
              {
                vad_flag=1;
                
           GPIOPinWrite(SOC_GPIO_0_REGS, 124, GPIO_PIN_HIGH);	// GP7[11]                  rev1
           GPIOPinWrite(SOC_GPIO_0_REGS, 123, GPIO_PIN_HIGH);	// GP7[10]                  rev2
              }
           }
          else
          {
              if(vad_flag==1)
              {
                 vad_j++;
                 if(vad_j==1)
                   { 
                    GPIOPinWrite(SOC_GPIO_0_REGS, 124, GPIO_PIN_LOW);	// GP7[11]                  rev1
                    GPIOPinWrite(SOC_GPIO_0_REGS, 123, GPIO_PIN_LOW);     // GP7[10]                  rev2	
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
                    if(vad_j==Vod_Off)
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



/////////////////////////////////////////////////////////
//  处理接收的命令
/////////////////////////////////////////////////////////

void main_cmd_receive_process(void)
	{
//	  short int CmdID;
//	  unsigned int counter = 0;
	  unsigned int i,j;
	  short int vomemsize,presult;


	  CmdFlag = main_recbuffer[2]; //提取命令类型
	  chn = main_recbuffer[3]; //通道号
	  in_type = main_recbuffer[4]; //输入语音编码类型
	  out_type = main_recbuffer[5]; //输出语音编码类型
	  frame = main_recbuffer[6]; //frame
	  data_len = (main_recbuffer[7] )*256 + main_recbuffer[8];//命令正确，提取命令ID
	  
	 switch (CmdFlag) 

	 {
		 /////////////////////////////////////////////////////////
		 ////					系统操作类					  ////	  
		 ////////////////////////////////////////////////////////
	      //编解码器初始化
		 case 0x00:
                       if(data_len==2)
                       	{
                       	    if(main_recbuffer[9]==0x00)    //编码器初始化
                       	    	{
                       	    	    NVOC_EncoderRate=main_recbuffer[10];
					    vomemsize=NVOC_VoEncoder_Init(VocEnDataMem, NVOC_EncoderRate);
						if (vomemsize > VOC_ENCODE_SIZE)
                                                {
                                                       sendbuf[0] =0x1;
	                                                buf_len=1;
                                                       main_cmd_send(Init_Ack, buf_len,sendbuf);
                                            
                                                 }
						else if(vomemsize!=1)
						//else
						    {
                                                       sendbuf[0] =0x0;
	                                                buf_len=1;
                                                       main_cmd_send(Init_Ack, buf_len,sendbuf);
                                            
                                                 }

			  
                       	    	}
				     else if(main_recbuffer[9]==0x01)//解码器初始化
				     	{
				     	   NVOC_DecoderRate=main_recbuffer[10];
					   vomemsize=NVOC_VoDecoder_Init(VocDeDataMem, NVOC_DecoderRate);
						if (vomemsize > VOC_DECODE_SIZE)
                                                {
                                                       sendbuf[0] =0x1;
	                                                buf_len=1;
                                                       main_cmd_send(Init_Ack, buf_len,sendbuf);
                                                 }
						else
						    {
                                                       sendbuf[0] =0x0;
	                                                buf_len=1;
                                                       main_cmd_send(Init_Ack, buf_len,sendbuf);
                                                 }	   
				     	}
                       	}

			 break; 
		 // NVOC编码
		 case 0x01:
                     if(data_len==322)
                       	{
                       	   
							 
                       	    NVOC_VoEncoder_Mode= (main_recbuffer[9] )*256 + main_recbuffer[10];
				  for(i=0; i<FRAMESIZE; i++)
				      {
				         PcmIn[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
				      }
				    /*
				   if(Vod_En)  //VOD检测
				   	{
				   	
                                       Vod_Test(PcmIn,20);  
				   	}
				   */
					 Encoder_Fun(PcmIn, NVOC_VoEncoder_Mode, out_type);	

					
                               }
			 break;
	 
		 //NVOC解码
		 case 0x02:
		//	if(data_len==8)
                       	{
                       	    if((in_type==nvoc_2_4K) ||(in_type==nvoc_2_2K))  
                       	    	{
                       	            ChanState= (main_recbuffer[9] )*256 + main_recbuffer[10];

					    for(i=0;i<3;i++)		
					    	{
					    decodeword[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
					    	}
					
									
					   NVOC2PCM(ChanState, decodeword,in_type);			
									
                       	    	}
				     if(in_type==nvoc_3_6K)
				     	{
				     	
				     	    for(i=0;i<FEC_SIZE;i++)		
					    	{
					            FECCodeWordRX[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
					    	}
                   
					
						/*	
							   for(i=0;i<FEC_byte;i++)
									{
									   for(j=0;j<8;j++)
										{
											FECCodeWordRX[8*i+j]=( (main_recbuffer[9+i]<<j)&0x80)>>7;
											 //  main_recbuffer[9+i]<<=1;	  
										}
									}
						*/		
							
		
					 FEC2PCM(FECCodeWordRX, in_type);		
				     	}


			        }
			 break;
	 
		 //NVOC设置AGC参数
		 case 0x03:
			 if (data_len == 6)
				 {
                                     set_fix_level= (main_recbuffer[9] )*256 + main_recbuffer[10];
					max_db_gain= (main_recbuffer[11] )*256 + main_recbuffer[12];
					min_db_gain= (main_recbuffer[13] )*256 + main_recbuffer[14];
					presult=NVOC_init_agc_dB(VocEnDataMem, set_fix_level,	max_db_gain,  min_db_gain);
                                     	 sendbuf[0]=(unsigned char)presult;
					 buf_len=1;						
				         main_cmd_send(Agc_Set_Ack, buf_len,sendbuf);									 
				 }
			
			 break;
		 
		 //生成单音或者DTMF 码流
		 case 0x04:
			 if (data_len == 6)
				 {
				     frametype= (main_recbuffer[9] )*256 + main_recbuffer[10];
				    FreqDtmfNO= (main_recbuffer[11] )*256 + main_recbuffer[12];
				    signal_Level= (main_recbuffer[13] )*256 + main_recbuffer[14];
				    NVOC_DTMF_Tone_encode(VocEnDataMem,frametype,FreqDtmfNO, signal_Level,codeword);
			           for(i=0;i<3;i++)
					{
					  sendbuf[2*i] =(unsigned char)( (codeword[i]>>8) & 0x00ff);
					  sendbuf[2*i+1] =(unsigned char)(codeword[i]& 0x00ff);
					}
					 buf_len=6;
					main_cmd_send(NVOC_DTMF_Tone_Encode_Ack, buf_len,sendbuf);
   
				 }
			 break;
		 
		 //生成单音或者DTMF 的PCM 数据
		 case 0x05:
			 if (data_len == 6)
				 {
				     frametype= (main_recbuffer[9] )*256 + main_recbuffer[10];
				    FreqDtmfNO= (main_recbuffer[11] )*256 + main_recbuffer[12];
				    signal_Level= (main_recbuffer[13] )*256 + main_recbuffer[14];	
                                  NVOC_DTMF_Tone_generate (VocDeDataMem,frametype, FreqDtmfNO,signal_Level,PcmOut);
				for(i=0;i<FRAMESIZE;i++)
					{
						 sendbuf[2*i] =(unsigned char)( (PcmOut[i]>>8) & 0x00ff);
						 sendbuf[2*i+1] =(unsigned char)(PcmOut[i]& 0x00ff);
					}
				        buf_len=FRAMESIZE*2;
					main_cmd_send(NVOC_DTMF_Tone_Generate_Ack, buf_len,sendbuf);
				 }
			 break;
		 
		 //声码器编解码命令参数设置
		 case 0x06:
			 if (data_len == 8)
				 {
				     NVOC_VoEn_cmd= (main_recbuffer[9] )*256 + main_recbuffer[10];
				     NVOC_VoEn_para= (main_recbuffer[11] )*256 + main_recbuffer[12];
				     NVOC_VoDe_cmd= (main_recbuffer[13] )*256 + main_recbuffer[14];	 
				     NVOC_VoDe_para= (main_recbuffer[13] )*256 + main_recbuffer[14];	
                                  NVOC_VoEnCommand(VocEnDataMem, NVOC_VoEn_cmd, NVOC_VoEn_para);
	                           NVOC_VoDeCommand(VocDeDataMem,NVOC_VoDe_cmd,NVOC_VoDe_para);	
				    buf_len=0;
				    main_cmd_send(Codec_Para_Set_Ack, buf_len,sendbuf);		   

			         }
			 break;
		 
		 // NVOC_RepeaterFecProcess（转发模式FEC 处理，避免在转发中经过FEC 解码再编码后丢失信道特性）
		 case 0x07:
			 if (data_len == 144)
				 {
				   for(i=0;i<FEC_SIZE;i++)		
					    	{
					            FECCodeWordRX[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
					    	}
				   NVOC_RepeaterFecProcess(FECCodeWordRX, FECCodeWordTX, decodeword);
				    for(i=0;i<FEC_SIZE;i++)
   	                                    {
   	                                        sendbuf[2*i] =(unsigned char)( (FECCodeWordTX[i]>>8) & 0x00ff);
	                                        sendbuf[2*i+1] =(unsigned char)(FECCodeWordTX[i]& 0x00ff);
   	                                     }
				    for(i=0;i<3;i++)
   	                                    {
   	                                        sendbuf[144+2*i] =(unsigned char)( (decodeword[i]>>8) & 0x00ff);
	                                        sendbuf[144+2*i+1] =(unsigned char)(decodeword[i]& 0x00ff);
   	                                     }
				     buf_len=150;
				     main_cmd_send(NVOC_RepeaterFecProcess_Ack, buf_len,sendbuf);		 
				 }
			 break;
		 
		 //NVOC_RepeaterSoftFecProcess（转发模式软解码FEC 处理，提高FEC 的抗误码性能）
		 case 0x08:
			 if (data_len == 74)
				 {
					Positive1Level= (main_recbuffer[9] )*256 + main_recbuffer[10];
					for(i=0;i<36;i++)		
					    	{
					            FskSoftInfoIn[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
					    	}
				ChanState=NVOC_RepeaterSoftFecProcess(Positive1Level,FskSoftInfoIn, FECCodeWordTX, decodeword);	

					for(i=0;i<FEC_SIZE;i++)
					      {
							sendbuf[2*i] =(unsigned char)( (FECCodeWordTX[i]>>8) & 0x00ff);
							sendbuf[2*i+1] =(unsigned char)(FECCodeWordTX[i]& 0x00ff);
		                             }
					 for(i=0;i<3;i++)
					      {
							  sendbuf[144+2*i] =(unsigned char)( (decodeword[i]>>8) & 0x00ff);
							  sendbuf[144+2*i+1] =(unsigned char)(decodeword[i]& 0x00ff);
					      }
					                 sendbuf[150] =(unsigned char)( (ChanState>>8) & 0x00ff);
							  sendbuf[151] =(unsigned char)(ChanState& 0x00ff);

					buf_len=152;
					main_cmd_send(NVOC_RepeaterSoftFecProcess_Ack, buf_len,sendbuf);	   



			        }
			 break;
		 
		 //NVOC_SoftFecDecoder（软解码FEC 处理，提高FEC 的抗误码性能）
		 case 0x09:
		 if (data_len == 74)
				 {
					Positive1Level= (main_recbuffer[9] )*256 + main_recbuffer[10];
					for(i=0;i<36;i++)		
					    	{
					            FskSoftInfoIn[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
					    	}
				     NVOC_SoftFecDecoder(Positive1Level,FskSoftInfoIn,decodeword);
				     for(i=0;i<3;i++)
   	                             {
   	                                 sendbuf[2*i] =(unsigned char)( (decodeword[i]>>8) & 0x00ff);
	                                 sendbuf[2*i+1] =(unsigned char)(decodeword[i]& 0x00ff);
   	                             }
	                           buf_len=6;
                                  main_cmd_send(NVOC_SoftFecDecoder_Ack, buf_len,sendbuf);
				 }
			 break;
		  //NVOC_VadSet（设置VAD 激活门限和保持时间）
		 case 0x0a:
			 if (data_len == 4)
				 {
				     VADThreshold= (main_recbuffer[9] )*256 + main_recbuffer[10];
				     VADpersistence= (main_recbuffer[11] )*256 + main_recbuffer[12];
			            VadSet(VocEnDataMem, VADThreshold, VADpersistence);	 
				    buf_len=0;
				    main_cmd_send(NVOC_VadSet_Ack, buf_len,sendbuf);	 
				 }
			
			 break;
		//NVOC_ReadVersion（查询声码器版本号）
		 case 0x0b:
		 	 if (data_len == 0)
				 {
				     NVOC_ReadVersion(vocid);
					for(i=0;i<20;i++)
					{
						 sendbuf[i] =(unsigned char)vocid[i];	
					} 
				   buf_len=20;
				    main_cmd_send(NVOC_ReadVersion_Ack, buf_len,sendbuf);	
					 
				 }
			
			 break;
		 
		 //NVOC_FecEncoder（FEC 编码，20ms 语音帧进行一次FEC）
		 case 0x0c:
			 if (data_len == 6)
				{
				
				  
				      for(i=0;i<3;i++)		
					    	{
					            codeword[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
					    	}
					NVOC2FEC(codeword);

					
	  
				}
			 break;
		//NVOC_FecDecoder（FEC 解码,20ms 语音帧进行一次解FEC）	 
		 case 0x0d:
		 	    
		 	if (data_len == 144)
				{
				
				       for(i=0;i<FEC_SIZE;i++)		
					    	{
					            FECCodeWordRX[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
					    	}
                                        FEC2NVOC(FECCodeWordRX);	
	 	                 }
		    /*		
			if (data_len == FEC_byte)
				{
				     for(i=0;i<FEC_byte;i++)
				     	{
				     	    for(j=0;j<8;j++)
				     	    	{
				                  FECCodeWordRX[8*i+j]=( (main_recbuffer[9+i]<<j)&0x80)>>7;
						 //  main_recbuffer[9+i]<<=1;	  
				     	    	}
				     	}
				     	FEC2NVOC_Opt(FECCodeWordRX);	
				   }
			*/
			 break;
		 case 0x0e:
			 break;
	         case 0x0f:
			 break;
	
	 
		 //MDT编解码器初始化
		 case 0xA0:
                       if(data_len==2)
                       	{
                       	    if(main_recbuffer[9]==0x00)    //编码器初始化
                       	    	{
                       	    	    Mdt_EncoderRate=main_recbuffer[10];
					    selp_encoder_init( Mdt_EncoderRate);			
					    
                                          sendbuf[0] =0x0;
	                                   buf_len=1;
                                          main_cmd_send(mdt_Init_Ack, buf_len,sendbuf);
                                                                       			  
                       	    	}
				     else if(main_recbuffer[9]==0x01)//解码器初始化
				     	{
				     	   Mdt_DecoderRate=main_recbuffer[10];
					    selp_decoder_init( Mdt_DecoderRate);			
					    
                                          sendbuf[0] =0x0;
	                                   buf_len=1;
                                          main_cmd_send(mdt_Init_Ack, buf_len,sendbuf);
				     	}
                       	}

			 break; 	 
	      // MDT编码
		 case 0xA1:
                     if(data_len==402)
                       	{
                       	   Mdt_VoEncoder_Mode= (main_recbuffer[9] )*256 + main_recbuffer[10];
				  for(i=0; i<FRAMESIZE_MDT; i++)
				      {
				         inbuf[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
				      }
				      /*
				      if(Vod_En)  //VOD检测
				      	{

				             Vod_Test(inbuf,25);
							 
				      	}
					*/  
					 selp_encoder_main( Mdt_VoEncoder_Mode,  inbuf,  code_word);

                      
                 for(i=0;i<60;i++)
   	                {
   	                  sendbuf[2*i] =(unsigned char)( (code_word[i]>>8) & 0x00ff);
	                  sendbuf[2*i+1] =(unsigned char)(code_word[i]& 0x00ff);
   	                }
                      buf_len=120;
            
                       /*
                               for(i=0;i<MDT_byte-1;i++)
                                    {
                                               sendbuf[i]=0;
                                            for(j=0;j<8;j++)
                   	                            {
                                                     sendbuf[i]|=(unsigned char)(code_word[i*8+j]&0x0001);
		                                       if(j<7)
		            	                           {
		                                               sendbuf[i]<<=1;
		            	                            }
		     	           
                   	                            }
					}
					sendbuf[MDT_byte-1]=0;				
					for(j=0;j<4;j++)
						{
                         sendbuf[MDT_byte-1]|=(unsigned char)(code_word[56+j]&0x0001);
		                  sendbuf[MDT_byte-1]<<=(7-j);   
                   	 }	
            	    buf_len=MDT_byte;

			*/					 
	                              main_cmd_send(mdt_Encoder_Ack, buf_len,sendbuf);
                               }
			 break;
		   //MDT解码
		 case 0xA2:
			if(data_len==124)
                       	{
					rate_select= (main_recbuffer[9] )*256 + main_recbuffer[10];
					crc_check= (main_recbuffer[11] )*256 + main_recbuffer[12];
                                     
					for(i=0;i<60;i++)		
					   {
					        decode_word[i] = (main_recbuffer[13+2*i] )*256+ (main_recbuffer[13+2*i+1]);
					    }
						
					/*
					for(i=0;i<MDT_byte-1;i++)
				     	{
				     	    for(j=0;j<8;j++)
				     	    	{
				                  decode_word[8*i+j]=( (main_recbuffer[13+i]<<j)&0x80)>>7;
						 //  main_recbuffer[9+i]<<=1;	  
				     	    	}
				     	}
                                           for(j=0;j<4;j++)
				     	    	{
				                  decode_word[56+j]=( (main_recbuffer[13+i]<<j)&0x80)>>7;   //main_recbuffer[20]
						 //        main_recbuffer[9+i]<<=1;	  
				     	    	}
                    */					   
                       	    	  selp_decoder_main( rate_select,  decode_word,  out_speech, crc_check);
					 for(i=0;i<FRAMESIZE_MDT;i++)
   	                                 {
   	                                     sendbuf[2*i] =(unsigned char)( (out_speech[i]>>8) & 0x00ff);
	                                     sendbuf[2*i+1] =(unsigned char)(out_speech[i]& 0x00ff);
   	                                }
                                     buf_len=FRAMESIZE_MDT*2;
					main_cmd_send(mdt_Decoder_Ack, buf_len,sendbuf);

			        }
			 break; 

                     //VOD set
                 /*      
		  case 0x10:
			if(data_len==3)
                       	{

					
					
					  Vod_En = main_recbuffer[9] ;
					
                                   Vod_On = main_recbuffer[10];
				      Vod_Off= main_recbuffer[11];
					 			
						
					 sendbuf[0] =0x0;
	                              buf_len=1;
                                      main_cmd_send(vod_Set_Ack, buf_len,sendbuf);				
                       	     
			        }
			 break; 
			*/ 
			 //////////////////////optimization PDT  MDT //////////////////////////////
			 // NVOC编码
			case 0x20:
				 if(data_len==322)
					{
										
					    NVOC_VoEncoder_Mode= (main_recbuffer[9] )*256 + main_recbuffer[10];
					   for(i=0; i<FRAMESIZE; i++)
						{
							PcmIn[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
						}
					     /*
					   if(Vod_En)	//VOD检测
						{
							Vod_Test(PcmIn,20);  
						}
					   */
						Encoder_Fun_Opt(PcmIn, NVOC_VoEncoder_Mode, out_type); 
			                 }
			 break;
			 //NVOC解码
			case 0x21:
			    //  if(data_len==8)
					{
					   if((in_type==nvoc_2_4K) ||(in_type==nvoc_2_2K))
						{
							ChanState= (main_recbuffer[9] )*256 + main_recbuffer[10];
			 
						       for(i=0;i<3;i++)		 
							{
							    decodeword[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
							 }
								 
							NVOC2PCM_Opt(ChanState, decodeword,in_type);		 
												 
						}
						       if(in_type==nvoc_3_6K)
							{
							/*
							for(i=0;i<FEC_SIZE;i++)	 
								{
									FECCodeWordRX[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
								}
							*/						 
							for(i=0;i<FEC_byte;i++)
							        {
									for(j=0;j<8;j++)
									{
										FECCodeWordRX[8*i+j]=( (main_recbuffer[9+i]<<j)&0x80)>>7;
									//  main_recbuffer[9+i]<<=1;	   
									 }
								}
											 
								  FEC2PCM_Opt(FECCodeWordRX, in_type);		 
							}
			 
			 
					}
			 break;
			 //NVOC_FecEncoder（FEC 编码，20ms 语音帧进行一次FEC）
			 case 0x22:
				if (data_len == 6)
				{
					for(i=0;i<3;i++) 	 
						{
                                                 codeword[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
						 }
						 NVOC2FEC_Opt(codeword);
				 }
			break;
			//NVOC_FecDecoder（FEC 解码,20ms 语音帧进行一次解FEC）	 
			case 0x23:
			/*
			if (data_len == 144)
			  {
				for(i=0;i<FEC_SIZE;i++)		
				{
					FECCodeWordRX[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
				}
			*/
				if (data_len == FEC_byte)
				{
					for(i=0;i<FEC_byte;i++)
					{
						for(j=0;j<8;j++)
						{
							FECCodeWordRX[8*i+j]=( (main_recbuffer[9+i]<<j)&0x80)>>7;
							 //  main_recbuffer[9+i]<<=1;	  
						}
					}
						FEC2NVOC_Opt(FECCodeWordRX);	
				}
			break;
			// MDT编码
			case 0x24:
			  if(data_len==402)
				{
				   Mdt_VoEncoder_Mode= (main_recbuffer[9] )*256 + main_recbuffer[10];
				   for(i=0; i<FRAMESIZE_MDT; i++)
					{
						inbuf[i] = (main_recbuffer[11+2*i] )*256+ (main_recbuffer[11+2*i+1]);
					}
				/*   
				if(Vod_En)  //VOD检测
					{
			                  Vod_Test(inbuf,25);
					}
				*/
					selp_encoder_main( Mdt_VoEncoder_Mode,	inbuf,	code_word);
					/*
					   for(i=0;i<60;i++)
						{
						  sendbuf[2*i] =(unsigned char)( (code_word[i]>>8) & 0x00ff);
						  sendbuf[2*i+1] =(unsigned char)(code_word[i]& 0x00ff);
						}
						buf_len=120;
					 */
					for(i=0;i<MDT_byte-1;i++)
					{
						sendbuf[i]=0;
					       for(j=0;j<8;j++)
						 {
						    sendbuf[i]|=(unsigned char)((code_word[i*8+j]&0x0001)<<(7-j));
				                //   if(j<7)
						//	{
						//		 sendbuf[i]<<=1;
			                       //    }
						   }
					}
					sendbuf[MDT_byte-1]=0;			   
					for(j=0;j<4;j++)
					{
					    sendbuf[MDT_byte-1]|=(unsigned char)((code_word[56+j]&0x0001)<<(7-j));
					
					} 
		                     buf_len=MDT_byte;
			              main_cmd_send(mdt_Encoder_Ack_Opt, buf_len,sendbuf);
                         }
	       break;
			 //MDT解码
	      case 0x25:
		 if(data_len==12)
			{
			    rate_select= (main_recbuffer[9] )*256 + main_recbuffer[10];
			    crc_check= (main_recbuffer[11] )*256 + main_recbuffer[12];
			/*
			for(i=0;i<60;i++) 	  
			{
				decode_word[i] = (main_recbuffer[13+2*i] )*256+ (main_recbuffer[13+2*i+1]);
			}
		   */  
	            for(i=0;i<MDT_byte-1;i++)
			 {
				for(j=0;j<8;j++)
			       {
				      decode_word[8*i+j]=( (main_recbuffer[13+i]<<j)&0x80)>>7;
					 //  main_recbuffer[9+i]<<=1; 	
		               }
		         }
	            for(j=0;j<4;j++)
			{
				decode_word[56+j]=( (main_recbuffer[13+i]<<j)&0x80)>>7;   //main_recbuffer[20]
			       //  main_recbuffer[9+i]<<=1; 	
		        }
                  selp_decoder_main( rate_select,  decode_word,  out_speech, crc_check);
                   for(i=0;i<FRAMESIZE_MDT;i++)
			{
					 sendbuf[2*i] =(unsigned char)( (out_speech[i]>>8) & 0x00ff);
					 sendbuf[2*i+1] =(unsigned char)(out_speech[i]& 0x00ff);
	              }
					 buf_len=FRAMESIZE_MDT*2;
		   
					  main_cmd_send(mdt_Decoder_Ack_Opt, buf_len,sendbuf);

		}
			   break; 
                   //NVOC_FecEncoder（FEC 编码，60ms 语音帧进行一次FEC）
			 case 0x26:
				if (data_len == 18)
				{
					for(i=0;i<9;i++) 	 
						{
                                                 codeword[i] = (main_recbuffer[9+2*i] )*256+ (main_recbuffer[9+2*i+1]);
						 }
						 NVOC2FEC_Opt_60(codeword);
				 }
			break;
			 //NVOC_FecDecoder（FEC 解码,60ms 语音帧进行一次解FEC）	 
			case 0x27:
		
				if (data_len == FEC_byte*3)
				{
					for(i=0;i<FEC_byte*3;i++)
					{
						for(j=0;j<8;j++)
						{
							FECCodeWordRX_60[8*i+j]=( (main_recbuffer[9+i]<<j)&0x80)>>7;
							 //  main_recbuffer[9+i]<<=1;	  
						}
					}
						FEC2NVOC_Opt_60(FECCodeWordRX_60);	
				}
			break;

			default:
				      
			break;

		
		 }


	}


/*

//给电台发送命令后，在一定时间内等待回应
//return: 1:回应正确，2：NAK回应，0：超时回应
u8 wait_main_cmd_respond(void)
{
  u8 ret = 0;
  main_timer2_enabled = 1;
  main_timer2_counter=TimCnt1ms+CMD_RESPOND_DELAY;//激活等待计数器
  main_overtime_finish = 0;
  while (!main_overtime_finish)
  {
    //遥控数据
    if (usart2_start != usart2_end)
    {
        length=usart2_process();
					  if(length!=0)
					  	{
	    main_cmd_receive_process();
					  	}
    }
    //等待的协议回应
    if (((cmdid_recv == cmdid_wait)&&(cmd_sort==REQ_CMD))||((cmdid_recv == cmdid_wait)&&(cmd_sort==SET_CMD)&&(cmdid_ack==TRUE)))
    {
      ret = 1;
      break;
    }
    //NAK回应
    if ((cmdid_recv == cmdid_wait)&&(cmd_sort==SET_CMD)&&(cmdid_ack==FALSE))
    {
      ret = 2;
      break;
    }
	
  }
  //超时
  if (main_overtime_finish)
  {
    ret = 3;
  }
  main_timer2_enabled = 0;
  cmdid_refresh=TRUE;
  return ret;
}
*/
