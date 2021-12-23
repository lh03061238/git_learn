#include <stdio.h>
#include "hw_types.h"				// HWREG(x)
#include "gpio.h"					// GPIO_DIR_OUTPUT
#include "TL6748.h"
#include "soc_C6748.h"
#include "hw_syscfg0_C6748.h"
#include "interrupt.h"              // DSP C6748 中断相关应用程序接口函数声明及系统事件号定义
#include "i2c.h"                    // I2C 宏及设备抽象层函数声明


void GPIOBank6Pin0PinMuxSetup(void);
void GPIOBank7Pin12PinMuxSetup(void);
void GPIOBank7Pin13PinMuxSetup(void);

void GPIOBank7Pin10PinMuxSetup(void);      //w3-rev2 
void GPIOBank7Pin11PinMuxSetup(void);      //  v1-rev1            



//void delay_us(unsigned int number);
void GPIO_SCL_RSET(void);
void GPIO_SCL_SET(void);
void GPIO_SDA_SET_IN(void);
void GPIO_SDA_SET_OUT(void);
void GPIO_SDA_RSET(void);
void GPIO_SDA_SET(void);
unsigned char GPIO_SDA_RE(void);

void IICInit(void);
void IICInterruptInit(void);
void IICIsr(void);
void I2CByteWrite(unsigned int Address, unsigned char Data);
unsigned char I2CByteRead(unsigned int Address);
unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo);
unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo);
void IICSend(unsigned int iic, unsigned int dataCnt);
void IICReceive(unsigned int iic, unsigned int dataCnt);


/****************************************************************************/
/// MDTIC 地址
#define MDTIC_ADDRESS   0x3D



#define PINMUX19_GPIO6_0_ENABLE   (SYSCFG_PINMUX19_PINMUX19_27_24_GPIO6_0  << \
                                    SYSCFG_PINMUX19_PINMUX19_27_24_SHIFT)

#define PINMUX16_GPIO7_12_ENABLE  (SYSCFG_PINMUX16_PINMUX16_23_20_GPIO7_12 << \
                                    SYSCFG_PINMUX16_PINMUX16_23_20_SHIFT)

#define PINMUX16_GPIO7_13_ENABLE  (SYSCFG_PINMUX16_PINMUX16_19_16_GPIO7_13  << \
                                    SYSCFG_PINMUX16_PINMUX16_19_16_SHIFT)

//v1-rev1
#define PINMUX16_GPIO7_11_ENABLE  (SYSCFG_PINMUX16_PINMUX16_27_24_GPIO7_11 << \
                                    SYSCFG_PINMUX16_PINMUX16_27_24_SHIFT)
//w3-rev2
#define PINMUX16_GPIO7_10_ENABLE  (SYSCFG_PINMUX16_PINMUX16_31_28_GPIO7_10  << \
                                    SYSCFG_PINMUX16_PINMUX16_31_28_SHIFT)


/****************************************************************************/
/*                                                                          */
/*              全局变量                                                    */
/*                                                                          */
/****************************************************************************/
static volatile unsigned char slaveData[20];

static volatile unsigned int dataIdx = 0;
static volatile unsigned int txCompFlag = 1;
static volatile unsigned int AckRolling = 0;


void GPIOBank6Pin0PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(19)) &
                 ~(SYSCFG_PINMUX19_PINMUX19_27_24));

    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(19)) =
         (PINMUX19_GPIO6_0_ENABLE | savePinmux);
}



void GPIOBank7Pin10PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) &
                 ~(SYSCFG_PINMUX16_PINMUX16_31_28));

    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) =
         (PINMUX16_GPIO7_10_ENABLE | savePinmux);
}

void GPIOBank7Pin11PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) &
                 ~(SYSCFG_PINMUX16_PINMUX16_27_24));

    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) =
         (PINMUX16_GPIO7_11_ENABLE | savePinmux);
}


void GPIOBank7Pin12PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) &
                 ~(SYSCFG_PINMUX16_PINMUX16_23_20));

    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) =
         (PINMUX16_GPIO7_12_ENABLE | savePinmux);
}

void GPIOBank7Pin13PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) &
                 ~(SYSCFG_PINMUX16_PINMUX16_19_16));

    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(16)) =
         (PINMUX16_GPIO7_13_ENABLE | savePinmux);
}


/*************时钟线置低*******************/
void GPIO_SCL_RSET(void)
{
	GPIOPinWrite(SOC_GPIO_0_REGS, 126, GPIO_PIN_LOW);
	//GPIO_setOutput(GPIO_BANK8, GPIO_PIN6,OUTPUT_LOW);//GPIO_FSET(IODATA,IO7D,0);
}

/*************时钟线置高*******************/
void GPIO_SCL_SET(void)
{
	GPIOPinWrite(SOC_GPIO_0_REGS, 126, GPIO_PIN_HIGH);
	//GPIO_setOutput(GPIO_BANK8, GPIO_PIN6,OUTPUT_HIGH);//GPIO_FSET(IODATA,IO7D,1);
}

/*************数据线设置为输入*******************/
void GPIO_SDA_SET_IN(void)
{
	GPIODirModeSet(SOC_GPIO_0_REGS, 125, GPIO_DIR_INPUT);//GPIO8[5] 5:SDA
	//GPIO_setDir(GPIO_BANK8, GPIO_PIN5, GPIO_INPUT);//GPIO_RAOI(IODIR,0xBF,NULL,NULL);
}

/*************数据线设置为输出*******************/
void GPIO_SDA_SET_OUT(void)
{
	GPIODirModeSet(SOC_GPIO_0_REGS, 125, GPIO_DIR_OUTPUT);//GPIO8[5] 5:SDA
	//GPIO_setDir(GPIO_BANK8, GPIO_PIN5, GPIO_OUTPUT);//GPIO_RAOI(IODIR,NULL,0xC0,NULL);
}

/*************数据线置低*******************/
void GPIO_SDA_RSET(void)
{
	GPIOPinWrite(SOC_GPIO_0_REGS, 125, GPIO_PIN_LOW);
	//GPIO_setOutput(GPIO_BANK8, GPIO_PIN5,OUTPUT_LOW);//GPIO_FSET(IODATA,IO6D,0);
}

/*************数据线置高*******************/
void GPIO_SDA_SET(void)
{
	GPIOPinWrite(SOC_GPIO_0_REGS, 125, GPIO_PIN_HIGH);
	//GPIO_setOutput(GPIO_BANK8, GPIO_PIN5,OUTPUT_HIGH);//GPIO_FSET(IODATA,IO6D,1);
}

/*************数据线读数据*******************/
unsigned char GPIO_SDA_RE(void)
{	

	return GPIOPinRead(SOC_GPIO_0_REGS, 125);
  //return GPIO_ReadIO(GPIO_BANK8, GPIO_PIN5);
}

/*************us延时函数*******************/
/*
void delay_us(unsigned int a)
{
	volatile long int i,j=0,k;

	for(i=a*22;i>0;i--)
		j -= 1;
}
*/

/*************************I2C总线相关函数*********************/


/*              IIC 中断初始化                                              */
/*                                                                          */
/****************************************************************************/
void IICInterruptInit(void)
{
    IntRegister(C674X_MASK_INT10, IICIsr);
    IntEventMap(C674X_MASK_INT10, SYS_INT_I2C0_INT);
	IntEnable(C674X_MASK_INT10);
}

/****************************************************************************/
/*                                                                          */
/*              IIC 初始化                                                  */
/*                                                                          */
/****************************************************************************/
void IICInit(void)
{
    //IIC 复位 / 禁用
    I2CMasterDisable(SOC_I2C_0_REGS);

    // 配置总线速度为 80KHz
    I2CMasterInitExpClk(SOC_I2C_0_REGS, 24000000, 8000000, 80000);

    // 设置从设备地址
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, MDTIC_ADDRESS);

    // IIC 使能
    I2CMasterEnable(SOC_I2C_0_REGS);
}
/****************************************************************************/
/*                                                                          */
/*              IIC 中断服务函数                                            */
/*                                                                          */
/****************************************************************************/
void IICIsr(void)

{
    volatile unsigned int intCode = 0;

    // 取得中断代码
    intCode = I2CInterruptVectorGet(SOC_I2C_0_REGS);

    while(intCode!=0)
    {
    	// 清除中断事件
    	IntEventClear(SYS_INT_I2C0_INT);

		if (intCode == I2C_INTCODE_TX_READY)
		{
		  I2CMasterDataPut(SOC_I2C_0_REGS, slaveData[dataIdx]);
		  dataIdx++;
		}

		if(intCode == I2C_INTCODE_RX_READY)
		{
		  slaveData[dataIdx] = I2CMasterDataGet(SOC_I2C_0_REGS);
		  dataIdx++;
		}

		if (intCode == I2C_INTCODE_STOP)
		{
		 I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY |
											   I2C_INT_DATA_READY |
											   I2C_INT_NO_ACK |
											   I2C_INT_STOP_CONDITION);
		  txCompFlag = 0;
		}

		if (intCode == I2C_INTCODE_NACK)
		{
		 I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY |
											   I2C_INT_DATA_READY |
											   I2C_INT_NO_ACK |
											   I2C_INT_STOP_CONDITION);
		 // 产生停止位
		 I2CMasterStop(SOC_I2C_0_REGS);

		 I2CStatusClear(SOC_I2C_0_REGS, I2C_CLEAR_STOP_CONDITION);

		 // 清除中断
		 IntEventClear(SYS_INT_I2C0_INT);
		 txCompFlag = 0;
		 AckRolling = 1;
		}

		if (I2CMasterIntStatus(SOC_I2C_0_REGS) & I2C_ICSTR_NACKSNT)
		{
		 I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY |
											   I2C_INT_DATA_READY |
											   I2C_INT_NO_ACK |
											   I2C_INT_STOP_CONDITION);

		 // 产生停止位
		 I2CMasterStop(SOC_I2C_0_REGS);

		 I2CStatusClear(SOC_I2C_0_REGS, (I2C_CLEAR_NO_ACK_SENT |
										 I2C_CLEAR_STOP_CONDITION));

		 // 清除中断
		 IntEventClear(SYS_INT_I2C0_INT);
		 txCompFlag = 0;
		}

		intCode = I2CInterruptVectorGet(SOC_I2C_0_REGS);
    }
}
/****************************************************************************/
/*                                                                          */
/*              IIC 发送数据                                                */
/*                                                                          */
/****************************************************************************/
// iic IIC 模块基址
// dataCnt 数据大小
// 阻塞函数
void IICSend(unsigned int iic, unsigned int dataCnt)
{
    txCompFlag = 1;
    dataIdx = 0;

    while(I2CMasterBusBusy(iic));

    I2CSetDataCount(iic, dataCnt);

    I2CMasterControl(iic, I2C_CFG_MST_TX | I2C_CFG_STOP);

    I2CMasterIntEnableEx(iic, I2C_INT_TRANSMIT_READY | I2C_INT_STOP_CONDITION | I2C_INT_NO_ACK);

    I2CMasterStart(iic);

    // 等待数据发送完成
    while(txCompFlag);

    while(I2CMasterBusBusy(iic));
}

/****************************************************************************/
/*                                                                          */
/*              IIC 接收数据                                                */
/*                                                                          */
/****************************************************************************/
// iic IIC 模块基址
// dataCnt 数据大小
// 阻塞函数
void IICReceive(unsigned int iic, unsigned int dataCnt)
{
    txCompFlag = 1;
    dataIdx = 0;

    while(I2CMasterBusBusy(SOC_I2C_0_REGS));

    I2CSetDataCount(SOC_I2C_0_REGS, dataCnt);

    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_RX | I2C_CFG_STOP);

    I2CMasterIntEnableEx(SOC_I2C_0_REGS, I2C_INT_DATA_READY | I2C_INT_STOP_CONDITION | I2C_INT_NO_ACK);

    I2CMasterStart(SOC_I2C_0_REGS);

    // 等待数据接收完成
    while(txCompFlag);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS));
}

/****************************************************************************/
/*                                                                          */
/*            I2C 写一个字节到指定地址                                 */
/*                                                                          */
/****************************************************************************/
void I2CByteWrite(unsigned int Address, unsigned char Data)
{
    slaveData[0] = Address;
    slaveData[1] = Data;

    IICSend(SOC_I2C_0_REGS,2);
}
/****************************************************************************/
/*                                                                          */
/*            I2C 从指定地址读一个字节                                 */
/*                                                                          */
/****************************************************************************/
unsigned char  I2CByteRead(unsigned int Address)
{
	slaveData[0] = Address;

	IICSend(SOC_I2C_0_REGS,1);
	IICReceive(SOC_I2C_0_REGS, 1);

   	return slaveData[0];
}
unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
       	int i;

	slaveData[0] = sub_addr;

	for (i=0;i<ByteNo;i++)
	slaveData[i+1] = buff[i];

	IICSend(SOC_I2C_0_REGS, ByteNo + 1); 

	return 0;
}
unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
   	int i;
	for (i=0;i<ByteNo;i++)
		slaveData[i]=0;

	slaveData[0] = sub_addr;
	
	
	IICSend(SOC_I2C_0_REGS,1);
	IICReceive(SOC_I2C_0_REGS, ByteNo);

	for (i=0;i<ByteNo;i++)
	buff[i] = slaveData[i] ;

        return 0;
}


