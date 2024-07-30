#define _MAIN_C
#include <Intrins.h>
#include <Absacc.h>
#include <Stdio.h>
#include ".\include\REG_MG82F6D17.H"
#include ".\include\Type.h"
#include ".\include\API_Macro_MG82F6D17.H"
#include ".\include\API_Uart_BRGRL_MG82F6D17.H"
#include <string.h>


#define MCU_SYSCLK		12000000
#define MCU_CPUCLK		(MCU_SYSCLK)

#define TIMER_1T_1ms_TH	((65536-(u16)(float)(1000*((float)(MCU_SYSCLK)/(float)(1000000)))) /256) 			
#define TIMER_1T_1ms_TL	((65536-(u16)(float)(1000*((float)(MCU_SYSCLK)/(float)(1000000)))) %256)
#define SFR_Page_(x)		SFRPI = x;
#define DISPLAY_ON_MS    5000
#define DISPLAY_OFF_MS   4000

#define ADC              P17
#define IO1              P15
#define IO2              P16
#define IO3              P24
#define IO4              P22
#define SDA              P30
#define SCL              P31
#define HUMAN            P33
#define MOSFET_ONOFF2    P61
#define MOSFET_ONOFF1    P34


#define TX               P11
#define RX               P10

#define ON               1  
#define OFF              0 


uint8_t CheckSend = ON;
// khoi tao du lieu, bo nho UART1*****
#define UART1_RX_BUFF_SIZE   32
#define UART1_TX_BUFF_SIZE   32
xdata u8 RcvBuf[UART1_RX_BUFF_SIZE];
char dataUart = '1';
u8 Uart1RxIn = 0;
u8 Uart1RxOut = 0;
xdata u8 TxBuf[UART1_TX_BUFF_SIZE];
u8 Uart1TxIn = 0;
u8 Uart1TxOut = 0;
bit bUart1TxFlag;
// Du lieu giao tieo UART1
// Du lieu doc du lieu ADC
#define TEST_ADC_DATA_CNT		64
xdata WordTypeDef TestBuf[TEST_ADC_DATA_CNT];
u8 TestBufLen;
// Du lieu truyen nhan I2C_Master
#define SLAVE_ADDRESS_A0 0x06
#define SLAVE_ADDRESS_B0 0x08
#define TEST_BUF_SIZE	16
u8 TWI0OvTime;
u8 rand;
WordTypeDef TWI0TestAddr;
bit bES0;
u8 LedTime;
xdata u8 WriteBuf[TEST_BUF_SIZE];
xdata u8 ReadBuf[TEST_BUF_SIZE];
u8 TestBufLen;
// Du lieu truyen nhan I2C_Slave
#define	TWI_OV_TIME_MAX		20
#define I2C_SCL		SCL
#define I2C_SDA		SDA
#define SLAVE_ADDRESS	 0x0C // ~ 0x06
u8	TWI0OvTime;
bit bTWI0Error;
idata BYTE Tx[2];
idata BYTE Rx[7];
u8 RxInx;
u8 TxInx;
// Timer T0 init
volatile uint16_t time_cnt_1s = 0;
int checkHuman = 0;
int checkShutdown = 1;
//int checkReset =0;
////////////////////////////////////////
uint8_t val[5];
//////////////////////////////////////// Khai bao bien phuc vu do dien ap vao cua thiet bi
float medium_voltage; // dien ap trung binh moi 5 giay
float voltage_current = 0; // dien ap tuc thoi moi lan quet
uint8_t cnt_voltage = 0; // bien dem moi lan quet, 5 lan tu xoa ve 0
uint8_t POWER_STATUS; // Bien khai bao UPS dang dung nguon tu Adapter(0x00 la su dung dien tu PIN)
uint8_t HUMAN_SENSOR = 0x00; // Bien bao trang thai khong co nguoi lai gan thiet bi(0xFF la co nguoi lai gan)
#define BATTERY_LOW   18 // Khai bao dien ap thap chuan bi shutdown
#define ADAPTER_VOL   22 // Khai bao dien ap ADAPTER tro len
#define BATTERY_SHUTDOWN 17.5 // Khai bao dien ap  shutdown

void INT_INT1(void)	interrupt INT_VECTOR_INT1 // Chuong trinh ngat INT1
{}
void InitINT1(void) // Config Interrupt1
{
	INT_SetINT1P33();						// nINT1 : P33
	INT_SetINT1_DetectEdge();				// nINT1 Detect type: edge . On Power-Down mode,only Level.
	INT_SetINT1_DetectLowFalling();			// nINT1 Detect: low level/falling edge
	INT_SetINT1Filter_SysclkDiv6_x3();		// nINT1 filter:£¨sysclk/6£©*3
}
void INT_UART1(void) interrupt INT_VECTOR_UART1 // Chuong trinh ngat truyen nhan du lieu UART
{
	_push_(SFRPI);
	SFR_Page_(1);		  
	if(TI1)					
	{ 
		
	  TI1 = 0;	   
		if(Uart1TxIn==Uart1TxOut)
		{
			bUart1TxFlag=FALSE;
		}
		else
		{
			S1BUF=TxBuf[Uart1TxOut];
			bUart1TxFlag=TRUE;
			Uart1TxOut++;
			if(Uart1TxOut>=UART1_TX_BUFF_SIZE)
			{
				Uart1TxOut=0;
			}
		}
	}
	if(RI1)
	{ 
		/*checkReset=1;*/
		RI1 = 0;
		RcvBuf[Uart1RxIn] = S1BUF;
		Uart1RxIn++;
		if(Uart1RxIn >= UART1_RX_BUFF_SIZE)
		{
			Uart1RxIn =0;
		}
	}
	_pop_(SFRPI);		  
}
void Uart1SendByte(u8 tByte) // Chuong trinh gui byte du lieu UART
{
	u8 i;
	if(bUart1TxFlag==FALSE)
	{
		Uart1TxOut=0;
		Uart1TxIn=1;
		TxBuf[0]=tByte;
		SFR_Page_(1);
    TI1=1;
    SFR_Page_(0);
	}else
	{
		i=Uart1TxIn;
		TxBuf[i]=tByte;
		i++;
		if(i>=UART1_TX_BUFF_SIZE)
		{
			i=0;
		}
		while(i==Uart1TxOut){}
		INT_DisUART1();
		Uart1TxIn=i;
		INT_EnUART1();
	}
}
void Uart1SendStr(u8* PStr) // Chuong trinh gui chuoi du lieu UART
{
	while(*PStr != 0)
	{
		Uart1SendByte(*PStr);
		PStr ++;
	}
}

void DelayXus(u8 xUs)
{
	while(xUs!=0)
	{
#if (MCU_CPUCLK>=11059200)
		_nop_();
#endif
#if (MCU_CPUCLK>=14745600)
		_nop_();
		_nop_();
		_nop_();
		_nop_();
#endif
#if (MCU_CPUCLK>=16000000)
		_nop_();
#endif

#if (MCU_CPUCLK>=22118400)
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
#endif
#if (MCU_CPUCLK>=24000000)
		_nop_();
		_nop_();
#endif		
#if (MCU_CPUCLK>=29491200)
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
#endif
#if (MCU_CPUCLK>=32000000)
		_nop_();
		_nop_();
#endif
		xUs--;
	}
}
void DelayXms(u16 xMs)
{
	while(xMs!=0)
	{
		CLRWDT();
		DelayXus(200);
		DelayXus(200);
		DelayXus(200);
		DelayXus(200);
		DelayXus(200);
		xMs--;
	}
}

void InitTimer0(void) // Chuong trinh khoi tao Timer0
{
	TM_SetT0Mode_2_8BIT_AUTORELOAD();		// TIMER0 Mode:8bit timer with auto reload
	TM_SetT0Clock_SYSCLKDiv192();			// TIMER0 Clock source: SYSCLK/192
	TM_SetT0Gate_Disable();					// TIMER0 disable gate
	
	TM_SetT0LowByte(0);						// Set TL0 value
	TM_SetT0HighByte(193);					// Set TH0 value for reload 62 xung se ngat 1ms (255-193 = 62)
	
	TM_EnableT0();							// Enable TIMER0
}
void InitUart1(void) // Chuong trinh khoi tao Uart1
{
	UART1_SetMode8bitUARTVar();							// UART1 Mode: 8-bit, Variable B.R
	UART1_EnS1BRG();									   	  // Enable S1BRG
	UART1_SetBaudRateX2();									// S1BRG x2
	UART1_SetRxTxP10P11();									// UART1 Pin£ºRX:P10 TX:P11
	UART1_EnReception();								   	// Enable reception
	UART1_SetS1BRGSelSYSCLK();							// S1BRG clock source£ºSYSCLK
	
	UART1_SetS1BRGValue(S1BRG_BRGRL_9600_2X_12000000_1T);	// 12M
	//UART1_SetS1BRGValue(S1BRG_BRGRL_9600_2X_24000000_1T);	// 24M
	//UART1_SetS1BRGValue(S1BRG_BRGRL_9600_2X_32000000_1T);	// 32M
	//UART1_SetS1BRGValue(S1BRG_BRGRL_9600_2X_36000000_1T);	// 36M
	//////////////////////////////////////////////////////////////////////
	Uart1RxIn=0;
	Uart1RxOut=0;
	Uart1TxIn=0;
	Uart1TxOut=0;
	bUart1TxFlag=0;
}
void INT_T0(void) interrupt INT_VECTOR_T0 // Chuong trinh ngat timer0
{
	time_cnt_1s ++;
}
void InitPort(void) // Chuong trinh cau hinh Port
{	
	PORT_SetP1AInputOnly(BIT7); // ADC 
	PORT_SetP2PushPull(BIT2|BIT4); // IO4, IO3
	PORT_SetP3QuasiBi(BIT0|BIT1);  //SDA, SCL
	PORT_SetP3PushPull(BIT4); // ON_OFF1
	PORT_SetP3InputOnly(BIT3); // HUMAN
	PORT_SetP6PushPull(BIT1); // ON_OFF2
	PORT_SetP1OpenDrain(BIT6|BIT5); // IO2 , IO1
	PORT_SetP1PushPull(BIT1); // TX
	PORT_SetP1OpenDrain(BIT0); // RX
}
void InitADC(void) // Chuong trinh cau hinh ADC
{
	ADC_Enable();									// Enable ADC
	ADC_SetClock_SYSCLK();							// ADC Clock = SYSCLK       sps= 12M/30=600K
	//ADC_SetMode_FreeRunning();						// ADC Trigger mode: Freeruning
	ADC_SetMode_SetADCS();				// ADC Trigger mode: set ADCS
	ADC_SetRightJustified();						// ADC Right-Justified
	ADC_SetADCData_8Bit();							// ADC Data resolution: 8bit
	ADC_SetChannel_AIN7();							// ADC set channel 0
}
void InitClock(void) // Chuong trinh cau hinh xung Clock
{
#if (MCU_SYSCLK==11059200)
#if (MCU_CPUCLK==MCU_SYSCLK)
	// SysClk=11.0592MHz CpuClk=11.0592MHz
	CLK_SetCKCON0(IHRCO_110592MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1);
	
#else
	// SysClk=11.0592MHz CpuClk=5.5296MHz
	CLK_SetCKCON0(IHRCO_110592MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1);
#endif
#endif

#if (MCU_SYSCLK==12000000)
#if (MCU_CPUCLK==MCU_SYSCLK)
	// SysClk=12MHz CpuClk=12MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1);
	
#else
	// SysClk=12MHz CpuClk=6MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1);
#endif
#endif

#if (MCU_SYSCLK==22118400)
#if (MCU_CPUCLK==MCU_SYSCLK)
	// SysClk=22.1184MHz CpuClk=22.1184MHz
	CLK_SetCKCON0(IHRCO_110592MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx4, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X4|OSCIn_IHRCO);
#else
	// SysClk=22.1184MHz CpuClk=11.0592MHz
	CLK_SetCKCON0(IHRCO_110592MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx4, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X4|OSCIn_IHRCO);
#endif
#endif

#if (MCU_SYSCLK==24000000)
#if (MCU_CPUCLK==MCU_SYSCLK)
	// SysClk=24MHz CpuClk=24MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx4, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X4|OSCIn_IHRCO);
#else
	// SysClk=24MHz CpuClk=12MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx4, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X4|OSCIn_IHRCO);
#endif
#endif

#if (MCU_SYSCLK==29491200)
#if (MCU_CPUCLK==MCU_SYSCLK)
	// Cpuclk high speed
	CLK_SetCpuCLK_HighSpeed();
	// SysClk=29.491200MHz CpuClk=29.491200MHz
	CLK_SetCKCON0(IHRCO_110592MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx5.33, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X533|OSCIn_IHRCO);
#else
	// SysClk=29.491200MHz CpuClk=14.7456MHz
	CLK_SetCKCON0(IHRCO_110592MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx5.33, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X533|OSCIn_IHRCO);
#endif
#endif

#if (MCU_SYSCLK==32000000)
#if (MCU_CPUCLK==MCU_SYSCLK)
	// Cpuclk high speed
	CLK_SetCpuCLK_HighSpeed();
	// SysClk=32MHz CpuClk=32MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx5.33, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X533|OSCIn_IHRCO);
#else
	// SysClk=32MHz CpuClk=16MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx5.33, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X533|OSCIn_IHRCO);
#endif
#endif

#if (MCU_SYSCLK==36000000)
#if (MCU_CPUCLK==MCU_SYSCLK)
	// Cpuclk high speed
	CLK_SetCpuCLK_HighSpeed();
	// CKMIx6,x8,x12
	CLK_SetCKM_x6x8x12();	
	// SysClk=36MHz CpuClk=18MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx6, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X4_X6|OSCIn_IHRCO);
#else
	// CKMIx6,x8,x12
	CLK_SetCKM_x6x8x12();	
	// SysClk=36MHz CpuClk=18MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx6, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X4_X6|OSCIn_IHRCO);
#endif
#endif


#if (MCU_SYSCLK==44236800)
	// SysClk=44.2368MHz CpuClk=22.1184MHz
	CLK_SetCKCON0(IHRCO_110592MHz|CPUCLK_SYSCLK_DIV_1|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx8, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X8|OSCIn_IHRCO);
#endif

#if (MCU_SYSCLK==48000000)
	// SysClk=48MHz CpuClk=24MHz
	CLK_SetCKCON0(IHRCO_12MHz|CPUCLK_SYSCLK_DIV_2|SYSCLK_MCKDO_DIV_1|ENABLE_CKM|CKM_OSCIN_DIV_2);
	DelayXus(100);
	// IHRCO, MCK=CKMIx8, OSCin=IHRCO
	CLK_SetCKCON2(ENABLE_IHRCO|MCK_CKMI_X8|OSCIn_IHRCO);
#endif
}

void Enable_P44P45() // Chuong trinh bat tinh nang chan P44,P45
{
	u8 x;
	bit bEA=EA;
	ISPCR=0x80;
	IFADRH=0x00;
	IFADRL=DCON0_P;
	IFMT=ISP_READ_P;
	SCMD=0x46;
	SCMD=0xB9;
	_nop_();
	x=IFD;
	x=x&(~OCDE_P);
	IFD=x;
	IFMT=ISP_WRITE_P;
	SCMD=0x46;
	SCMD=0xB9;
	_nop_();
	ISPCR=0x00;
	IFMT=ISP_STANBY;
}
uint16_t GetAdcValue() // Chuong trinh doc gia tri ADC
{
	WordTypeDef wAdcValue;
	ADCON0 = ADCON0|ADCS;							// set ADCS,Start ADC
  while((ADCON0&ADCI)==0);           				// wait ADC complete
  wAdcValue.B.BHigh=ADCDH;
  wAdcValue.B.BLow=ADCDL;
 	ADCON0 = ADCON0&(~ADCI);		           		// clear ADCI flag
 	return wAdcValue.W&0x0FFF;
}
/*bit Check_adc(uint16_t val) // neu quet 100 lan lien tiep ma gia tri ADC doc ve nho hon val thi tra ve 1
{
	uint8_t j;
	uint8_t k = 0;
	for(j = 0; j < 100; j ++)
	{
		if(GetAdcValue() < val){k ++;}
		DelayXus(100);
	}
	if(k == 100){return 1;}else{return 0;}
}*/
// Ham cau hinh cho I2C Master
void InitTWI0()
{
	TWI0_Clear();
	TWI0_SetClock(TWI0_CLK_SYSCLK_256);
	TWI0_SetUseP30P31();
	TWI0_Enable();
}
//u8 TWI0_WriteBuf(u8 DevAddr,u16 RegStartAddr,u8 *pBuf,u8 Len)
u8 TWI0_WriteBuf(u8 DevAddr,u8 *pBuf,u8 Len)
{
	u8 i;
	u8 Flag;
	Flag=1;
	SICON |=STA;	// Send START
	TWI0OvTime=10;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_WRITE_ERR;}	 // wait completed, if time overflow,then return fail.
	SICON &=~STA;
	SICON=SICON|(AA);
	Uart1SendStr("B\r\n");
	
	Flag++;
	SIDAT = DevAddr&0xFE;		// send Slave Device address
	TWI0OvTime=10;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime == 0) goto TWI0_WRITE_ERR;}		// wait completed, if time overflow,then return fail.
	Uart1SendStr("C\r\n");
  /*
	Flag++;
	SIDAT = HIBYTE(RegStartAddr);		// send Slave Data address high
	TWI0OvTime=5;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_WRITE_ERR;}		// wait completed, if time overflow,then return fail.
	Flag++;
	SIDAT = LOBYTE(RegStartAddr);		// send Slave Data address low
	TWI0OvTime=5;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_WRITE_ERR;}		// wait completed, if time overflow,then return fail.
  */
	Flag++;
	i=0;
	while(i<Len)
	{
		if(i==(Len-1))
		{
			SICON=SICON&(~AA);    
		}else
		{
			SICON=SICON|(AA);	   
		}
		SIDAT=pBuf[i];			// send DATA
		TWI0OvTime=10;       
		SICON &=~SI;			  
		while((SICON&SI)!=SI){if(TWI0OvTime == 0) goto TWI0_WRITE_ERR;}		// wait completed, if time overflow,then return fail.
		i++;
	}
	Uart1SendStr("D\r\n");
	
	Flag++;
	SICON |= STO;				  // Send STOP
	TWI0OvTime=10;   
	SICON &=~SI;	 
	while((SICON&STO)==STO){if(TWI0OvTime == 0) goto TWI0_WRITE_ERR;}		// wait completed, if time overflow,then return fail.
	SICON &=~STO;
	SICON = SICON &(~SI);		
	return 0;
  TWI0_WRITE_ERR:
	return Flag;
}
u8 TWI0_ReadBuf(u8 DevAddr,u8 *pBuf,u8 Len)
{
	u8 i;
	u8 Flag;
	Flag=1;
	
	SICON |=STA;				// Send START
	TWI0OvTime=10;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail
	SICON &=~STA;

	Flag++;
	SICON=SICON|(AA);	 
	SIDAT = DevAddr&0xFE;		// send Slave Device address  
	TWI0OvTime=10;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail
  /*
	Flag++;
	SIDAT = HIBYTE(RegStartAddr);		// send Slave data address high
	TWI0OvTime=5;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail

	Flag++;
	SIDAT = LOBYTE(RegStartAddr);		// send Slave data address low
	TWI0OvTime=5;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail
  */
	Flag++;
	SICON |= STA;				// resend I2C START
	TWI0OvTime=10;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail
	SICON &=~STA;

	Flag++;
	SIDAT = DevAddr|0x01;		// send Slave Device address,enter read mode
	TWI0OvTime=10;
	SICON &=~SI;			
	while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail
  
	Flag++;
	i=0;
	while(i<Len)
    {
		if(i==(Len-1))
    	{
			SICON=SICON&(~AA);	   
		}
		else
		{
			SICON=SICON|(AA);	   
		}
		TWI0OvTime=5;
		SICON &=~SI;			
		while((SICON&SI)!=SI){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail
		pBuf[i] = SIDAT; 			// read Data
		i++;
	}
			
	Flag++;
	SICON |= STO;				// send STOP
	TWI0OvTime=5;
	SICON &=~SI;			
	while((SICON&STO)==STO){if(TWI0OvTime==0) goto TWI0_READ_ERR;}		// wait completed, if time overflow,then return fail
	SICON &=~STO;

	SICON = SICON &(~SI);		

	return 0;
	
  TWI0_READ_ERR:
	SICON |= STO;				// send STOP
	SICON = SICON &(~SI);		
	return Flag;
}
// Ham cau hinh cho I2C Slave
void INT_TWSI() interrupt INT_VECTOR_TWI0
{
	if(SISTA==0x80)
	{ // 0x80: DATA has been received. ACK has been returned.
		Rx[RxInx]=SIDAT;				// receive data
		RxInx++;
		if(RxInx >= 7) RxInx=0;
	}
	else if((SISTA==0x60)||(SISTA==0x68))
	{ // 0x60,0x68:Own SLA+W has been received. ACK has been returned
		RxInx=0;						
	}
	else if((SISTA==0xA8)||(SISTA==0xB0))
	{ // 0xA8,0xB0:Own SLA+R has been received. ACK has been returned
		TxInx=0;						
		goto _IIC_SET_SIDAT;
	}
	else if((SISTA==0xB8)||(SISTA==0xC0)||(SISTA==0xC8))
	{
		// 0xB8:  Data byte in SIDAT has been transmitted ACK has been received
		// 0xC0:  Data byte or Last data byte in SIDAT has been transmitted Not ACK has been received
		// 0xC8:  Last Data byte in SIDAT has been transmitted ACK has been received
    _IIC_SET_SIDAT:
		SIDAT=Tx[TxInx]; 		 // send data
		TxInx++;
		if(TxInx >= 7) TxInx=0;
	}
	I2C_SCL=0;
	SICON = ENSI|AA;					// clear TWI0 interrupt flag
	TWI0OvTime = TWI_OV_TIME_MAX;		// restore TWI0 overtime
	I2C_SCL=1;
}
void InitTWI0_Slave()
{
	TWI0_Clear();
	TWI0_SetClock(TWI0_CLK_SYSCLK_64);
	TWI0_SetUseP30P31(); // P30-SCL, P31-SDA
	TWI0_Enable();
	TWI0_SetSlaveAddr(SLAVE_ADDRESS);
	TWI0_SendACK();
}
void InitInterrupt(void) // Chuong trinh khoi tao cac ngat
{
	INT_EnTIMER0();
	INT_EnUART1();
	//INT_EnINT1();
	INT_EnDMA();						// Enable DMA interrrupt
	//INT_EnTWI0();
}
void InitSystem(void) // Chuong trinh khoi tao he thong
{
	InitPort();
	//Enable_P44P45();
	InitClock();
	InitTimer0();
	InitUart1();
	InitADC();
	//InitINT1();
	//InitTWI0(); // I2C Master
	//InitTWI0_Slave(); // I2C Slave
	InitInterrupt();
	INT_EnAll();
}
uint8_t Security_Check_Data_I2C()
{
	if(Rx[0]+Rx[1]==0xFF && Rx[2]+Rx[3]==0xFF && Rx[4]+Rx[5]==0xFF)
	{return 0xFF;}else{return 0x00;}
}
float Get_voltage()
{
	return ((float)GetAdcValue() / 255 ) * 44.44;
}
void setMOSFETState(int state) {
    MOSFET_ONOFF1 = state;
    MOSFET_ONOFF2 = state;
}
void DetectHuman(){
	Uart1SendStr("  \"HUMAN\":");
	if(0 == IO1){Uart1SendStr("\"YES\"");}
  else{Uart1SendStr("\"NO\"");}
}
void SEND_IO_STATUS(int io) {
    Uart1SendStr("  \"io_status\":"); 
    Uart1SendStr((io == 1) ? "\"ON\"" : "\"OFF\"");
    Uart1SendStr(",\n");
}
void sendStatus()
{
	  Uart1SendStr(",\n");
    //SEND_IO_STATUS(IO1);
	  SEND_IO_STATUS(IO2);
	  //SEND_IO_STATUS(IO3);
    //SEND_IO_STATUS(IO4); 
}
void BlinkLed(int Value){
	  IO4 = 0; DelayXms(Value);IO4 =1;
}
void LOG()
{
	Uart1SendStr("{\n");
	Uart1SendStr("  \"VOLTAGE\":");
	sprintf(val,"%.4g",medium_voltage);
	Uart1SendStr(val);
	Uart1SendStr(",\n");
	Uart1SendStr("  \"POWER\":");
	switch(POWER_STATUS)
	{
		case 0xFF: Uart1SendStr("\"ADAPTER\"");setMOSFETState(ON); break;
		case 0xAA: Uart1SendStr("\"BATTERY\"");setMOSFETState(ON); break;
		case 0x11: Uart1SendStr("\"LOW\"");    setMOSFETState(ON); break;
		case 0x00: Uart1SendStr("\"SHUTDOWN\"");
		           /*SHUT DOWN */
		           setMOSFETState(OFF);
			         //Provide pulse to IO2 pin
		           IO2 =1;DelayXms(200);IO2 =0;
		          break;
		default: break;
	}
	Uart1SendStr(",\n");
	DetectHuman();
  sendStatus();
	Uart1SendStr("}\n");
}
void handleVol(int *checkShutdown, uint8_t *powerStatus, int displayMs, int shutdownValue) {
    if (*checkShutdown == 0) {
        BlinkLed(displayMs);
        *checkShutdown = 1;
    }
    *powerStatus = shutdownValue;
}

void TRANVANLUU(){
	if (!IO1) {IO3 = ON;}
		else {IO3 = OFF; }
		if(time_cnt_1s > 1000)
		{
			time_cnt_1s = 0;
			cnt_voltage ++;
			voltage_current = voltage_current + Get_voltage() / 5;
			if(cnt_voltage == 5)
			{
				medium_voltage = voltage_current;
				voltage_current = 0;
				cnt_voltage = 0;
				if (medium_voltage > 10) CheckSend = ON;
				if (CheckSend){
					
				if (medium_voltage >= ADAPTER_VOL) {
          handleVol(&checkShutdown, &POWER_STATUS, DISPLAY_ON_MS, 0xFF);} // ADAPTER
        else if (medium_voltage >= BATTERY_LOW) {
          handleVol(&checkShutdown, &POWER_STATUS, DISPLAY_ON_MS, 0xAA);}  // BATTERY
        else if (medium_voltage >= BATTERY_SHUTDOWN) {
          handleVol(&checkShutdown, &POWER_STATUS, DISPLAY_ON_MS, 0x11);} // PREPARE TO SHUTDOWN
        else {
          if (checkShutdown == 1) {
          BlinkLed(DISPLAY_OFF_MS);
          CheckSend = OFF;
          checkShutdown = 0;
          }
          POWER_STATUS = 0x00; // SHUTDOWN
       }
				/*if(medium_voltage >= ADAPTER_VOL){if(checkShutdown == 0) BlinkLed(DISPLAY_ON_MS);checkShutdown =1;POWER_STATUS = 0xFF;} // ADAPTER
				else if(medium_voltage < ADAPTER_VOL && medium_voltage >= BATTERY_LOW){if(checkShutdown == 0) BlinkLed(DISPLAY_ON_MS);checkShutdown =1;POWER_STATUS = 0xAA;} // BATTERY
				else if(medium_voltage < BATTERY_LOW && medium_voltage >= BATTERY_SHUTDOWN ){if(checkShutdown == 0) BlinkLed(DISPLAY_ON_MS);checkShutdown =1;POWER_STATUS = 0x11;} // PREPARE TO SHUTDOWN
				else if(medium_voltage < BATTERY_SHUTDOWN) { //BlinkLedinShutdown();
		           if (checkShutdown == 1) BlinkLed(DISPLAY_OFF_MS);CheckSend = 1;
		           checkShutdown = 0;POWER_STATUS = 0x00;}*/  // SHUTDOWN 
				LOG();
			}
			}
		}
}

	
void main()
{
  InitSystem();
	IO4 = 1;
	while(1)
	{ 
		/*Receive RESET request from user */
	  if (!Uart1RxIn) {
			setMOSFETState(OFF);
			DelayXms(1000);
			setMOSFETState(ON);
			Uart1RxIn = 0;
	 }
		/*If detecting human , send information immediately*/
		if (!IO1 && 0 == checkHuman){
			cnt_voltage = voltage_current = 0; 
			LOG();
			checkHuman = 1;
		}
		TRANVANLUU();
		if (IO1) checkHuman =0;
	}
}