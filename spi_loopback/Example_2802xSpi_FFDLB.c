//#############################################################################
//
//  File:   f2802x_examples/adc_soc/Example_F2802xAdcSoc.c
//
//  Title:  F2802x ADC Start-Of-Conversion (SOC) Example Program.
//
//  Group:          C2000
//  Target Device:  TMS320F2802x
//
//! \addtogroup example_list
//!  <h1>ADC Start-Of-Conversion (SOC)</h1>
//!
//!   Interrupts are enabled and the ePWM1 is setup to generate a periodic
//!   ADC SOC - ADCINT1. Two channels are converted, ADCINA4 and ADCINA2.
//!
//!   Watch Variables:
//!
//!   - Voltage1[10] - Last 10 ADCRESULT0 values
//!   - Voltage2[10] - Last 10 ADCRESULT1 values
//!   - ConversionCount - Current result number 0-9
//!   - LoopCount - Idle loop counter
//
//#############################################################################
// $TI Release: F2802x Support Library v222 $
// $Release Date: Thu Jan 15 13:56:57 CST 2015 $
// $Copyright: Copyright (C) 2008-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/pwm.h"
#include "f2802x_common/include/wdog.h"
#include "f2802x_common/include/spi.h"
#define uchar unsigned char
// Prototype statements for functions found within this file.
__interrupt void adc_isr(void);
void Adc_Config(void);
void LED4_Display (void);
void LED_OUT(uchar X);
void spi_fifo_init(void);
void spi_init(void);
Uint16 slip(void);
// Global variables used in this example:
uint16_t LoopCount;
uint16_t ConversionCount;
uint16_t Voltage1[10];
uint16_t Voltage2[10];
uint16_t Volt1[9];
Uint16 volt;

Uint16 Volt=0;
int slipcount=0;
Uint16  adcdata1;
Uint16  adcdata2;
/////////////LED
unsigned char LED_0F[17] ={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0x8C,0xBF,0xC6,0xA1,0x86,0xFF,0xbf};
unsigned char LED[8];	//
unsigned char LED_1F[17] ={0x40,0x79,0x24,0x30,0x19,0x12,0x02,0x78,0x00,0x10,0x8C,0xBF,0xC6,0xA1,0x86,0xFF,0xbf};
Uint16 LED_display_data= 0;
//////////////////////
ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
PWM_Handle myPwm;
SPI_Handle mySpi;

void main(void)
{
	uint16_t sdata;  // send data
	uint16_t rdata;  // received data
    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;

    // Initialize all the handles needed for this application
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myPwm = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    mySpi = SPI_init((void *)SPIA_BASE_ADDR, sizeof(SPI_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

    // Perform basic system initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();

    //Select the internal oscillator 1 as the clock source
    //CLK_setOscSrc(myClk, CLK_OscSrc_Internal);
    CLK_enableCrystalOsc(myClk);
    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);

    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

// If running from flash copy RAM only functions to RAM
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif


/////SPI///////////////////////////
    // Initialize GPIO
    GPIO_setPullUp(myGpio, GPIO_Number_16, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_17, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_18, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_19, GPIO_PullUp_Enable);
    GPIO_setQualification(myGpio, GPIO_Number_16, GPIO_Qual_ASync);
    GPIO_setQualification(myGpio, GPIO_Number_17, GPIO_Qual_ASync);
    GPIO_setQualification(myGpio, GPIO_Number_18, GPIO_Qual_ASync);
    GPIO_setQualification(myGpio, GPIO_Number_19, GPIO_Qual_ASync);
    GPIO_setMode(myGpio, GPIO_Number_16, GPIO_16_Mode_SPISIMOA);
    GPIO_setMode(myGpio, GPIO_Number_17, GPIO_17_Mode_SPISOMIA);
    GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_SPICLKA);
    GPIO_setMode(myGpio, GPIO_Number_19, GPIO_19_Mode_SPISTEA_NOT);
    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);
////////////SPI
    spi_init();         // Initialize SPI
       spi_fifo_init();    // Initialize the SPI FIFOs

       sdata = 0x0000;
       sdata = 0xC3AA;
    // Register interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_10, PIE_SubGroupNumber_1, (intVec_t)&adc_isr);

    // Initialize the ADC
    ADC_enableBandGap(myAdc);
    ADC_enableRefBuffers(myAdc);
    ADC_powerUp(myAdc);
    ADC_enable(myAdc);
    ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

    // Enable ADCINT1 in PIE
    PIE_enableAdcInt(myPie, ADC_IntNumber_1);
    // Enable CPU Interrupt 1
    CPU_enableInt(myCpu, CPU_IntNumber_10);
    // Enable Global interrupt INTM
    CPU_enableGlobalInts(myCpu);
    // Enable Global realtime interrupt DBGM
    CPU_enableDebugInt(myCpu);

    LoopCount = 0;
    ConversionCount = 0;


adcdata1=0;
adcdata2=0;
    // Configure ADC
    //Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata
    ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);               //ADCINT1 trips after AdcResults latch
    ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enabled ADCINT1
    ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);          //Disable ADCINT1 Continuous mode
    ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC2);                 //setup EOC2 to trigger ADCINT1 to fire
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A4);    //set SOC0 channel select to ADCINA4
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A4);    //set SOC1 channel select to ADCINA4
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_A2);    //set SOC2 channel select to ADCINA2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

    // Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_1);

    // Setup PWM
    PWM_enableSocAPulse(myPwm);                                         // Enable SOC on A group
    PWM_setSocAPulseSrc(myPwm, PWM_SocPulseSrc_CounterEqualCmpAIncr);   // Select SOC from from CPMA on upcount
    PWM_setSocAPeriod(myPwm, PWM_SocPeriod_FirstEvent);                 // Generate pulse on 1st event
    PWM_setCmpA(myPwm, 0x0080);                                         // Set compare A value
    PWM_setPeriod(myPwm, 0x07D0);                                       // Set period for ePWM1
    PWM_setCounterMode(myPwm, PWM_CounterMode_Up);                      // count up and start
    CLK_enableTbClockSync(myClk);
    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
        GPIO_setMode(myGpio, GPIO_Number_1, GPIO_0_Mode_GeneralPurpose);
        GPIO_setMode(myGpio, GPIO_Number_7, GPIO_0_Mode_GeneralPurpose);
        GPIO_setMode(myGpio, GPIO_Number_12, GPIO_0_Mode_GeneralPurpose);

        GPIO_setMode(myGpio, GPIO_Number_3, GPIO_0_Mode_GeneralPurpose);//DIO for led data
        GPIO_setMode(myGpio, GPIO_Number_4, GPIO_0_Mode_GeneralPurpose);//RSCK Clock pulse for data, rising edage is active;
        GPIO_setMode(myGpio, GPIO_Number_5, GPIO_0_Mode_GeneralPurpose);//SCLK Led data display enable clock ,  rising edage is active;

        GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
        GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
        GPIO_setDirection(myGpio, GPIO_Number_7, GPIO_Direction_Output);
        GPIO_setDirection(myGpio, GPIO_Number_12, GPIO_Direction_Output);

        GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);
        GPIO_setDirection(myGpio, GPIO_Number_4, GPIO_Direction_Output);
        GPIO_setDirection(myGpio, GPIO_Number_5, GPIO_Direction_Output);


        GPIO_setLow(myGpio, GPIO_Number_0);
        GPIO_setLow(myGpio, GPIO_Number_1);
        GPIO_setLow(myGpio, GPIO_Number_7);
        GPIO_setLow(myGpio, GPIO_Number_12);
       // GPIO_setHigh(myGpio, GPIO_Number_1);
       // GPIO_setLow(myGpio, GPIO_Number_7);
       // GPIO_setHigh(myGpio, GPIO_Number_12);

        GPIO_setLow(myGpio, GPIO_Number_3);
        GPIO_setLow(myGpio, GPIO_Number_4);
        GPIO_setLow(myGpio, GPIO_Number_5);

    // Wait for ADC interrupt
    for(;;)
    {

        LoopCount++;
        LED4_Display ();
        // Transmit data
                SPI_write(mySpi, sdata);

                // Wait until data is received
                while(SPI_getRxFifoStatus(mySpi) == SPI_FifoStatus_Empty)
                {
                }

                // Check against sent data
                rdata = SPI_read(mySpi);
               // if(rdata != sdata)
                 //   error();

               // sdata++;
                sdata = 0x33AA;
                rdata= 0xC3AA;
    }

}


__interrupt void adc_isr(void)
{
	    GPIO_setHigh(myGpio, GPIO_Number_0);

	    GPIO_setLow(myGpio, GPIO_Number_0);
	    GPIO_toggle(myGpio, GPIO_Number_1);
	    GPIO_toggle(myGpio, GPIO_Number_7);
	    GPIO_toggle(myGpio, GPIO_Number_12);

    //discard ADCRESULT0 as part of the workaround to the 1st sample errata for rev0
    Voltage1[ConversionCount] = ADC_readResult(myAdc, ADC_ResultNumber_1);
    Voltage2[ConversionCount] = ADC_readResult(myAdc, ADC_ResultNumber_2);
    adcdata1=ADC_readResult(myAdc, ADC_ResultNumber_0);
    adcdata2=ADC_readResult(myAdc, ADC_ResultNumber_2);
    Volt=slip();

    // If 10 conversions have been logged, start over
    if(ConversionCount == 9)
    {
        ConversionCount = 0;

    }
    else ConversionCount++;
    LED_display_data=Volt;
    LED[3]=LED_display_data/1000; //1262/1000=1
    LED[2]=LED_display_data/100%10;//1262/100=12  12%10=2
    LED[1]=LED_display_data/10%10;//1262/10=126   126%10=6
    LED[0]=LED_display_data%10;//

    // Clear ADCINT1 flag reinitialize for next SOC
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_10);
    GPIO_setLow(myGpio, GPIO_Number_0);
    return;
}
void LED_OUT(uchar X)
{
	uchar i;
	for(i=8;i>=1;i--)
	{
		if (X&0x80)// always shift output the maximum bit
			GPIO_setHigh(myGpio, GPIO_Number_3);//DIO
		else
			GPIO_setLow(myGpio, GPIO_Number_3);//DIO

		X<<=1;
		GPIO_setLow(myGpio, GPIO_Number_5);//SCLK
		__asm(" NOP");
		GPIO_setHigh(myGpio, GPIO_Number_5);//SCLK
		__asm(" NOP");
	}
}

void LED4_Display (void)
{
	unsigned char  *led_table;          // 查表指针
	uchar i;
	////////////显示4位数的个位
	led_table = LED_0F + LED[0];
	i = *led_table;
	LED_OUT(i);
	LED_OUT(0x01);
	GPIO_setLow(myGpio, GPIO_Number_4);//RCLK = 0
	__asm(" NOP");
	GPIO_setHigh(myGpio, GPIO_Number_4);//RCLK = 1;
	__asm(" NOP");
	////////////显示4位数的十位
	led_table = LED_1F + LED[1];
	i = *led_table;
	LED_OUT(i);
	LED_OUT(0x02);
	GPIO_setLow(myGpio, GPIO_Number_4);//RCLK = 0
	__asm(" NOP");
	GPIO_setHigh(myGpio, GPIO_Number_4);//RCLK = 1;
	__asm(" NOP");
	///////////////显示4位数的百位
	led_table = LED_0F + LED[2];
	i = *led_table;

	LED_OUT(i);
	LED_OUT(0x04);

	GPIO_setLow(myGpio, GPIO_Number_4);//RCLK = 0
	__asm(" NOP");
	GPIO_setHigh(myGpio, GPIO_Number_4);//RCLK = 1;
	__asm(" NOP");
	///////////////显示4位数的千位
	led_table = LED_0F + LED[3];
	i = *led_table;
	LED_OUT(i);
	LED_OUT(0x08);
	GPIO_setLow(myGpio, GPIO_Number_4);//RCLK = 0
	__asm(" NOP");
	GPIO_setHigh(myGpio, GPIO_Number_4);//RCLK = 1;
	__asm(" NOP");
}
/////////SPI//////
void spi_init()
{
    CLK_enableSpiaClock(myClk);

    // Reset on, rising edge, 16-bit char bits
    SPI_setCharLength(mySpi, SPI_CharLength_16_Bits);

    // Enable master mode, normal phase,
    // enable talk, and SPI int disabled.
    SPI_setMode(mySpi, SPI_Mode_Master);
    SPI_enableTx(mySpi);

    SPI_setBaudRate(mySpi, SPI_BaudRate_1_MBaud);

    // Relinquish SPI from Reset
    SPI_enableLoopBack(mySpi);
    SPI_enable(mySpi);

    // Set so breakpoints don't disturb xmission
    SPI_setPriority(mySpi, SPI_Priority_FreeRun);

    return;
}

void spi_fifo_init()
{
    // Initialize SPI FIFO registers
    SPI_enableChannels(mySpi);
    SPI_enableFifoEnh(mySpi);
    SPI_resetTxFifo(mySpi);
    SPI_clearTxFifoInt(mySpi);
    SPI_resetRxFifo(mySpi);
    SPI_clearRxFifoInt(mySpi);
    SPI_setRxFifoIntLevel(mySpi, SPI_FifoLevel_4_Words);

    return;
}

Uint16 slip()
{

	uint16_t sum=0;
	int sumcount;
	Volt1[slipcount] = ADC_readResult(myAdc, ADC_ResultNumber_1);


		slipcount++;


		if(slipcount==9)
					slipcount=0;
		for(sumcount=0;sumcount<9;sumcount++)
			sum=sum+Volt1[slipcount];

		volt=sum/9;
		return(volt);


}


