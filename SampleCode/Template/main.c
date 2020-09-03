/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 , 
	
	ADC0_CH2 , 
	ADC0_CH3 , 
	ADC0_CH4 , 
	ADC0_CH5 , 
	ADC0_CH6 , 
	ADC0_CH7 , 
	ADC0_CH8 , 
	ADC0_CH9 , 
	ADC0_CH10 , 
	ADC0_CH11 , 
	ADC0_CH12 ,
	ADC0_CH13 , 
	ADC0_CH14 , 
	ADC0_CH15 , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;

typedef struct
{	
	uint8_t idx ;
	uint8_t adc_ch;
}ADC_struct;

const ADC_struct adc_data[] =
{
	{1 		, ADC0_CH0},
	{2 		, ADC0_CH4},
	{3 		, ADC0_CH7},
	{4 		, ADC0_CH10},

	{0xFF 	, ADC_CH_DEFAULT},	
};

#define ADC_SAMPLETIME_MS					(uint16_t) (10)

#define ADC_RESOLUTION						((uint16_t)(4096u))
#define ADC_REF_VOLTAGE						((uint16_t)(3300u))	//(float)(3.3f)

#define ABS(X)  								((X) > 0 ? (X) : -(X)) 

#define ADC_DIGITAL_SCALE(void) 				(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 
#define ADC_CALC_DATA_TO_VOLTAGE(DATA,VREF) ((DATA) * (VREF) / ADC_DIGITAL_SCALE())

#define ADCextendSampling 					(0)

#define ADC_CH_NUM	 						(4)

volatile uint16_t aADCxCH = 0;
uint32_t AVdd = 0;
uint16_t ADCxConvertedData[ADC_CH_NUM] = {0};

typedef enum{
	flag_ADC_Data_Ready = 0 ,
	
	flag_DEFAULT	
}flag_Index;

#define HIBYTE(v1)              					((uint8_t)((v1)>>8))                      //v1 is UINT16
#define LOBYTE(v1)              					((uint8_t)((v1)&0xFF))

#define	USE_SIMPLE_AVG

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))



__STATIC_INLINE uint32_t FMC_ReadBandGap(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = 0x70u;                         /* Must keep 0x70 when read Band-Gap */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */

    return FMC->ISPDAT & 0xFFF;
}

void convertDecToBin(int n)
{
    int k = 0;
    unsigned char *p = (unsigned char*)&n;
    int val2 = 0;
    int i = 0;
    for(k = 0; k <= 1; k++)
    {
        val2 = *(p+k);
        for (i = 7; i >= 0; i--)
        {
            if(val2 & (1 << i))
                printf("1");
            else
                printf("0");
        }
        printf(" ");
    }
}

void ADC_IRQHandler(void)
{
	uint8_t ch_idx = 0;

	for (ch_idx = 0 ; adc_data[ch_idx].idx < 0xFF ; ch_idx++)
	{
		if (aADCxCH == adc_data[ch_idx].adc_ch)
		{
			break;
		}
		else
		{
			continue;
		}
	}
	ADCxConvertedData[ch_idx] = ADC_GET_CONVERSION_DATA(ADC, aADCxCH);
	
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
}

void ADC_ReadAVdd(void)
{
    int32_t  i32ConversionData;
    int32_t  i32BuiltInData;

    ADC_POWER_ON(ADC);
    CLK_SysTickDelay(10000);

	
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT29);
    ADC_SetExtendSampleTime(ADC, 0, 71);
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);
    ADC_START_CONV(ADC);

    ADC_DISABLE_INT(ADC, ADC_ADF_INT);
		
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 29);
    SYS_UnlockReg();
    FMC_Open();
    i32BuiltInData = FMC_ReadBandGap();	

	AVdd = 3072*i32BuiltInData/i32ConversionData;

//	printf("%s : %d,%d,%d\r\n",__FUNCTION__,AVdd, i32ConversionData,i32BuiltInData);

    NVIC_DisableIRQ(ADC_IRQn);
	
}

void ADC_InitChannel(uint8_t ch)
{
//	ADC_ReadAVdd();

//    /* Enable ADC converter */
//    ADC_POWER_ON(ADC);

//    /*Wait for ADC internal power ready*/
//    CLK_SysTickDelay(10000);

    /* Set input mode as single-end, and Single mode*/
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE,(uint32_t) 0x1 << ch);//BIT0|BIT4

    /* To sample band-gap precisely, the ADC capacitor must be charged at least 3 us for charging the ADC capacitor ( Cin )*/
    /* Sampling time = extended sampling time + 1 */
    /* 1/24000000 * (Sampling time) = 3 us */
	/*
	    printf("+----------------------------------------------------------------------+\n");
	    printf("|   ADC clock source -> PCLK1  = 48 MHz                                |\n");
	    printf("|   ADC clock divider          = 2                                     |\n");
	    printf("|   ADC clock                  = 48 MHz / 2 = 24 MHz                   |\n");
	    printf("|   ADC extended sampling time = 71                                    |\n");
	    printf("|   ADC conversion time = 17 + ADC extended sampling time = 88         |\n");
	    printf("|   ADC conversion rate = 24 MHz / 88 = 272.7 ksps                     |\n");
	    printf("+----------------------------------------------------------------------+\n");
	*/

    /* Set extend sampling time based on external resistor value.*/
    ADC_SetExtendSampleTime(ADC,(uint32_t) NULL, ADCextendSampling);

    /* Select ADC input channel */
    ADC_SET_INPUT_CHANNEL(ADC, 0x1 << ch);

	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	ADC_ENABLE_INT(ADC, ADC_ADF_INT);
	NVIC_EnableIRQ(ADC_IRQn);

    /* Start ADC conversion */
    ADC_START_CONV(ADC);
	
}

void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
}

void TMR3_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	static uint16_t CNT = 0;
	static uint16_t CNT_ADC = 0;
	static uint8_t flag_num = 0;
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);
	
		if (CNT++ >= 1000)
		{		
			CNT = 0;
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}

		if (CNT_ADC++ >= ADC_SAMPLETIME_MS)
		{		
			CNT_ADC = 0;

			aADCxCH = adc_data[flag_num].adc_ch;
			flag_num = (flag_num >= adc_data[flag_num].idx) ? (0) : (flag_num + 1) ;
		
			ADC_InitChannel(aADCxCH);

		}
		
    }
}


void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}


void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
//	UART_SetTimeoutCnt(UART0, 20);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	
    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
	
    CLK_EnableModuleClock(ADC_MODULE);	
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(2));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPB_MFPH = (SYS->GPB_MFPH &~( SYS_GPB_MFPH_PB10MFP_Msk )) \
                    | ( SYS_GPB_MFPH_PB10MFP_ADC0_CH10) ;

    SYS->GPB_MFPL = (SYS->GPB_MFPL &~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk| SYS_GPB_MFPL_PB7MFP_Msk)) \
                    | (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB4MFP_ADC0_CH4| SYS_GPB_MFPL_PB7MFP_ADC0_CH7) ;

    /* Set PB.0 ~ PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT4|BIT7|BIT10, GPIO_MODE_INPUT);

    /* Disable the PB0 ~ PB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT4|BIT7|BIT10);

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	uint8_t idx = 0;
	uint16_t adc_data = 0;	

    SYS_Init();

    UART0_Init();

	GPIO_Init();

	TIMER3_Init();

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /*Wait for ADC internal power ready*/
    CLK_SysTickDelay(10000);

    /* Got no where to go, just loop forever */
    while(1)
    {
		#if 1
		adc_data = ADCxConvertedData[idx];
//		convertDecToBin(adc_data);
		printf("0x%2X:0x%3X(%4dmv)," , idx , adc_data , ADC_CALC_DATA_TO_VOLTAGE(adc_data,ADC_REF_VOLTAGE));

		if (idx++ >= 3)
		{
			printf("\r\n");
			idx = 0;
		}
//		idx = (idx++ >= 3) ? (0) : (idx) ;
		
		#else
		convertDecToBin(adc_value1);
		printf(" 0x%4X , %4dmv " , adc_value1 , ADC_CALC_DATA_TO_VOLTAGE(adc_value1,ADC_REF_VOLTAGE));
		printf("      ");
		convertDecToBin(adc_value2);	
		printf(" 0x%4X , %4dmv\r\n" , adc_value2, ADC_CALC_DATA_TO_VOLTAGE(adc_value2,ADC_REF_VOLTAGE));
		#endif
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
