/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

typedef enum{
	flag_reverse = 0 ,
	
	flag_DEFAULT	
}Flag_Index;


uint8_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint8_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))



/*
	Target : 200K Freq
	DUTY : 50%
	
	SYS_CLK : 48M
	PSC : 1

	48 000 000/200 000 = PSC * (CNR + 1)
	CNR = (SYS_CLK/FREQ)/PSC - 1 = 239

	50 /100 = CMR / (CNR + 1)
	CMR = 50 * (CNR + 1)/100
	
*/

#define SYS_CLK 									(4000000ul)	//(48000000ul)
#define PWM_PSC 								(1000)	
#define PWM_FREQ 								(1)	
#define PWM_DUTY                              	(0)

#define ENABLE_CKO
//#define ENABLE_LED

//#define ENABLE_PWM_CH0
#define ENABLE_PWM_CH4

#if defined (ENABLE_PWM_CH0)
#define PWM_CHANNEL                           	(0)
#define PWM_CHANNEL_MASK                     (PWM_CH_0_MASK)
#elif defined (ENABLE_PWM_CH4)
#define PWM_CHANNEL                           	(4)
#define PWM_CHANNEL_MASK                     (PWM_CH_4_MASK)
#endif

//16 bit
#define PWM_CNR 								((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)
#define PWM_CMR 								(PWM_DUTY * (PWM_CNR + 1)/1000)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	(u32DutyCycle * (PWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(PWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))

static uint16_t duty = 500;	// 1 ~ 1000 , 0.1 % to 100%


enum
{
	CKO_FREQ_24M = 0 ,
	CKO_FREQ_12M,
	CKO_FREQ_6M,
	CKO_FREQ_3M,
	CKO_FREQ_1_5M,
	CKO_FREQ_750K,
	CKO_FREQ_375K,
	CKO_FREQ_187_5K,
	CKO_FREQ_93_75K,

	CKO_FREQ_46_875K,
};

void CKO_Test(void)
{
	uint8_t DirectOutput = 1;
	/*
		CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1)
	*/
//    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HXT, 0, DirectOutput);
//    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HIRC, 0, DirectOutput);	
//    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_PLL, CKO_FREQ_750K, 0);	
//    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_PLL, CKO_FREQ_6M, DirectOutput);	
//    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_PLL, CKO_FREQ_24M, DirectOutput);	
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, CKO_FREQ_6M, DirectOutput);	
}

void PWM_Change(void)
{

	CalNewDuty(PWM0, PWM_CHANNEL, duty, 1000);

	if (is_flag_set(flag_reverse))
	{
		duty++;	
	}
	else
	{
		duty--;
	}

	if (duty == 1000)
	{
		set_flag(flag_reverse , DISABLE);				
	}
	else if (duty == 0)
	{
		set_flag(flag_reverse , ENABLE);
	}
	
}

void PWM_Set_Duty(PWM_T *pwm,uint32_t u32ChannelNum,uint32_t u32DutyCycle,uint32_t u32CycleResolution)		// 1 ~ 1000 , 0.1 % to 100%
{
    uint32_t u32NewCMR = 0;
	u32NewCMR = CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution);    
	PWM_SET_CMR(pwm, u32ChannelNum, u32NewCMR);
}


void PWM0_Init(void)
{
    /*
      Configure PWM0 channel 0 init period and duty(down counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = CMR / (CNR + 1)
      
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = 100 / (199 + 1) = 50%
    */
	
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, PWM_CHANNEL, PWM_PSC - 1);
	
    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, PWM_CHANNEL, PWM_CNR);
	
    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, PWM_CHANNEL, PWM_CMR);	
	
    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CHANNEL_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);
	
    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CHANNEL_MASK);
	
	PWM_Start(PWM0, PWM_CHANNEL_MASK);
	
	set_flag(flag_reverse , ENABLE);

	CalNewDuty(PWM0, PWM_CHANNEL, 500, 1000);
	
}

void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
}

void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	static uint16_t CNT = 0;

//	static uint16_t CNT_PWM = 1;
//	static uint16_t duty = 500;	// 1 ~ 1000 , 0.1 % to 100%

    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);

//		if (CNT_PWM++ == 500)
//		{
//			CNT_PWM = 1;
			
//			PWM_Set_Duty(PWM0, PWM_CHANNEL, duty, 1000);
//			PWM_Change();
			
//			PB14 ^= 1;
//		}		

		if (CNT++ >= 1000)
		{		
			CNT = 0;	

			#if defined (ENABLE_LED)
			PB14 ^= 1;
			#endif
			
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
		}
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}


void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        TIMER_ClearIntFlag(TIMER0);

    }
}


void TIMER0_Init(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);	
    TIMER_Start(TIMER0);
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

//	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
//	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
//	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
//	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
//	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
//	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    GPIO_SetMode(PF, BIT2|BIT3, GPIO_MODE_INPUT);

	#if 1
	CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);
	CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);
	#else
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);	
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	#endif

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
//    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));

    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

#if defined (ENABLE_PWM_CH0)
//    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk)) | SYS_GPA_MFPL_PA5MFP_PWM0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_PWM0_CH0;
#elif defined (ENABLE_PWM_CH4)
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_PWM0_CH4;	
#endif

#if defined (ENABLE_CKO)
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_CLKO;
#endif
	
    /* Update System Core Clock */
    SystemCoreClockUpdate();

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
    SYS_Init();

    UART0_Init();

	#if defined (ENABLE_LED)
	GPIO_Init();
	#endif

	PWM0_Init();

	TIMER0_Init();
	TIMER1_Init();

	#if defined (ENABLE_CKO)
	CKO_Test();
	#endif

    /* Got no where to go, just loop forever */
    while(1)
    {

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
