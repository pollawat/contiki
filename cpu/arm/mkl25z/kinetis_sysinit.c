/*
 *    kinetis_sysinit.c - Default init routines for Flycatcher
 *                     		Kinetis ARM systems
 *    Copyright � 2012 Freescale semiConductor Inc. All Rights Reserved.
 *    
 *    Modified by Graeme Bragg, g.bragg@ecs.soton.ac.uk, 23/1/2014:
 *    	Separate Default interrupt handler for each interrupt to make debugging easier.
 */
 
#include "kinetis_sysinit.h"
#include "derivative.h"
#include <stdlib.h>
#include <stdio.h>

#define DEFHANDLERDEBUG 	1

/**
 **===========================================================================
 **  External declarations
 **===========================================================================
 */
#if __cplusplus
extern "C" {
#endif
extern uint32_t __vector_table[];
extern unsigned long _estack;
extern uint32_t __SP_INIT;
extern void __thumb_startup( void );
#if __cplusplus
}
#endif

/**
 **===========================================================================
 **  Default interrupt handlers
 **===========================================================================
 */
void Default_Handler()
{
#if DEFHANDLERDEBUG	
	printf("Default Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_NMI()
{
#if DEFHANDLERDEBUG
	printf("NMI Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_HardFault()
{
#if DEFHANDLERDEBUG
	printf("HardFault Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_SVC()
{
#if DEFHANDLERDEBUG
	printf("SVC Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_PendSV()
{
#if DEFHANDLERDEBUG
	printf("PendSV Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_SysTick()
{
#if DEFHANDLERDEBUG
	printf("Systick Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_DMA0()
{
#if DEFHANDLERDEBUG
	printf("DMA0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_DMA1()
{
#if DEFHANDLERDEBUG
	printf("DMA1 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_DMA2()
{
#if DEFHANDLERDEBUG
	printf("DMA2 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_DMA3()
{
#if DEFHANDLERDEBUG
	printf("DMA3 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_MCM()
{
#if DEFHANDLERDEBUG
	printf("MCM Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_FTFL()
{
#if DEFHANDLERDEBUG
	printf("FTFL Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_PMC()
{
#if DEFHANDLERDEBUG
	printf("PMC Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_LLW()
{
#if DEFHANDLERDEBUG
	printf("LLW Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_I2C0()
{
#if DEFHANDLERDEBUG
	printf("I2C0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_I2C1()
{
#if DEFHANDLERDEBUG
	printf("I2C1 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_SPI0()
{
#if DEFHANDLERDEBUG
	printf("SPI0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_SPI1()
{
#if DEFHANDLERDEBUG
	printf("SPI1 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_UART0()
{
#if DEFHANDLERDEBUG
	printf("UART0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_UART1()
{
#if DEFHANDLERDEBUG
	printf("UART1 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_UART2()
{
#if DEFHANDLERDEBUG
	printf("UART2 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_ADC0()
{
#if DEFHANDLERDEBUG
	printf("ADC0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_CMP0()
{
#if DEFHANDLERDEBUG
	printf("CMP0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_FTM0()
{
#if DEFHANDLERDEBUG
	printf("FTM0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_FTM1()
{
#if DEFHANDLERDEBUG
	printf("FTM1 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_FTM2()
{
#if DEFHANDLERDEBUG
	printf("FTM2 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_RTC_Alarm()
{
#if DEFHANDLERDEBUG
	printf("RTC Alarm Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_RTC_Seconds()
{
#if DEFHANDLERDEBUG
	printf("RTC Seconds Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_PIT()
{
#if DEFHANDLERDEBUG
	printf("PIT Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_USBOTG()
{
#if DEFHANDLERDEBUG
	printf("USBOTG Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_DAC0()
{
#if DEFHANDLERDEBUG
	printf("DAC0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_TSI0()
{
#if DEFHANDLERDEBUG
	printf("TSI0 Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_MCG()
{
#if DEFHANDLERDEBUG
	printf("MCG Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_LPTimer()
{
#if DEFHANDLERDEBUG
	printf("LPTMR Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_PORTA()
{
#if DEFHANDLERDEBUG
	printf("PORTA Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

void Default_Handler_PORTD()
{
#if DEFHANDLERDEBUG
	printf("PORTD Handler - BREAKPOINT");
#endif
	__asm("bkpt");
}

/**
 **===========================================================================
 **  Reset handler
 **===========================================================================
 */

void __init_hardware(void)
{
  /*** ### MKL25Z128VLK4 "Cpu" init code ... ***/
  /*** PE initialization code after reset ***/
	SCB_VTOR = (uint32_t)__vector_table; /* Set the interrupt vector table position */
	//SCB_VTOR = (uint32_t)(&__vect_table); /* Set the interrupt vector table position */
	// Disable the Watchdog because it may reset the core before entering main().
	SIM_COPC = KINETIS_WDOG_DISABLED_CTRL;
}

/* Weak definitions of handlers point to Default_Handler if not implemented */
void NMI_Handler() __attribute__ ((weak, alias("Default_Handler_NMI")));
void HardFault_Handler() __attribute__ ((weak, alias("Default_Handler_HardFault")));
void SVC_Handler() __attribute__ ((weak, alias("Default_Handler_SVC")));
void PendSV_Handler() __attribute__ ((weak, alias("Default_Handler_PendSV")));
void SysTick_Handler() __attribute__ ((weak, alias("Default_Handler_SysTick")));

void DMA0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_DMA0")));
void DMA1_IRQHandler() __attribute__ ((weak, alias("Default_Handler_DMA1")));
void DMA2_IRQHandler() __attribute__ ((weak, alias("Default_Handler_DMA2")));
void DMA3_IRQHandler() __attribute__ ((weak, alias("Default_Handler_DMA3")));
void MCM_IRQHandler() __attribute__ ((weak, alias("Default_Handler_MCM")));
void FTFL_IRQHandler() __attribute__ ((weak, alias("Default_Handler_FTFL")));
void PMC_IRQHandler() __attribute__ ((weak, alias("Default_Handler_PMC")));
void LLW_IRQHandler() __attribute__ ((weak, alias("Default_Handler_LLW")));
void I2C0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_I2C0")));
void I2C1_IRQHandler() __attribute__ ((weak, alias("Default_Handler_I2C1")));
void SPI0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_SPI0")));
void SPI1_IRQHandler() __attribute__ ((weak, alias("Default_Handler_SPI1")));
void UART0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_UART0")));
void UART1_IRQHandler() __attribute__ ((weak, alias("Default_Handler_UART1")));
void UART2_IRQHandler() __attribute__ ((weak, alias("Default_Handler_UART2")));
void ADC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_ADC0")));
void CMP0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_CMP0")));
void FTM0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_FTM0")));
void FTM1_IRQHandler() __attribute__ ((weak, alias("Default_Handler_FTM1")));
void FTM2_IRQHandler() __attribute__ ((weak, alias("Default_Handler_FTM2")));
void RTC_Alarm_IRQHandler() __attribute__ ((weak, alias("Default_Handler_RTC_Alarm")));
void RTC_Seconds_IRQHandler() __attribute__ ((weak, alias("Default_Handler_RTC_Seconds")));
void PIT_IRQHandler() __attribute__ ((weak, alias("Default_Handler_PIT")));
void USBOTG_IRQHandler() __attribute__ ((weak, alias("Default_Handler_USBOTG")));
void DAC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_DAC0")));
void TSI0_IRQHandler() __attribute__ ((weak, alias("Default_Handler_TSI0")));
void MCG_IRQHandler() __attribute__ ((weak, alias("Default_Handler_MCG")));
void LPTimer_IRQHandler() __attribute__ ((weak, alias("Default_Handler_LPTimer")));
void PORTA_IRQHandler() __attribute__ ((weak, alias("Default_Handler_PORTA")));
void PORTD_IRQHandler() __attribute__ ((weak, alias("Default_Handler_PORTD")));

/* The Interrupt Vector Table */
void (* const InterruptVector[])() __attribute__ ((section(".vectortable"))) = {
    /* Processor exceptions */
    (void(*)(void)) &_estack,
    __thumb_startup,
    NMI_Handler,
    HardFault_Handler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SVC_Handler,
    0,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* Interrupts */
    DMA0_IRQHandler, /* DMA Channel 0 Transfer Complete and Error */
    DMA1_IRQHandler, /* DMA Channel 1 Transfer Complete and Error */
    DMA2_IRQHandler, /* DMA Channel 2 Transfer Complete and Error */
    DMA3_IRQHandler, /* DMA Channel 3 Transfer Complete and Error */
    MCM_IRQHandler, /* Normal Interrupt */
    FTFL_IRQHandler, /* FTFL Interrupt */
    PMC_IRQHandler, /* PMC Interrupt */
    LLW_IRQHandler, /* Low Leakage Wake-up */
    I2C0_IRQHandler, /* I2C0 interrupt */
    I2C1_IRQHandler, /* I2C1 interrupt */
    SPI0_IRQHandler, /* SPI0 Interrupt */
    SPI1_IRQHandler, /* SPI1 Interrupt */
    UART0_IRQHandler, /* UART0 Status and Error interrupt */
    UART1_IRQHandler, /* UART1 Status and Error interrupt */
    UART2_IRQHandler, /* UART2 Status and Error interrupt */
    ADC0_IRQHandler, /* ADC0 interrupt */
    CMP0_IRQHandler, /* CMP0 interrupt */
    FTM0_IRQHandler, /* FTM0 fault, overflow and channels interrupt */
    FTM1_IRQHandler, /* FTM1 fault, overflow and channels interrupt */
    FTM2_IRQHandler, /* FTM2 fault, overflow and channels interrupt */
    RTC_Alarm_IRQHandler, /* RTC Alarm interrupt */
    RTC_Seconds_IRQHandler, /* RTC Seconds interrupt */
    PIT_IRQHandler, /* PIT timer all channels interrupt */
    Default_Handler, /* Reserved interrupt 39/23 */
    USBOTG_IRQHandler, /* USB interrupt */
    DAC0_IRQHandler, /* DAC0 interrupt */
    TSI0_IRQHandler, /* TSI0 Interrupt */
    MCG_IRQHandler, /* MCG Interrupt */
    LPTimer_IRQHandler, /* LPTimer interrupt */
    Default_Handler, /* Reserved interrupt 45/29 */
    PORTA_IRQHandler, /* Port A interrupt */
    PORTD_IRQHandler /* Port D interrupt */
};
