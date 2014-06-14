#include "contiki.h"
#include "dev/leds.h"
#include "platform-conf.h"

#include "cpu.h"

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
	/* All LEDs are on PWM pins, so can be brightness controlled.
	 * All LEDs are connected to that sending pin LOW illuminates
	 * LED and sending pin HIGH extinguishes. */
	
	/* Make sure that needed ports are enabled so that we don't hardfault. */
	port_enable(LED_RED_PORT_MASK | LED_GREEN_PORT_MASK | LED_BLUE_PORT_MASK);
	
	/* LED Blue (yellow). */
	LED_BLUE_PCR &= ~PORT_PCR_MUX_MASK;							/* Clear PCR Multiplex. */
	LED_BLUE_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);		/* Clear ISF & set MUX to be basic pin. */
	LED_BLUE_PORT(PDDR) |= BV(LED_BLUE_PIN);					/* Set pin as Output. */ 
	LED_BLUE_PORT(PSOR) = BV(LED_BLUE_PIN);						/* Turn off LED. */
    
	/* LED Red. */
	LED_RED_PCR &= ~PORT_PCR_MUX_MASK;							/* Clear PCR Multiplex. */
	LED_RED_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);		/* Clear ISF & set MUX to be basic pin. */
	LED_RED_PORT(PDDR) |= BV(LED_RED_PIN);						/* Set pin as Output. */ 
	LED_RED_PORT(PSOR) = BV(LED_RED_PIN);						/* Turn off LED. */
	
	/* LED Green. */
	LED_GREEN_PCR &= ~PORT_PCR_MUX_MASK;						/* Clear PCR Multiplex. */
	LED_GREEN_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);	/* Clear ISF & set MUX to be basic pin. */
	LED_GREEN_PORT(PDDR) |= BV(LED_GREEN_PIN);					/* Set pin as Output. */ 
	LED_GREEN_PORT(PSOR) = BV(LED_GREEN_PIN);					/* Turn off LED. */
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
	unsigned char leds;
	if(LED_BLUE_PORT(PDOR) & BV(LED_BLUE_PIN))
	{
		leds |= LEDS_BLUE;
	}
	if(LED_GREEN_PORT(PDOR) & BV(LED_GREEN_PIN))
	{
		leds |= LEDS_GREEN;
	}
	if(LED_RED_PORT(PDOR) & BV(LED_RED_PIN))
	{
		leds |= LEDS_RED;
	}
	return leds;
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
	if(leds & LEDS_BLUE)	
	{
		LED_BLUE_PORT(PDOR) &= ~BV(LED_BLUE_PIN);		/* Turn on the LED. */
	}
	else
	{
		LED_BLUE_PORT(PDOR) |= BV(LED_BLUE_PIN);		/* Turn off the LED. */
	}
	
	if(leds & LEDS_RED)	
	{
		LED_RED_PORT(PDOR) &= ~BV(LED_RED_PIN);			/* Turn on the LED. */
	}
	else
	{
		LED_RED_PORT(PDOR) |= BV(LED_RED_PIN);			/* Turn off the LED. */
	}
	
	if(leds & LEDS_GREEN)	
	{
		LED_GREEN_PORT(PDOR) &= ~BV(LED_GREEN_PIN);		/* Turn on the LED. */
	}
	else
	{
		LED_GREEN_PORT(PDOR) |= BV(LED_GREEN_PIN);		/* Turn off the LED. */
	}
}