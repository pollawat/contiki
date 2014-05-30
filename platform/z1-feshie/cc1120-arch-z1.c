
/* 
 * Copyright (c) 2014, University of Southampton, Electronics and Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */				

 /**
 * \file
 *         Architecture specific CC1120 functions for the Zolertia Z1 
 * \author
 *         Graeme Bragg <g.bragg@ecs.soton.ac.uk>
 *         Phil Basford <pjb@ecs.soton.ac.uk>
 */

#include "cc1120.h"
#include "cc1120-arch.h"

#include "dev/spi.h"


#include "isr_compat.h"
#include <stdio.h>



/* ---------------------------- Init Functions ----------------------------- */
/*---------------------------------------------------------------------------*/

void
cc1120_arch_init(void)
{
	/* Configure pins.  This may have already been done but func is repeat-execution safe. */
	cc1120_arch_pin_init();
	
	/* Init SPI.  May have already done but we need to ensure SPI is configured.  On Z1 this is done in main. */
	//spi_init();

	/* Setup GPIO pins. */
	CC1120_GDO0_PORT(SEL) &= ~BV(CC1120_GDO0_PIN);
	CC1120_GDO0_PORT(DIR) &= ~BV(CC1120_GDO0_PIN);
	
#ifdef CC1120_GPIO2_PRESENT	
	CC1120_GDO2_PORT(SEL) &= ~BV(CC1120_GDO2_PIN);
	CC1120_GDO2_PORT(DIR) &= ~BV(CC1120_GDO2_PIN);
#endif

#ifdef CC1120_GPIO3_PRESENT	
	CC1120_GDO3_PORT(SEL) &= ~BV(CC1120_GDO3_PIN);
	CC1120_GDO3_PORT(DIR) &= ~BV(CC1120_GDO3_PIN);
#endif
	
}

/*---------------------------------------------------------------------------*/

void 
cc1120_arch_pin_init(void)
{
	/* Turn off CC2420. */
	CC2420_PWR_PORT(DIR) |= BV(CC2420_PWR_PIN);
	CC2420_PWR_PORT(OUT) &= ~BV(CC2420_PWR_PIN); 
	CC2420_CSN_PORT(DIR) |= BV(CC2420_CSN_PIN);
	CC2420_CSN_PORT(OUT) |= BV(CC2420_CSN_PIN); 
	CC2420_RESET_PORT(DIR) |= BV(CC2420_RESET_PIN);
	CC2420_RESET_PORT(OUT) |= BV(CC2420_RESET_PIN);
	
	/* Setup !RESET and CSn pins. */
	CC1120_SPI_CSN_PORT(DIR) |= BV(CC1120_SPI_CSN_PIN);		/* Set CSn pin to Output. */
	CC1120_SPI_CSN_PORT(SEL) &= ~BV(CC1120_SPI_CSN_PIN);	/* Set CSn pin to GPIO mode. */
	CC1120_SPI_CSN_PORT(OUT) |= BV(CC1120_SPI_CSN_PIN);		/* Set CSn high to de-select radio. */

	CC1120_RESET_PORT(DIR) |= BV(CC1120_RESET_PIN);			/* Set !Reset pin to Output. */
	CC1120_RESET_PORT(SEL) &= ~BV(CC1120_RESET_PIN);		/* Set !Reset pin to GPIO mode. */
	CC1120_RESET_PORT(OUT) |= BV(CC1120_RESET_PIN);			/* Set !Reset high to ensure radio does not hog SPI. */
}


/* ---------------------------- Reset Functions ---------------------------- */
/*---------------------------------------------------------------------------*/
void
cc1120_arch_reset(void)
{
	CC1120_SPI_CSN_PORT(OUT) |= BV(CC1120_SPI_CSN_PIN);		/* Assert CSn to de-select CC1120. */
	CC1120_RESET_PORT(OUT) &= ~BV(CC1120_RESET_PIN);		/* Clear !Reset pin. */
	clock_delay_usec(CC1120_RESET_DELAY_USEC);				/* Delay for a little. */
	CC1120_RESET_PORT(OUT) |= BV(CC1120_RESET_PIN);			/* Assert !Reset pin. */
}


/* ----------------------------- SPI Functions ----------------------------- */
/*---------------------------------------------------------------------------*/
void
cc1120_arch_enable(void)
{
	/* Set CSn to low to select CC1120 */
	CC1120_SPI_CSN_PORT(OUT) &= ~BV(CC1120_SPI_CSN_PIN);

	/* The MISO pin should go LOW before chip is fully enabled. */
	while((CC1120_SPI_MISO_PORT(IN) & BV(CC1120_SPI_MISO_PIN)) != 0);
	// TODO: Include a timeout here and change to have a return code?
}

/*---------------------------------------------------------------------------*/
void
cc1120_arch_disable(void)
{
	/* Check if MISO is high at disable.  If it is, send a SNOP to clear it. */
	if((CC1120_SPI_MISO_PORT(IN) & BV(CC1120_SPI_MISO_PIN)) != 0)
	{
		(void) CC11xx_ARCH_SPI_RW_BYTE(CC11xx_SNOP);
	}
	// TODO: Make this more resilient?

	/* Set CSn to high (1) */
	CC1120_SPI_CSN_PORT(OUT) |= BV(CC1120_SPI_CSN_PIN);
}

/*---------------------------------------------------------------------------*/
uint8_t
cc1120_arch_rw_byte(uint8_t val)
{
	SPI_WAITFORTx_BEFORE();
	SPI_TXBUF = val;
	SPI_WAITFOREORx();
	return SPI_RXBUF;
}

/*---------------------------------------------------------------------------*/
uint8_t
cc1120_arch_rw_buf(uint8_t *inBuf, uint8_t *outBuf, uint8_t len)
{
	uint8_t i, ret;
	if(inBuf == NULL && outBuf == NULL) 	/* error: both buffers are NULL */
	{
		return 0x0f;		/* Return an odd status. */
	}
	else if(inBuf == NULL) 					/* Input buffer empty, TX only. */
	{
		for(i = 0; i < len; i++) 
		{
			SPI_WAITFORTx_BEFORE();			/* Wait for TX buf to be empty. */
			SPI_TXBUF = outbuf[i];			/* Transmit current byte. */
			SPI_WAITFOREORx();				/* Wait for TX & RX to complete. */
			if(i ==0)
			{
				ret = SPI_RXBUF				/* Set return to be status byte */
			}
		}
	} 
	else if(outBuf == NULL) 				/* Output buffer empty, read only. */
	{
		for(i = 0; i < len; i++) 
		{
			SPI_WAITFORTx_BEFORE();			/* Wait for TX buf to be empty. */
			SPI_TXBUF = 0;					/* Dummy TX to get RX. */
			SPI_WAITFOREORx();				/* Wait for TX & RX to complete. */
			inbuf[i] = SPI_RXBUF;			/* Write read byte to the buffer. */
		}
	} 
	else 
	{
		for(i = 0; i < len; i++) 
		{
			SPI_WAITFORTx_BEFORE();			/* Wait for TX buf to be empty. */
			SPI_TXBUF = outBuf[i];			/* Transmit current byte. */
			SPI_WAITFOREORx();				/* Wait for TX & RX to complete. */
			inBuf[i] = SPI_RXBUF;			/* Write read byte to the buffer. */
		}
		ret = inbuf[0];						/* Get the status byte for return. */
	}
	
	ret &= CC1120_STATUS_CHIP_RDY_MASK | CC1120_STATUS_STATE_MASK;	/* Get just the status and RDY for return. */
	
	return ret;
}


/* -------------------------- Interrupt Functions -------------------------- */

/* On the Z1, the interrupt is shared with the ADXL345 accelerometer.  
 * The interrupt routine is handled in adcl345.c. */
 
/*---------------------------------------------------------------------------*/
void
cc1120_arch_interrupt_enable(void)
{
	/* Reset interrupt trigger */
	CC1120_GDO0_PORT(IFG) &= ~BV(CC1120_GDO0_PIN);
	/* Enable interrupt on the GDO0 pin */
	CC1120_GDO0_PORT(IE) |= BV(CC1120_GDO0_PIN);
}

/*---------------------------------------------------------------------------*/
void
cc1120_arch_interrupt_disable(void)
{
	/* Disable interrupt on the GDO0 pin */
	CC1120_GDO0_PORT(IE) &= ~BV(CC1120_GDO0_PIN);
	/* Reset interrupt trigger */
	CC1120_GDO0_PORT(IFG) &= ~BV(CC1120_GDO0_PIN);
}
/*---------------------------------------------------------------------------*/



