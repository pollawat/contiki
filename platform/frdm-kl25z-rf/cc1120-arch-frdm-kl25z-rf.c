
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

#include "contiki.h"
#include "contiki-net.h"

#include "cc1120.h"
#include "cc1120-arch.h"
#include "cc1120-const.h"

#include <MKL25Z4.h>                   /* I/O map for MKL25Z128VLK4 */
#include "cpu.h"
#include "spi.h"
#include "nvic.h"


#include <stdio.h>



/* ---------------------------- Init Functions ----------------------------- */
/*---------------------------------------------------------------------------*/

void
cc1120_arch_init(void)
{
#ifdef CC1120ARCHDEBUG
	printf("CC1120 Architecture Initialisation.\n");
#endif

#ifdef CC1120ARCHDEBUG
	printf("\tPins...\n");
#endif	
	/* Configure pins.  This may have already been done but func is repeat-execution safe. */
	cc1120_arch_pin_init();

#ifdef CC1120ARCHDEBUG
	printf("\tSPI0...\n");
#endif	
	/* Init SPI.  May have already done but we need to ensure SPI is configured.  On Z1 this is done in main. */
	SPI0_init();
	
#ifdef CC1120_GPIO2_FUNC	
	CC1120_GDO2_PORT(PDDR) &= ~BV(CC1120_GDO2_PIN);					/* Set pin as Input. */
	CC1120_GDO2_PCR &= ~PORT_PCR_MUX_MASK;							/* Clear PCR Multiplex. */
	CC1120_GDO2_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);		/* Clear ISF & set MUX to be basic pin. */
#endif

#ifdef CC1120_GPIO3_FUNC
	CC1120_GDO3_PORT(PDDR) &= ~BV(CC1120_GDO3_PIN);					/* Set pin as Input. */
	CC1120_GDO3_PCR &= ~PORT_PCR_MUX_MASK;							/* Clear PCR Multiplex. */
	CC1120_GDO3_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);		/* Clear ISF & set MUX to be basic pin. */
#endif
#ifdef CC1120ARCHDEBUG
	printf("\tArchitecture initialisation done.\n");
#endif
}

/*---------------------------------------------------------------------------*/

void 
cc1120_arch_pin_init(void)
{
	/* Configure CSn pin on Port D, Pin 0 */
	CC1120_SPI_CSN_PORT(PDDR) |= BV(CC1120_SPI_CSN_PIN);			/* Set pin as Output. */ 
	CC1120_SPI_CSN_PORT(PSOR) = BV(CC1120_SPI_CSN_PIN);				/* Set CSn bit. */
    CC1120_SPI_CSN_PCR &= ~PORT_PCR_MUX_MASK;						/* Clear PCR Multiplex. */
	CC1120_SPI_CSN_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);	/* Clear ISF & set MUX to be basic pin. */
                          
	/* Setup !RESET pin. */
	CC1120_RESET_PORT(PDDR) |= BV(CC1120_RESET_PIN);				/* Set pin as Output. */ 
	CC1120_RESET_PORT(PSOR) = BV(CC1120_RESET_PIN);				/* Set CSn bit. */
    CC1120_RESET_PCR &= ~PORT_PCR_MUX_MASK;							/* Clear PCR Multiplex. */
	CC1120_RESET_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);		/* Clear ISF & set MUX to be basic pin. */
 	
	/* Setup interrupt/GDO0 pin. */
	CC1120_GDO0_PORT(PDDR) &= ~BV(CC1120_GDO0_PIN);				/* Set pin as Input. */
	CC1120_GDO0_PCR &= ~PORT_PCR_MUX_MASK;						/* Clear PCR Multiplex. */
	CC1120_GDO0_PCR |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x01);	/* Clear ISF & set MUX to be basic pin. */
	
}


/* ---------------------------- Reset Functions ---------------------------- */
/*---------------------------------------------------------------------------*/
void
cc1120_arch_reset(void)
{
#ifdef CC1120ARCHDEBUG
	printf("CC1120 Arch reset....");
#endif
	CC1120_SPI_CSN_PORT(PSOR) = BV(CC1120_SPI_CSN_PIN);		/* Assert CSn to de-select CC1120. */
	CC1120_RESET_PORT(PCOR) = BV(CC1120_RESET_PIN);			/* Clear !Reset pin. */
	clock_delay_usec(2);										/* Delay for a little. */
	CC1120_RESET_PORT(PSOR) = BV(CC1120_RESET_PIN);			/* Assert !Reset pin. */
#ifdef CC1120ARCHDEBUG
	printf(" OK\n");
#endif
}


/* ----------------------------- SPI Functions ----------------------------- */
/*---------------------------------------------------------------------------*/
void
cc1120_arch_spi_enable(void)
{
	/* Set CSn to low (0) */
	CC1120_SPI_CSN_PORT(PCOR) = BV(CC1120_SPI_CSN_PIN);		/* Clear CSn bit. */

	/* The MISO pin should go LOW before chip is fully enabled. */
	while((CC1120_SPI_MISO_PORT(PDIR) & BV(CC1120_SPI_MISO_PIN)) != 0);
}

/*---------------------------------------------------------------------------*/
void
cc1120_arch_spi_disable(void)
{
	/* Check if MISO is high at disable.  If it is, send a SNOP to clear it. */
	if((CC1120_SPI_MISO_PORT(PDIR) & BV(CC1120_SPI_MISO_PIN)) != 0)
	{
		(void) cc1120_arch_spi_rw_byte(CC1120_STROBE_SNOP);
	}
	// TODO: Make this more resilient?

	/* Set CSn to high (1) */
	CC1120_SPI_CSN_PORT(PSOR) = BV(CC1120_SPI_CSN_PIN);		/* Set CSn bit. */
}

/*---------------------------------------------------------------------------*/
uint8_t
cc1120_arch_spi_rw_byte(uint8_t val)
{
	return SPI0_single_tx_rx(val);
}

/*---------------------------------------------------------------------------*/
uint8_t 
cc1120_arch_txfifo_load(uint8_t *packet, uint8_t packet_length)
{
	uint8_t status, i;
	status = cc1120_arch_spi_rw_byte(CC1120_FIFO_ACCESS | CC1120_STANDARD_BIT | CC1120_WRITE_BIT);
	cc1120_arch_spi_rw_byte(packet_length);
	for(i = 0; i < packet_length; i++)
	{
		cc1120_arch_spi_rw_byte(CC1120_FIFO_ACCESS | CC1120_STANDARD_BIT | CC1120_WRITE_BIT);
		cc1120_arch_spi_rw_byte(packet[i]);
	}
	
	return status;
}


/*---------------------------------------------------------------------------*/
void 
cc1120_arch_rxfifo_read(uint8_t *packet, uint8_t packet_length)
{
	uint8_t i;
	
	for(i = 0; i < packet_length; i++)
	{
		(void) cc1120_arch_spi_rw_byte(CC1120_FIFO_ACCESS | CC1120_STANDARD_BIT | CC1120_READ_BIT);
		packet[i] = cc1120_arch_spi_rw_byte(0);
	}
	
}


/*---------------------------------------------------------------------------*/
uint8_t 
cc1120_arch_read_cca(void)
{
	if((CC1120_GDO3_PORT(PDIR) & BV(CC1120_GDO3_PIN)) == BV(CC1120_GDO3_PIN))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/*---------------------------------------------------------------------------*/
uint8_t
cc1120_arch_read_gpio0(void)
{
	if((CC1120_GDO0_PORT(PDIR) & BV(CC1120_GDO0_PIN)) == BV(CC1120_GDO0_PIN))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/* -------------------------- Interrupt Functions -------------------------- */

void
cc1120_arch_interrupt_enable(void)
{
	/* Reset interrupt trigger */
	CC1120_GDO0_PCR |= PORT_PCR_ISF_MASK;
	/* Enable rising edge interrupt on the GDO0 pin */
	CC1120_GDO0_PCR |= (PORT_PCR_IRQC(0x09) & PORT_PCR_IRQC_MASK);
}

/*---------------------------------------------------------------------------*/
void
cc1120_arch_interrupt_disable(void)
{
	/* Disable interrupt on the GDO0 pin */
	CC1120_GDO0_PCR &= ~PORT_PCR_IRQC_MASK;
	/* Reset interrupt trigger */
	CC1120_GDO0_PCR |= PORT_PCR_ISF_MASK;
}
/*---------------------------------------------------------------------------*/
void
cc1120_arch_interrupt_acknowledge(void)
{
	/* Reset interrupt trigger */
	CC1120_GDO0_PCR |= PORT_PCR_ISF_MASK;
}

/*---------------------------------------------------------------------------*/
/* Interrupt Handler. */
void
PORTA_IRQHandler(void)
{
	if(CC1120_GDO0_PCR & PORT_PCR_ISF_MASK)
	{
		/* CC1120 Interrupt. */
		cc1120_rx_interrupt();
	}
	/* Otherwise unknown interrupt? */
}
