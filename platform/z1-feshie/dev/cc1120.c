/**
 * \file
 *         TI CC1120 driver.
 * \author
 *         Graeme Bragg <g.bragg@ecs.soton.ac.uk>
 *         Phil Basford <pjb@ecs.soton.ac.uk>
 *	ECS, University of Southampton
 */
 
#include "contiki.h"
#include "contiki-conf.h"

#include <watchdog.h>

#if CC1120LEDS
#include "dev/leds.h"
#endif

/* CC1120 headers. */
#include "cc1120.h"
#include "cc1120-arch.h"
#include "cc1120-config.h"


/* LEDs. */
#undef LEDS_ON
#undef LEDS_OFF
#if CC1120LEDS
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

/* Busy Wait for time-outable waiting. */
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)


/*Define default RSSI Offset if it is not defined. */
#ifndef CC1120_RSSI_OFFSET
#define CC1120_RSSI_OFFSET		0x9A
#endif

#define PACKET_PENDING			0x01
#define RX_PENDING				0x02


/* -------------------- Internal Function Definitions. -------------------- */
static void on(void);
static void off(void);
static void LOCK_SPI(void);
static void RELEASE_SPI(void);

/* ---------------------- CC1120 SPI Functions ----------------------------- */
static uint8_t cc1120_spi_write_addr(uint16_t addr, uint8_t burst, uint8_t rw);
static void cc1120_write_txfifo(uint8_t *payload, uint8_t payload_len);

/* ------------------- CC1120 State Set Functions -------------------------- */
static uint8_t cc1120_set_idle(void);
static uint8_t cc1120_set_rx(void);
static uint8_t cc1120_set_tx(void);
static uint8_t cc1120_flush_rx(void);
static uint8_t cc1120_flush_tx(void);


PROCESS(cc1120_process, "CC1120 driver");

/* -------------------- Radio Driver Structure ---------------------------- */
const struct radio_driver cc1120_driver = {
	cc1120_driver_init,
	cc1120_driver_prepare,
	cc1120_driver_transmit,
	cc1120_driver_send_packet,
	cc1120_driver_read_packet,
	cc1120_driver_channel_clear,
	cc1120_driver_receiving_packet,
	cc1120_driver_pending_packet,
	cc1120_driver_on,
	cc1120_driver_off,
};



/* ------------------- Internal variables -------------------------------- */
static volatile uint8_t current_channel, transmitting, radio_pending, radio_on, tx_error, txfirst, txlast = 0;
static volatile uint16_t bad_crc;
static uint8_t locked, lock_on, lock_off;



/* ------------------- Radio Driver Functions ---------------------------- */

int 
cc1120_driver_init(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** CC1120 Radio Driver: Init ****\n");
#endif
	uint8_t part = 0;
	bad_crc = 0;
	
	/* Initialise arch  - pins, spi, turn off cc2420 */
	cc1120_arch_init();
	
	/* Reset CC1120 */
	cc1120_arch_reset();
	
	/* Check CC1120 - we read the part number register as a test. */
	part = cc1120_spi_single_read(CC1120_ADDR_PARTNUMBER);
	switch(part)
	{
		case CC1120_PART_NUM_CC1120:
			printf("CC1120 Detected - Radio OK");
			break;
		case CC1120_PART_NUM_CC1121:
			printf("CC1121 Detected - Radio OK");
			break;
		case CC1120_PART_NUM_CC1125:
			printf("CC1125 Detected - Radio OK");
			break;
		case CC1120_PART_NUM_CC1175:
			printf("CC1175 Detected\n");
			printf("*** ERROR: CC1175 is a transmitter only. Replace radio with a supported type and reset. ***\n");
			while(1)	/* Spin ad infinitum as we cannot continue. */
			{
				watchdog_periodic();	/* Feed the dog to stop reboots. */
			}
			break; /* spurious but.... */
		default:	/* Not a supported chip or no chip present... */
			printf("*** ERROR: Unsupported radio connected or no radio present (Part Number %02x detected) ***\n", part);
			printf("*** Check radio and reset ***\n");
			while(1)	/* Spin ad infinitum as we cannot continue. */
			{
				watchdog_periodic();	/* Feed the dog to stop reboots. */
			}
			break;
	}
	
#if CC1120LEDS
	printf(" & using LEDs.");
#endif

	printf("\n"); 
	
	// TODO: Cover sync-word errata somewhere?
	
	/* Configure CC1120 */
	cc1120_register_config();
	cc1120_gpio_config();
	cc1120_misc_config();
	
	/* Set Channel */
	cc1120_set_channel(RF_CHANNEL);
	
    /* Set radio off */
	cc1120_driver_off();
	
	process_start(&cc1120_process, NULL);
	
#if CC1120DEBUG || DEBUG
	printf("\tCC1120 Initialised and OFF\n");
#endif
	return 1;
}

int
cc1120_driver_prepare(const void *payload, unsigned short len)
{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("**** Radio Driver: Prepare ****\n");
#endif

	if(len > CC1120_MAX_PAYLOAD)
	{
		/* Packet is too large - max packet size is 125 bytes. */
#if CC1120DEBUG || CC1120TXERDEBUG || DEBUG || CC1120TXDEBUG
		printf("!!! PREPARE ERROR: Packet too large. !!!\n");
#endif
		return RADIO_TX_ERR;
	}
	
	/* Make sure that the TX FIFO is empty as we only want a single packet in it. */
	cc1120_flush_tx();
	
	/* If radio is meant to be in RX, put it back in. */
	if(radio_on)
	{	
		cc1120_set_state(CC1120_STATE_RX);
	}
		
	/* Write to the FIFO. */
	cc1120_write_txfifo(payload, len);
	RIMESTATS_ADD(lltx);
	
	return RADIO_TX_OK;
}

int
cc1120_driver_transmit(unsigned short transmit_len)
{
	
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\n\n**** Radio Driver: Transmit ****\n");
#endif
	
	uint8_t txbytes, cur_state, marc_state;
	
	/* Check that the packet is not too large. */
	if(transmit_len > CC1120_MAX_PAYLOAD)
	{
		/* Packet is too large - max packet size is 125 bytes. */
#if CC1120DEBUG || CC1120TXERDEBUG || DEBUG || CC1120TXDEBUG
		printf("!!! TX ERROR: Packet too large. !!!\n");
#endif
		return RADIO_TX_ERR;
	}
	
	LOCK_SPI();
	transmitting = 1;
#if CC1120LEDS	
	leds_on(LEDS_GREEN);
#endif
	
	/* Disable CC1120 interrupt to prevent a spurious trigger. */
	if(radio_on)
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf("\tTX: Disabling RX Interrupt...\n");
#endif	
		cc1120_arch_interrupt_disable();		/* Disable CC1120 Interrupt. */
		ENERGEST_OFF(ENERGEST_TYPE_LISTEN);		/* Set Energest for RX. */
	}

	/* check that we have data in the FIFO */
	txbytes = cc1120_read_txbytes();
	if(txbytes == 0)
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf("\tRetransmit last packet.\n");
#endif	
		/* Retransmit last packet. */
		cc1120_set_state(CC1120_STATE_IDLE);
		
		/* These registers should only be written in IDLE. */
		cc1120_spi_single_write(CC1120_ADDR_TXFIRST, txfirst);
		cc1120_spi_single_write(CC1120_ADDR_TXLAST, txlast);
	
	}
	else
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf("\tStoring txfirst and txlast\n", transmit_len, txbytes);
#endif			
		/* Store TX Pointers. */
		txfirst =  cc1120_spi_single_read(CC1120_ADDR_TXFIRST);
		txlast = cc1120_spi_single_read(CC1120_ADDR_TXLAST);
	}


#if WITH_SEND_CCA
	/* If we use LBT... */
	rtimer_clock_t t0;
	
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\tTransmitting with LBT.\n");
#endif	
	
	/* Set RX if radio is not already in it. */
	if(cc1120_get_state() != CC1120_STATUS_RX)
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf("\tEnter RX.\n");
#endif			
		cc1120_set_state(CC1120_STATE_RX);
	}


#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\tWait for valid RSSI.");
#endif		
	/* Wait for RSSI to be valid. */
	while(!(cc1120_spi_single_read(CC1120_ADDR_RSSI0) & (CC1120_RSSI_VALID)))
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf(".");
#endif			
		watchdog_periodic();	/* Feed the dog to stop reboots. */
	}
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\n");
#endif	

#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\tTX: Enter TX\n");
#endif	
	t0 = RTIMER_NOW();
	cur_state = cc1120_get_state();
	marc_state = cc1120_arch_read_gpio0();
	cc1120_spi_cmd_strobe(CC1120_STROBE_STX);	/* Strobe TX. */
	
	
	/* Block till in TX. */ 
	/* If reach timeout, strobe IDLE and reset CCA to clear TX & flush FIFO. */
	while((cur_state != CC1120_STATUS_TX) || (marc_state == 1))
	{
		if(RTIMER_CLOCK_LT((t0 + RTIMER_SECOND), RTIMER_NOW()))
		{
			/* 1 second Timeout reached. */
			cc1120_set_state(CC1120_STATE_IDLE);
			
			// TODO: Do we need to reset the CCA mode?
			
			/* Set Energest and TX flag. */
			ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
			transmitting = 0;
			
			
#if CC1120LEDS		
			/* Turn off LED if it is being used. */
			leds_off(LEDS_GREEN);
#endif				
			
			transmitting = 0;
			RELEASE_SPI();	
					
			if(radio_on)
			{
				cc1120_set_state(CC1120_STATE_RX);
				cc1120_arch_interrupt_enable();			/* Enable CC120 Interrupt. */
				ENERGEST_ON(ENERGEST_TYPE_LISTEN);		/* Set Energest for RX. */
			}
			
#if CC1120DEBUG || CC1120TXERDEBUG || DEBUG || CC1120TXDEBUG
			printf("!!! TX ERROR: Collision before TX - Timeout reached. !!!\n");
#endif	
			RIMESTATS_ADD(contentiondrop);
			/* Return Collision. */
			return RADIO_TX_COLLISION;
		}
		watchdog_periodic();	/* Feed the dog to stop reboots. */
		cur_state = cc1120_get_state();
		marc_state = cc1120_arch_read_gpio0();
	}
	
#else /* WITH_SEND_CCA */

#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\tTransmitting without LBT.\n");
#endif	
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\tTX: Enter TX\n");
#endif

	/* Disable CC1120 interrupt to prevent a spurious trigger. */
	if(radio_on)
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf("\tTX: Disabling RX Interrupt...\n");
#endif	
		cc1120_arch_interrupt_disable();		/* Disable CC1120 Interrupt. */
	}

	/* Enter TX. */
	cur_state = cc1120_set_state(CC1120_STATE_TX);

	if(cur_state != CC1120_STATUS_TX)
	{
		/* We didn't TX... */
		transmitting = 0;
#if CC1120DEBUG || CC1120TXERDEBUG || DEBUG || CC1120TXDEBUG
		printf("!!! TX ERROR: did not enter TX. Current state = %02x !!!\n", cur_state);
#endif			
		RELEASE_SPI();
		
		if(radio_on)
		{
			cc1120_set_state(CC1120_STATE_RX);
			cc1120_arch_interrupt_enable();			/* Enable CC120 Interrupt. */
			ENERGEST_ON(ENERGEST_TYPE_LISTEN);		/* Set Energest for RX. */
		}
		
		return RADIO_TX_ERR;
	}
#endif /* WITH_SEND_CCA */	
	
	
	
	
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("\tTX: in TX.");
#endif	
	ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
	
	/* Block till TX is complete. */	
	marc_state = cc1120_arch_read_gpio0();
	while(marc_state == 0)
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf(".");
#endif					
		marc_state = cc1120_arch_read_gpio0();
		watchdog_periodic();	/* Feed the dog to stop reboots. */
	}	

#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf("\n");
#endif		
	
	ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	transmitting = 0;
		
#if CC1120LEDS		
	leds_off(LEDS_GREEN);
#endif		
	cur_state = cc1120_get_state();
	
	if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
	{
		/* TX FIFO Error. Need to clear it... */
#if CC1120DEBUG || CC1120TXERDEBUG || DEBUG || CC1120TXDEBUG
		printf("!!! TX ERROR: TXFIFO Error - flushing. !!!\n");
#endif	
		cc1120_flush_tx();
		tx_error = 1;
	}
	
	marc_state = cc1120_spi_single_read(CC1120_ADDR_MARCSTATE) & 0x1F;	
	if((marc_state == CC1120_MARC_STATE_MARC_STATE_TX_END) && (cur_state == CC1120_STATUS_TX))
	{
		cc1120_set_state(CC1120_STATE_IDLE);
		cur_state = cc1120_get_state();
	}
	
	if(cur_state == CC1120_STATUS_TX)
	{
		/* Should never get here, just here for security. */
		printf("!!!!! TX ERROR: Still in TX according to status byte. !!!\n");
		cc1120_set_state(CC1120_STATE_IDLE);
		cur_state = cc1120_get_state();
	}
		
	txbytes = cc1120_read_txbytes();
	if(txbytes != 0)
	{
		/* we have not transmitted what we wanted to. */
#if CC1120DEBUG || CC1120TXERDEBUG || DEBUG || CC1120TXDEBUG
		printf("!!! TX ERROR: have not transmitted everything: %d bytes left in TX FIFO !!!\n",  txbytes);
#endif			
		cc1120_flush_tx();
		tx_error = 1;
	}
	
	RELEASE_SPI();
	
	if(radio_on)
	{
		cc1120_set_state(CC1120_STATE_RX);
		cc1120_arch_interrupt_enable();
		ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	}
	
	if(tx_error)
	{	
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG || CC1120TXERDEBUG
		printf("\tTX NOT OK.\n");
#endif		
		tx_error = 0;
		return RADIO_TX_ERR;
	}
	else
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf("\tTX OK.\n");
#endif	
		return RADIO_TX_OK;
	}
	return RADIO_TX_OK;
}

int
cc1120_driver_send_packet(const void *payload, unsigned short payload_len)
{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
	printf("**** Radio Driver: Send ****\n");
#endif
	if(cc1120_driver_prepare(payload, payload_len) != RADIO_TX_OK)
	{
		return RADIO_TX_ERR;
	}
	
	return cc1120_driver_transmit(payload_len);
}

int
cc1120_driver_read_packet(void *buf, unsigned short buf_len)
{
#if CC1120DEBUG || CC1120RXDEBUG 
	printf("**** Radio Driver: Read ****\n");
#endif
	uint8_t length, status, rxbytes = 0;
	
	rxbytes = cc1120_read_rxbytes();
	
	if(rxbytes < CC1120_MIN_PAYLOAD)
	{
		/* not enough data. */
		cc1120_flush_rx();
		
		RIMESTATS_ADD(tooshort);
		
#if CC1120DEBUG || CC1120RXDEBUG || CC1120RXERDEBUG
	printf("\tERROR: Packet too short\n");
#endif
		return 0;
	}
	
	
	/* Read length byte. */
	length = cc1120_spi_single_read(CC1120_FIFO_ACCESS);
	if((length + 1) > rxbytes)
	{
		/* Packet too long, out of Sync? */
		cc1120_flush_rx();
		
		RIMESTATS_ADD(badsynch);
#if CC1120DEBUG || CC1120RXDEBUG || CC1120RXERDEBUG
		printf("\tERROR: not enough data in FIFO. Length = %d, rxbytes = %d\n", length, rxbytes);
#endif
		return 0;
	}
	
	if(length > CC1120_MAX_PAYLOAD)
	{
		/* Packet too long, out of Sync? */
		cc1120_flush_rx();
		
		RIMESTATS_ADD(badsynch);
#if CC1120DEBUG || CC1120RXDEBUG || CC1120RXERDEBUG
	printf("\tERROR: Packet longer than FIFO or bad sync\n");
#endif
		return 0;
	}
	
	if((length) > buf_len) 
	{
		/* Packet is too long. */
		cc1120_flush_rx();
		
		RIMESTATS_ADD(toolong);
#if CC1120DEBUG || CC1120RXDEBUG || CC1120RXERDEBUG
	printf("\tERROR: Packet too long for buffer\n");
#endif		
		return 0;
	}
	
	
	/* Read the packet. */
	//dint();
	LOCK_SPI();
	cc1120_arch_spi_enable();
	
	cc1120_arch_rxfifo_read(buf, length);	
	
	cc1120_arch_spi_disable();
	
	//eint();
	
	status = cc1120_spi_single_read(CC1120_ADDR_MODEM_STATUS1);
	if(status & CC1120_MODEM_STATUS1_RXFIFO_UNDERFLOW)
	{
		/* FIFO underflow */
		cc1120_flush_rx();
		RELEASE_SPI();
#if CC1120DEBUG || CC1120RXDEBUG || CC1120RXERDEBUG
		printf("\tERROR: RX FIFO underflow. Meant to have %d bytes\n", length);
#endif		
		return 0;		
	}
	
	/* Read RSSI. */
	packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc1120_spi_single_read(CC1120_FIFO_ACCESS));

	/* Read LQI. */
	packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, (cc1120_spi_single_read(CC1120_FIFO_ACCESS) & CC1120_LQI_MASK));
	RELEASE_SPI();
	
	
	RIMESTATS_ADD(llrx);
	
	
	
	status = cc1120_spi_single_read(CC1120_ADDR_MODEM_STATUS1);
	if(status & CC1120_MODEM_STATUS1_RXFIFO_OVERFLOW)
	{
		/* FIFO overflow */
		cc1120_flush_rx();	
#if CC1120DEBUG || CC1120RXDEBUG || CC1120RXERDEBUG
		printf("\tERROR: RX FIFO Overflow\n");
#endif			
	}

	if((radio_on))
	{
		cc1120_driver_on();
	}

#if CC1120DEBUG || CC1120RXDEBUG
	printf("\tRX OK - %d byte packet.\n", length);
#endif	
	/* Return read length. */
	return length;
}

int
cc1120_driver_channel_clear(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: CCA ****\n");
#endif

	uint8_t cur_state, cca;
	
	/* We need to be in RX to do a CCA so find out where we are. */
	cur_state = cc1120_get_state();
	
	/* If we are not in RX, get us there. */
	if(cur_state != CC1120_STATUS_RX)
	{
		cc1120_set_state(CC1120_STATE_RX);	
	}
	
	uint8_t rssi0 = cc1120_spi_single_read(CC1120_ADDR_RSSI0);
	
	/* Wait till the CARRIER_SENSE is valid. */
    while(!(rssi0 & CC1120_CARRIER_SENSE_VALID))
	{
		rssi0 = cc1120_spi_single_read(CC1120_ADDR_RSSI0);
	}
	
	if(rssi0 & CC1120_RSSI0_CARRIER_SENSE)
	{
		cca = 0;
#if CC1120DEBUG || DEBUG
		printf("\t Channel NOT clear.\n");
#endif
#if CC1120LEDS
		leds_off(LEDS_BLUE);
#endif		
	}
	else
	{
		cca = 1;
#if CC1120DEBUG || DEBUG
		printf("\t Channel clear.\n");
#endif
#if CC1120LEDS
		leds_on(LEDS_BLUE);
#endif		
	}

	
	if(!radio_on)
	{
		cc1120_driver_off();
	}
	
	return cca;
}

int
cc1120_driver_receiving_packet(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Receiving Packet? ");
#endif

	if(transmitting)
	{
		/* Can't be receiving in TX. */
#if CC1120DEBUG || DEBUG
		printf(" - NO, in TX. ****\n");
#endif
		return 0;
	}
	else if(!radio_on)
	{
#if CC1120DEBUG || DEBUG
		printf(" - NO, Radio OFF. ****\n");
#endif
		/* cannot be receiving with the radio off. */
		return 0;
	}
	else
	{
		if(cc1120_arch_read_gpio0())
		{
			/* Not receiving */
#if CC1120DEBUG || DEBUG
			printf(" - NO. ****\n");
#endif
			return 0;
		}
		else 
		{
			
#if CC1120DEBUG || DEBUG
			printf(" Yes. ****\n");
#endif
			return 1;
		}
	}
}

int
cc1120_driver_pending_packet(void)
{
	//return 1;
#if CC1120DEBUG || CC1120RXDEBUG || DEBUG
	printf("**** Radio Driver: Pending Packet? ");
#endif
	if((radio_pending & PACKET_PENDING) || (cc1120_read_rxbytes() > 0))
	{
#if CC1120DEBUG || CC1120RXDEBUG || DEBUG
		printf(" yes ****\n");
#endif
		return 1;		
	}
	else
	{
#if CC1120DEBUG || CC1120RXDEBUG || DEBUG
		printf(" no ****\n");
#endif
		return 0;
		
	}
}

int
cc1120_driver_on(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: On ****\n");
#endif
	/* Set CC1120 into RX. */
	// TODO: If we are in SLEEP before this, do we need to do a cal and reg restore?
	// TODO: DO we want to set TXOFF_MODE=11 so that it goes to RX after TX?
	// TODO: Do we want to set RXOFF_MODE=01 so that we go to FSTXON after RX? 
	// TODO: Do we want to flush RX before going into RX?
	
	
	if(radio_on)
	{
		return 1;
	}
	if(locked)
	{
		lock_on = 1;
		return 1;
	}
	
	//LOCK_SPI();
	on();
	//RELEASE_SPI();
	return 1;
}

int
cc1120_driver_off(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Off ****\n");
#endif
	
	if(radio_on != 1)
	{
		/* Radio is already off, do nothing. */
		return 1;
	}
	if(locked)
	{
		/* Radio is locked, indicate that we want to turn off. */
		lock_off = 1;
		return 1;
	}
	
	//LOCK_SPI();
	if(cc1120_get_state() == CC1120_STATUS_TX)
	{	
		/* Don't kill the radio if a TX is in progress. */
		lock_off = 1;
	}
	else
	{
		off();
	}
	//RELEASE_SPI();
	return 1;
}


/* --------------------------- CC1120 Support Functions --------------------------- */
void
cc1120_gpio_config(void)
{
	cc1120_spi_single_write(CC1120_ADDR_IOCFG0, CC1120_GPIO0_FUNC);
	
#ifdef CC1120_GPIO2_FUNC
	cc1120_spi_single_write(CC1120_ADDR_IOCFG2, CC1120_GPIO2_FUNC);
#endif
#ifdef CC1120_GPIO3_FUNC
	cc1120_spi_single_write(CC1120_ADDR_IOCFG3, CC1120_GPIO3_FUNC);
#endif
}

void
cc1120_misc_config(void)
{
	/* Set RX FIFO CRC Auto flush. */
	cc1120_spi_single_write(CC1120_ADDR_FIFO_CFG, 0x80);
	
	/* Set PKT_CFG1 - CRC Configured as 01 and status bytes appended, No data whitening, address check or byte swap.*/
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG1, 0x05);
	
	/* Set PKT_CFG1 for variable length packet. */
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG0, 0x20);
	
	/* Set RXEND to go back into RX after good packet and to never timeout. */
	cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG1, 0x3E);
	
	/* Set TXOFF to go to IDLE and to stay in RX on bad packet. */
	cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG0, 0x00);
	
	
	/* Set the RSSI Offset.  This is a two's compliment number and
	 * affects the value of anything involving the RSSI, including
	 * the Carrier Sense Threshold and RSSI 11:0. */
	cc1120_spi_single_write(CC1120_ADDR_AGC_GAIN_ADJUST, (CC1120_RSSI_OFFSET));   

	/* Set Carrier Sense Threshold. This is a two's compliment number. */
	cc1120_spi_single_write(CC1120_ADDR_AGC_CS_THR, (CC1120_CS_THRESHOLD));   

#if WITH_SEND_CCA	
	/* Configure Listen Before Talk (LBT), see Section 6.12 on Page 42 of the CC1120 userguide (swru295) for details. */
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG2, 0x10);
#else
	/* Let the MAC handle Channel Clear. CCA indication is given if below RSSI threshold and NOT receiving packet. */
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG2, 0x0C);
#endif	
}


uint8_t
cc1120_set_channel(uint8_t channel)
{
	uint32_t freq_registers;
	
	//dint();
	
	freq_registers = CC1120_CHANNEL_MULTIPLIER;
	freq_registers *= channel;
	freq_registers += CC1120_BASE_FREQ;
	
	
	cc1120_spi_single_write(CC1120_ADDR_FREQ0, ((unsigned char*)&freq_registers)[0]);
	cc1120_spi_single_write(CC1120_ADDR_FREQ1, ((unsigned char*)&freq_registers)[1]);
	cc1120_spi_single_write(CC1120_ADDR_FREQ2, ((unsigned char*)&freq_registers)[2]);
	
	printf("Frequency set to %02x %02x %02x (Requested %02x %02x %02x)\n", cc1120_spi_single_read(CC1120_ADDR_FREQ2), 
		cc1120_spi_single_read(CC1120_ADDR_FREQ1), cc1120_spi_single_read(CC1120_ADDR_FREQ0), ((unsigned char*)&freq_registers)[2], ((unsigned char*)&freq_registers)[1], ((unsigned char*)&freq_registers)[0]);
	
	
	/* If we are an affected part version, carry out calibration as per CC112x/CC1175 errata. 
	 * See http://www.ti.com/lit/er/swrz039b/swrz039b.pdf, page 3 for details.*/
	if(cc1120_spi_single_read(CC1120_ADDR_PARTVERSION) == 0x21)
	{
		uint8_t original_fs_cal2, calResults_for_vcdac_start_high[3], calResults_for_vcdac_start_mid[3];
		
		/* Set VCO cap Array to 0. */
		cc1120_spi_single_write(CC1120_ADDR_FS_VCO2, 0x00);				
		/* Read FS_CAL2 */
		original_fs_cal2 = cc1120_spi_single_read(CC1120_ADDR_FS_CAL2);
		/* Write FS_CAL2 as original_fs_cal2 +2 */
		cc1120_spi_single_write(CC1120_ADDR_FS_CAL2, (original_fs_cal2 + 2));
		/* Strobe CAL and wait for completion. */
		cc1120_set_state(CC1120_STATE_CAL);
		while(cc1120_get_state() == CC1120_STATUS_CALIBRATE);
		/* Read FS_VCO2, FS_VCO4, FS_CHP. */
		calResults_for_vcdac_start_high[0] = cc1120_spi_single_read(CC1120_ADDR_FS_VCO2);
		calResults_for_vcdac_start_high[1] = cc1120_spi_single_read(CC1120_ADDR_FS_VCO4);
		calResults_for_vcdac_start_high[2] = cc1120_spi_single_read(CC1120_ADDR_FS_CHP);
		/* Set VCO cap Array to 0. */
		cc1120_spi_single_write(CC1120_ADDR_FS_VCO2, 0x00);
		/* Write FS_CAL2 as original_fs_cal2 */
		cc1120_spi_single_write(CC1120_ADDR_FS_CAL2, original_fs_cal2);
		/* Strobe CAL and wait for completion. */
		cc1120_set_state(CC1120_STATE_CAL);
		while(cc1120_get_state() == CC1120_STATUS_CALIBRATE);
		/* Read FS_VCO2, FS_VCO4, FS_CHP. */
		calResults_for_vcdac_start_mid[0] = cc1120_spi_single_read(CC1120_ADDR_FS_VCO2);
		calResults_for_vcdac_start_mid[1] = cc1120_spi_single_read(CC1120_ADDR_FS_VCO4);
		calResults_for_vcdac_start_mid[2] = cc1120_spi_single_read(CC1120_ADDR_FS_CHP);
		
		if(calResults_for_vcdac_start_high[0] > calResults_for_vcdac_start_mid[0])
		{
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO2, calResults_for_vcdac_start_high[0]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_high[1]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_high[2]);
		}
		else
		{
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO2, calResults_for_vcdac_start_mid[0]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_mid[1]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_mid[2]);
		}
	}
	else
	{
		/* Strobe CAL and wait for completion. */
		cc1120_set_state(CC1120_STATE_CAL);
		while(cc1120_get_state() == CC1120_STATUS_CALIBRATE);
	}
	
	//eint();
	
	current_channel = channel;
	return current_channel;
}

uint8_t
cc1120_get_channel(void)
{
	return current_channel;
}

uint8_t
cc1120_read_txbytes(void)
{
	return cc1120_spi_single_read(CC1120_ADDR_NUM_TXBYTES);
}

uint8_t
cc1120_read_rxbytes(void)
{
	return cc1120_spi_single_read(CC1120_ADDR_NUM_RXBYTES);
}





/* -------------------------- CC1120 Internal Functions --------------------------- */
static void
on(void)
{
	/* Enable CC1120 RX interrupt*/
	cc1120_arch_interrupt_enable();
	
	/* Put radio into RX. */
	cc1120_set_state(CC1120_STATE_RX);
	// TODO: Do we want to wait till radio osc is stable?
	radio_on = 1;

	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
}

static void
off(void)
{
	/* Disable CC1120 RX interrupt. */
	cc1120_arch_interrupt_disable();
	
	/* Wait for any current TX to end */
	BUSYWAIT_UNTIL((cc1120_get_state() != CC1120_STATUS_TX), RTIMER_SECOND/10);
	
	/* Set state to IDLE.  This will flush the RX FIFO if there is an error. */
	cc1120_set_state(CC1120_STATE_IDLE);
	radio_on = 0;
	
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	
	/* Put radio into the off state defined in platform-conf.h. */
	cc1120_set_state(CC1120_OFF_STATE);
}

static void
LOCK_SPI(void)
{
	locked++;
}

static void 
RELEASE_SPI(void)
{
	if(locked == 1) 
	{
		if(lock_on) 
		{
			on();
			lock_on = 0;
		}
		if(lock_off) 
		{
			off();
			lock_off = 0;
		}
	}
	locked--;
}


/* ---------------------------- CC1120 State Functions ---------------------------- */
uint8_t
cc1120_set_state(uint8_t state)
{
	/* Get the current state. */
	uint8_t cur_state = cc1120_get_state();
	
	switch(state)
	{
		case CC1120_STATE_FSTXON:	/* Can only enter from IDLE, TX or RX. */
#if CC1120STATEDEBUG
								printf("\t\tEntering FSTXON (%02x) 683\n", state);
								printf("\t\tCurrent State = %02x 684\n", cur_state);
#endif								
								if(!((cur_state == CC1120_STATUS_IDLE) 
									|| (cur_state == CC1120_STATUS_TX)
									|| (cur_state == CC1120_STATUS_FSTXON)))
								{
									/* If we are not in IDLE or TX or FSTXON, get us to IDLE.
									 * While we can enter FSTXON from RX, it may leave stuff stuck in the FIFO. */
									cc1120_set_idle();
									if (cur_state == CC1120_STATUS_RX)
									{
										cc1120_flush_rx();
									}
								}
								if(!(cur_state == CC1120_STATUS_FSTXON))
								{
									cc1120_spi_cmd_strobe(CC1120_STROBE_SFSTXON);	/* Intentional Error to catch warnings. */
									while(cc1120_get_state() != CC1120_STATUS_FSTXON);
								}
								return CC1120_STATUS_FSTXON;
								
		case CC1120_STATE_XOFF:		/* Can only enter from IDLE. */
#if CC1120STATEDEBUG
								printf("\t\tEntering XOFF (%02x) 707\n", state);
								printf("\t\tCurrent State = %02x 708\n", cur_state);
#endif								
								/* If we are not in IDLE, get us there. */
								if(cur_state != CC1120_STATUS_IDLE)
								{
									cc1120_set_idle();
									if (cur_state == CC1120_STATUS_RX)
									{
										cc1120_flush_rx();
									}
								}
								cc1120_spi_cmd_strobe(CC1120_STROBE_SXOFF);
								return CC1120_STATUS_IDLE;
								
		case CC1120_STATE_CAL:		/* Can only enter from IDLE. */
#if CC1120STATEDEBUG
								printf("\t\tEntering CAL (%02x) 724\n", state);
								printf("\t\tCurrent State = %02x 725\n", cur_state);
#endif
								/* If we are not in IDLE, get us there. */
								if(cur_state != CC1120_STATUS_IDLE)
								{
									cc1120_set_idle();
									if (cur_state == CC1120_STATUS_RX)
									{
										cc1120_flush_rx();
									}
								}
								cc1120_spi_cmd_strobe(CC1120_STROBE_SCAL);
								while(cc1120_get_state() != CC1120_STATUS_CALIBRATE);
								return CC1120_STATUS_CALIBRATE;
								
		case CC1120_STATE_RX:		/* Can only enter from IDLE, FSTXON or TX. */
#if CC1120STATEDEBUG
								printf("\t\tEntering RX (%02x) 742\n", state);
								printf("\t\tCurrent State = %02x 743\n", cur_state);
#endif
								if (cur_state == CC1120_STATUS_RX)
								{
									cc1120_set_idle();
									cc1120_flush_rx();
									return cc1120_set_rx();
								}								
								else if((cur_state == CC1120_STATUS_IDLE) 
									|| (cur_state == CC1120_STATUS_FSTXON)
									|| (cur_state == CC1120_STATUS_TX))
								{
									/* Return RX state. */
									return cc1120_set_rx();
								}
								else if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
								{
									/* If there is a RX FIFO Error, clear it and RX. */
									cc1120_flush_rx();
									return cc1120_set_rx();
								}
								else if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
								{
									/* If there is a TX FIFO Error, clear it and RX. */
									cc1120_flush_tx();
									return cc1120_set_rx();
								}
								else
								{
									/* We are in a state that will end up in IDLE, FSTXON or TX. Wait till we are there. */
									while(!((cur_state == CC1120_STATUS_IDLE)
										|| (cur_state == CC1120_STATUS_FSTXON)
										|| (cur_state == CC1120_STATUS_TX)) )
										{
											cur_state = cc1120_get_state();
										}
									
									/* Return RX state. */
									return cc1120_set_rx();
								}
								break;
								
		case CC1120_STATE_TX:		/* Can only enter from IDLE, FSTXON or RX. */
#if CC1120STATEDEBUG
								printf("\t\tEntering TX (%02x) 787\n", state);
								printf("\t\tCurrent State = %02x 788\n", cur_state);
#endif
								if((cur_state == CC1120_STATUS_IDLE) 
								|| (cur_state == CC1120_STATUS_FSTXON)
								|| (cur_state == CC1120_STATUS_RX))
								{
									/* Return TX state. */
									// TODO: do we want to set PKT_CFG2.CCA_MODE = 100b (Listen Before Talk) so that TX is entered if in RX and channel is ! clear?
#if CC1120STATEDEBUG
									printf("\t\tSet TX 797\n");
#endif										
									return cc1120_set_tx();
								}
								else if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
								{
									/* If there is a RX FIFO Error, clear it and TX. */
#if CC1120STATEDEBUG
								printf("\t\tFlush RX FIFO 805\n");
#endif										
									cc1120_flush_rx();
#if CC1120STATEDEBUG
								printf("\t\tSet TX 809\n");
#endif										
									return cc1120_set_tx();
								}
								else if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
								{
									/* If there is a TX FIFO Error, clear it and TX. */
#if CC1120STATEDEBUG
								printf("\t\tFlush TX 817\n");
#endif										
									cc1120_flush_tx();
#if CC1120STATEDEBUG
								printf("\t\tSet TX 821\n");
#endif	
									return cc1120_set_tx();
								}
								else
								{
									/* We are in a state that will end up in IDLE, FSTXON or RX. Wait till we are there. */
									while(!((cur_state == CC1120_STATUS_IDLE)
										|| (cur_state == CC1120_STATUS_FSTXON)
										|| (cur_state == CC1120_STATUS_RX)) )
										{
											cur_state = cc1120_get_state();
										}
#if CC1120STATEDEBUG
								printf("\t\tIn TX 835\n");
#endif	
									/* Return TX state. */
									return cc1120_set_tx();
								}
								break;
								
		case CC1120_STATE_IDLE:		/* Can enter from any state. */
#if CC1120STATEDEBUG
								printf("\t\tEntering IDLE (%02x) 844\n", state);
								printf("\t\tCurrent State = %02x 845\n", cur_state);
#endif
								/* If we are already in IDLE, do nothing and return the current state. */
								if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
								{
#if CC1120STATEDEBUG
								printf("\t\tFlush RX FIFO 851\n");
#endif	
									/* If there is a RX FIFO Error, clear it. */
									cc1120_flush_rx();
								}
								else if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
								{
#if CC1120STATEDEBUG
								printf("\t\tFlush TX FIFO 859\n");
#endif	
									/* If there is a TX FIFO Error, clear it. */
									cc1120_flush_tx();
								}
								else if(cur_state != CC1120_STATUS_IDLE)
								{
#if CC1120STATEDEBUG
								printf("\t\tSet IDLE 867\n");
#endif										
									/* Set Idle. */
									cc1120_set_idle();
								}
#if CC1120STATEDEBUG
								printf("\t\tIn IDLE 873\n");
#endif	
								/* Return IDLE state. */
								return CC1120_STATUS_IDLE;
								
		case CC1120_STATE_SLEEP:	/* Can only enter from IDLE. */
#if CC1120STATEDEBUG
								printf("\t\tEntering SLEEP (%02x) 880\n", state);
								printf("\t\tCurrent State = %02x 881\n", cur_state);
#endif
								/* If we are not in IDLE, get us there. */
								if(cur_state != CC1120_STATUS_IDLE)
								{
#if CC1120STATEDEBUG
								printf("\t\tSet IDLE 887\n");
#endif									
									cc1120_set_idle();
								}
								cc1120_spi_cmd_strobe(CC1120_STROBE_SPWD);
#if CC1120STATEDEBUG
								printf("\t\tIn SLEEP 893\n");
#endif	
								return CC1120_STATUS_IDLE;
								break;
								
		default:				printf("!!! INVALID STATE REQUESTED !!! 898\n"); 
								return CC1120_STATUS_STATE_MASK;
								break;	
	}
}

uint8_t
cc1120_get_state(void)
{
	return (cc1120_spi_cmd_strobe(CC1120_STROBE_SNOP) & CC1120_STATUS_STATE_MASK);
}


/* -------------------------- CC1120 State Set Functions -------------------------- */
uint8_t
cc1120_set_idle(void)
{
	uint8_t cur_state = cc1120_get_state();
	///* Send IDLE strobe. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);

	/* Spin until we are in IDLE. */
	while(cur_state != CC1120_STATUS_IDLE)
	{
		cur_state = cc1120_get_state();
#if CC1120STATEDEBUG
			printf("\t\t\tCurrent state = %02x. 924\n", cur_state);
#endif	

		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
		{
#if CC1120STATEDEBUG
			printf("\t\t\tTX FIFO Error, FLushing TX. 929\n");
#endif	
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);
		}
		else if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
		{
#if CC1120STATEDEBUG
			printf("\t\t\tRX FIFO Error. Flushing RX 936\n");
#endif				
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);
		}
		else if (cur_state != CC1120_STATUS_IDLE)
		{
#if CC1120STATEDEBUG
			printf("\t\t\tNot in IDLE, re-strobing. 943\n");
#endif				
			/* Send IDLE strobe. */
			cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);
		}
		
		clock_delay(10);
	}
	// TODO: give this a timeout?


	/* Return IDLE state. */
	return CC1120_STATUS_IDLE;
}

uint8_t
cc1120_set_rx(void)
{
	/* Enter RX. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_SRX);

	/* Spin until we are in RX. */
	BUSYWAIT_UNTIL((cc1120_get_state() == CC1120_STATUS_RX), RTIMER_SECOND/10);
	
	/* Return RX state. */
	return cc1120_get_state();
}

uint8_t
cc1120_set_tx(void)
{
	uint8_t cur_state, marc_state;

	/* Enter TX. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_STX);
	
	cur_state = cc1120_get_state();
	marc_state = cc1120_arch_read_gpio0();
	
	/* If we are NOT in TX, Spin until we are in TX. */
	if(cur_state != CC1120_STATUS_TX)
	{
		while((cur_state != CC1120_STATUS_TX) || (marc_state == 1))
		{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
			printf(",");
#endif				
			cur_state = cc1120_get_state();
			marc_state = cc1120_arch_read_gpio0();
			if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
			{	
				/* TX FIFO Error - flush TX. */	
				return cc1120_flush_tx();
			}
			clock_delay(1);
		}		
	}
	
	// TODO: give this a timeout?

	/* Return TX state. */
	return CC1120_STATUS_TX;
}

uint8_t
cc1120_flush_rx(void)
{
	/* FLush RX FIFO. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);

	/* Spin until we are in IDLE. */
	BUSYWAIT_UNTIL((cc1120_get_state() == CC1120_STATUS_IDLE), RTIMER_SECOND/10);

	/* Return IDLE state. */
	return CC1120_STATUS_IDLE;
}

uint8_t
cc1120_flush_tx(void)
{
	uint8_t cur_state = cc1120_get_state();
	rtimer_clock_t t0 = RTIMER_NOW();

	if(cur_state == CC1120_STATUS_IDLE || cur_state == CC1120_STATUS_TX_FIFO_ERROR)
	{
		/* FLush TX FIFO. */
		cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);
	}
	
	/* Spin until we have flushed TX and are in IDLE. */
	while((cur_state != CC1120_STATUS_IDLE) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 10)))
	{
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
		{
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);
		}
		else if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
		{
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);
		}
		else if (cur_state != CC1120_STATUS_IDLE)
		{
				/* NotSend IDLE strobe. */
			cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);
		}
		clock_delay_usec(1);
		
		cur_state = cc1120_get_state();
	}

	/* Return last state. */
	return cur_state;
}



/* ----------------------------- CC1120 SPI Functions ----------------------------- */
uint8_t
cc1120_spi_cmd_strobe(uint8_t strobe)
{
	cc1120_arch_spi_enable();
	
	strobe = cc1120_arch_spi_rw_byte(strobe);
	
	cc1120_arch_spi_disable();
	
	return strobe;
}

uint8_t
cc1120_spi_single_read(uint16_t addr)
{
	cc1120_arch_spi_enable();
	
	cc1120_spi_write_addr(addr, CC1120_STANDARD_BIT, CC1120_READ_BIT);
	addr = cc1120_arch_spi_rw_byte(0);		/* Get the value.  Re-use addr to save a byte. */ 
	
	cc1120_arch_spi_disable();
	
	return addr;
}

uint8_t
cc1120_spi_single_write(uint16_t addr, uint8_t val)
{
	cc1120_arch_spi_enable();
	
	addr = cc1120_spi_write_addr(addr, CC1120_STANDARD_BIT, CC1120_WRITE_BIT);	/* Read the status byte. */
	cc1120_arch_spi_rw_byte(val);		
	
	cc1120_arch_spi_disable();
	
	return addr;
}

static uint8_t
cc1120_spi_write_addr(uint16_t addr, uint8_t burst, uint8_t rw)
{
	uint8_t status;
	if(addr & CC1120_EXTENDED_MEMORY_ACCESS_MASK)
	{
		status = cc1120_arch_spi_rw_byte(CC1120_ADDR_EXTENDED_MEMORY_ACCESS | rw | burst); 
		(void) cc1120_arch_spi_rw_byte(addr & CC1120_ADDRESS_MASK);
	}
	else
	{
		status = cc1120_arch_spi_rw_byte(addr | rw | burst);
	}
	
	return status;
}

static void
cc1120_write_txfifo(uint8_t *payload, uint8_t payload_len)
{
	cc1120_arch_spi_enable();
	
	cc1120_arch_txfifo_load(payload, payload_len);
	
	cc1120_arch_spi_disable();
	
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG	
	uint8_t fifo_len = cc1120_read_txbytes();
	printf("\t%d bytes in fifo (%d + length byte requested)\n", fifo_len, payload_len);
#endif
}



/* -------------------------- CC1120 Interrupt Handler --------------------------- */
int
cc1120_rx_interrupt(void)
{
#if CC1120LEDS	
	leds_on(LEDS_RED);
#endif	
	/* Acknowledge the interrupt. */
	cc1120_arch_interrupt_acknowledge();
	
	cc1120_set_state(CC1120_STATE_IDLE);
	
	/* Mark packet pending. */
	radio_pending |= PACKET_PENDING;
		
#if CC1120DEBUG || CC1120INTDEBUG || DEBUG
	printf("!!!!!!!!!!!!!!!!!!!!!! RX INTERRUPT !!!!!!!!!!!!!!!!!!!!!! \n");
#endif

	/* Poll radio process. */
	//process_poll(&cc1120_process);
	
	int len;
	packetbuf_clear();
	len = cc1120_driver_read_packet(packetbuf_dataptr(), PACKETBUF_SIZE);
	
#if C1120PROCESSDEBUG		
	printf("\t Packet Length: %d\n", len);
#endif	
	packetbuf_set_datalen(len);

	NETSTACK_RDC.input();
	
	cc1120_flush_rx();
	
	radio_pending &= ~PACKET_PENDING;

#if CC1120LEDS		
	leds_off(LEDS_RED);
#endif
	
	if(radio_on & (cc1120_get_state() != CC1120_STATUS_RX))
	{
#if CC1120DEBUG || CC1120INTDEBUG || DEBUG
		printf("Re-start RX\n");
#endif		
		cc1120_set_state(CC1120_STATE_RX);
	}
	
	return 1;
}





/* ----------------------------------- CC1120 Process ------------------------------------ */

PROCESS_THREAD(cc1120_process, ev, data)
{
	int len;
	PROCESS_BEGIN();

	printf("cc1120_process: started\n");

	while(1) 
	{
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		
#if C1120PROCESSDEBUG		
		printf("**** Process Poll ****\n");
#endif		
			
		packetbuf_clear();
		len = cc1120_driver_read_packet(packetbuf_dataptr(), PACKETBUF_SIZE);
		
#if C1120PROCESSDEBUG		
		printf("\t Packet Length: %d\n", len);
#endif	
		packetbuf_set_datalen(len);

		NETSTACK_RDC.input();
			
		radio_pending &= ~PACKET_PENDING;

#if CC1120LEDS		
		leds_off(LEDS_RED);
#endif
	}
		

		

	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_EXIT);

	PROCESS_END();
}

