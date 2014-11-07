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

#include "net/rime.h"
#include "net/rime/rimeaddr.h"
#include "net/netstack.h"
#include "net/mac/contikimac.h"


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

/* Printf definitions for debug */
#if CC1120DEBUG || DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#if CC1120DEBUG || CC1120RXDEBUG || DEBUG
#define PRINTFRX(...) printf(__VA_ARGS__)
#else
#define PRINTFRX(...) do {} while (0)
#endif

#if CC1120DEBUG || CC1120RXDEBUG || DEBUG
#define PRINTFRXERR(...) printf(__VA_ARGS__)
#else
#define PRINTFRXERR(...) do {} while (0)
#endif

#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
#define PRINTFTX(...) printf(__VA_ARGS__)
#else
#define PRINTFTX(...) do {} while (0)
#endif

#if CC1120DEBUG || CC1120TXERDEBUG || DEBUG || CC1120TXDEBUG
#define PRINTFTXERR(...) printf(__VA_ARGS__)
#else
#define PRINTFTXERR(...) do {} while (0)
#endif

#if CC1120DEBUG || CC1120INTDEBUG || DEBUG
#define PRINTFINT(...) printf(__VA_ARGS__)
#else
#define PRINTFINT(...) do {} while (0)
#endif

#if CC1120DEBUG || CC1120INTDEBUG || CC1120RXERDEBUG || CC1120RXDEBUG || DEBUG
#define PRINTFINTRX(...) printf(__VA_ARGS__)
#else
#define PRINTFINTRX(...) do {} while (0)
#endif

#if C1120PROCESSDEBUG		
#define PRINTFPROC(...) printf(__VA_ARGS__)
#else
#define PRINTFPROC(...) do {} while (0)
#endif

#if CC1120STATEDEBUG
#define PRINTFSTATE(...) printf(__VA_ARGS__)
#else
#define PRINTFSTATE(...) do {} while (0)
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

#define ACK_PENDING				0x01			
#define RX_FIFO_OVER			0x02
#define RX_FIFO_UNDER			0x04
#define TX_FIFO_ERROR			0x08
#define TX_COMPLETE				0x10
#define TX_ERROR				0x20
#define TRANSMITTING			0x40
#define ON						0x80

#define ACK_LEN 3
#define ACK_FRAME_CONTROL_LSO	0x02
#define ACK_FRAME_CONTROL_MSO	0x00

#define CC1120_802154_FCF_ACK_REQ			0x20
#define CC1120_802154_FCF_DEST_ADDR_16BIT	0x08
#define CC1120_802154_FCF_DEST_ADDR_64BIT	0x0C


/* -------------------- Internal Function Definitions. -------------------- */
static void on(void);
static void off(void);
static void processor(void);

/* ---------------------- CC1120 SPI Functions ----------------------------- */
static uint8_t cc1120_spi_write_addr(uint16_t addr, uint8_t burst, uint8_t rw);
static void cc1120_write_txfifo(uint8_t *payload, uint8_t payload_len);

/* ------------------- CC1120 State Set Functions -------------------------- */
static uint8_t cc1120_set_idle(uint8_t cur_state);
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
static uint8_t ack_tx, current_channel, packet_pending, broadcast, ack_seq, tx_seq, 
				rx_rssi, rx_lqi, lbt_success, radio_pending, radio_on, txfirst, txlast = 0;
static uint8_t locked, lock_on, lock_off;

static uint8_t ack_buf[ACK_LEN];
static uint8_t tx_buf[CC1120_MAX_PAYLOAD];

static uint8_t tx_len;

/* ------------------- Radio Driver Functions ---------------------------- */

int 
cc1120_driver_init(void)
{
	PRINTF("**** CC1120 Radio  Driver: Init ****\n");
	uint8_t part;
	
	cc1120_arch_init(); /* Initialise arch  - pins, spi, turn off cc2420 */
	cc1120_arch_reset(); /* Reset CC1120 */
	
	part = cc1120_spi_single_read(CC1120_ADDR_PARTNUMBER);		/* Check CC1120 - we read the part number register as a test. */
	if(part == CC1120_PART_NUM_CC1120) {						/* Cascading IFs here save 10 bytes of FLASH over a switch case. */
		printf("CC1120\n");
	} else if(part == CC1120_PART_NUM_CC1121) {
		printf("CC1121\n");
	} else if(part == CC1120_PART_NUM_CC1125) {
		printf("CC1125\n");
	} else if(part == CC1120_PART_NUM_CC1200) {
		printf("CC1200\n");
	} else if(part == CC1120_PART_NUM_CC1201) {
		printf("CC1201\n");
	} else {
		printf("*** ERROR: No Radio ***\n");
		while(1) {					/* Spin ad infinitum as we cannot continue. */
			watchdog_periodic();	/* Feed the dog to stop reboots. */
		}
	}
	
	// TODO: Cover sync-word errata somewhere?
	
	/* Configure CC1120 */
	cc1120_register_config();											/* Configure the CC1120 to the supplied config. */

	cc1120_spi_single_write(CC1120_ADDR_IOCFG0, CC1120_GPIO0_FUNC);		/* Set GPIO0 function.  */
	cc1120_spi_single_write(CC1120_ADDR_IOCFG3, CC1120_GPIO3_FUNC);		/* Set GPIO3 function. */
#ifdef CC1120_GPIO2_FUNC
	cc1120_spi_single_write(CC1120_ADDR_IOCFG2, CC1120_GPIO2_FUNC);		/* Set GPIO2 function if it is configured. */
#endif

	cc1120_spi_single_write(CC1120_ADDR_FIFO_CFG, 0x80);				/* Set RX FIFO CRC Auto flush. */
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG1, 0x05);				/* Set PKT_CFG1 - CRC Configured as 01 and status bytes appended, No data whitening, address check or byte swap.*/
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG0, 0x20);				/* Set PKT_CFG1 for variable length packet. */
	cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG1, 0x0F);				/* Set RXEND to go into IDLE after good packet and to never timeout. */
	cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG0, 0x00);				/* Set TXOFF to go to IDLE and to stay in RX on bad packet. */
	cc1120_spi_single_write(CC1120_ADDR_AGC_GAIN_ADJUST, (CC1120_RSSI_OFFSET));	/* Set the RSSI Offset. This is a two's compliment number. */
	cc1120_spi_single_write(CC1120_ADDR_AGC_CS_THR, (CC1120_CS_THRESHOLD));   	/* Set Carrier Sense Threshold. This is a two's compliment number. */
	
	//cc1120_spi_single_write(CC1120_ADDR_SYNC_CFG0, 0x0B);       		/* Set the correct sync word length.  SmartRF sets 32-bits instead of 16-bits for 802.15.4G. */

#if RDC_CONF_HARDWARE_CSMA	
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG2, 0x10);				/* Configure Listen Before Talk (LBT), see Section 6.12 on Page 42 of the CC1120 userguide (swru295) for details. */
#else
	cc1120_spi_single_write(CC1120_ADDR_PKT_CFG2, 0x0C);				/* Let the MAC handle Channel Clear. CCA indication is given if below RSSI threshold and NOT receiving packet. */
#endif	
	
	cc1120_set_channel(RF_CHANNEL); 									/* Set Channel */                           
	cc1120_driver_off();												/* Set radio off */
	process_start(&cc1120_process, NULL);								/* Start the CC1120 process. */
	cc1120_arch_interrupt_enable();										/* Enable CC1120 interrupt. */
	
	PRINTF(" Init\n");
	return 1;
}

int
cc1120_driver_prepare(const void *payload, unsigned short len)
{
	PRINTFTX("**** Radio Driver: Prepare ****\n");

	if(len > CC1120_MAX_PAYLOAD) {
		PRINTFTXERR("!!! PREPARE ERROR: Packet too large. !!!\n");
		return RADIO_TX_ERR;						/* Packet is too large - max packet size is 125 bytes. */
	}
	
	CC1120_LOCK_SPI();								/* Lock SPI to prevent conflicts. */
	cc1120_flush_tx();								/* Flush the FIFO. */
	cc1120_write_txfifo((uint8_t *)payload, len);	/* Write to the FIFO. */
	ack_tx  = 0;									/* Fifo does not contain an ACK. */
	memcpy(tx_buf, payload, len);					/* Keep a local copy of the FIFO. */
	tx_len = len;									/* Keep a local copy of the length. */
	tx_seq = ((uint8_t *)payload)[2];   			/* Keep a local copy of the Seq No. */
	lbt_success = 0;								/* LBT has not succeeded. */
	RIMESTATS_ADD(lltx);
	
	if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) {
		broadcast = 1;								/* Mark the packet in TXFIFO as a broadcast packet. */
		PRINTFTX("\tBroadcast\n");
	} else {
		broadcast = 0;								/* Packet in TXFIFO is not a broadcast packet. */
		PRINTFTX("\tUnicast, Seqno = %d\n", tx_seq);
	}
	
	if(radio_on) {					// & !cc1120_driver_pending_packet()
		cc1120_set_state(CC1120_STATE_RX);			/* If radio is meant to be in RX, put it back in. */
	}
	CC1120_RELEASE_SPI();
	return RADIO_TX_OK;
}

int
cc1120_driver_transmit(unsigned short transmit_len)
{
	PRINTFTX("\n\n**** Radio Driver: Transmit ****\n");
	uint8_t txbytes, cur_state, marc_state;
	rtimer_clock_t t0;
	
	if(transmit_len > CC1120_MAX_PAYLOAD) {
		PRINTFTX("!!! TX ERROR: Packet too large. !!!\n");		/* Packet is too large - max packet size is 125 bytes. */
		return RADIO_TX_ERR;
	}
	
	CC1120_LOCK_SPI();											/* Lock the SPI to prevent interference. */
	
	radio_pending |= TRANSMITTING;								/* Mark that we are transmitting. */
	radio_pending &= ~(TX_COMPLETE);							/* Mark TX incomplete. */
	LEDS_ON(LEDS_GREEN);										/* Turn on TX LED. */
	
	if(ack_tx == 1) {
		cc1120_flush_tx();										/* Wrong data in the TXFIFO. Flush it. */
		cc1120_write_txfifo(tx_buf, tx_len);					/* Re-write the TXFIFO. */
		ack_tx = 0;												/* Mark as no ACK sent. */
	}
	
	txbytes = cc1120_read_txbytes();							/* Get the number of bytes in the TXFIFO. */			
	if(txbytes == 0) {
		PRINTFTX("\tRetransmit last packet.\n");				/* check if this is a retransmission */
		cc1120_set_state(CC1120_STATE_IDLE);					/* Retransmit last packet. */
		cc1120_spi_single_write(CC1120_ADDR_TXFIRST, txfirst);	/* TXFIRST should only be written in IDLE. */
		cc1120_spi_single_write(CC1120_ADDR_TXLAST, txlast);	/* TXLAST should only be written in IDLE. */
		txbytes = cc1120_read_txbytes();						/* Read txbytes. */
	}
	if(txbytes != tx_len + 1) {
		cc1120_flush_tx();										/* Check if the TXFIFO contains the correct amount of data. */
		cc1120_write_txfifo(tx_buf, tx_len);
		ack_tx = 0;
	}

	txfirst =  cc1120_spi_single_read(CC1120_ADDR_TXFIRST);		/* Store TXFIRST Pointer. */
	txlast = cc1120_spi_single_read(CC1120_ADDR_TXLAST);		/* Store TXFIRST Pointer. */
	
#if RDC_CONF_HARDWARE_CSMA
	/* If we use LBT... */
	if(lbt_success == 0) {
		PRINTFTX("\tTransmitting with LBT.\n");			/* First TX, so use LBT. */
	
		cc1120_set_state(CC1120_STATE_RX);				/* Make sure we are in RX for LBT. */	
		while(!(cc1120_spi_single_read(CC1120_ADDR_RSSI0) & (CC1120_RSSI_VALID))) {
			PRINTFTX(".");								/* Wait for RSSI to be valid. */
			watchdog_periodic();						/* Feed the dog to stop reboots. */
		}
	} else {
		PRINTFTX("Retransmitting last  packet.\n");		/* Retransmitting last packet not using LBT. For ContikiMAC*/	
		cc1120_set_state(CC1120_STATE_IDLE);			/* Set IDLE. */
	}

	cc1120_spi_cmd_strobe(CC1120_STROBE_STX);			/* Strobe TX. */
	
	/* Block until in TX.  If timeout is reached, strobe IDLE and reset CCA to clear TX & flush FIFO. */
	cur_state = cc1120_get_state();						/* Get the current state. */
	t0 = RTIMER_NOW();									/* Get the current time. */
	while(!cc1120_arch_read_gpio3()) {
		watchdog_periodic();	/* Feed the dog to stop reboots. */
		
		if(RTIMER_CLOCK_LT((t0 + CC1120_LBT_TIMEOUT), RTIMER_NOW()) ) {
			cc1120_set_state(CC1120_STATE_IDLE);		/* Timeout reached, set IDLE */
			
			// TODO: Do we need to reset the CCA mode?
			ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);		
			radio_pending &= ~(TRANSMITTING);			/* Mark that we are not transmitting. */
			LEDS_OFF(LEDS_GREEN);						/* Turn off TX LED. */
			cc1120_flush_tx();							/* Flush the TX FIFO. */
			lbt_success = 0;							/* LBT not successful. */
			RIMESTATS_ADD(contentiondrop);				
			
			PRINTFTXERR("!!! TX ERROR: Collision before TX - Timeout reached. !!!\n");
			
			if(radio_on) {					// & !cc1120_driver_pending_packet()
				cc1120_set_state(CC1120_STATE_RX);		/* If radio is meant to be in RX, put it back in. */
			}
			CC1120_RELEASE_SPI();						/* Release the SPI Lock. */
			
			return RADIO_TX_COLLISION;					/* Return Collision. */
		} else if (radio_pending & TX_FIFO_ERROR) {
			radio_pending &= ~(TRANSMITTING);			/* TXFIFO error has been set by interrupt handler. */
			cc1120_flush_tx();							/* Flush the TX FIFO. */
			LEDS_OFF(LEDS_GREEN);						/* Turn off TX LED. */
			lbt_success = 0;							/* LBT not successful. */
			
			ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
			PRINTFTXERR("!!! TX ERROR: FIFO Error. !!!\n");	
			
			if(radio_on) {					// & !cc1120_driver_pending_packet()
				cc1120_set_state(CC1120_STATE_RX);		/* If radio is meant to be in RX, put it back in. */
			}
			CC1120_RELEASE_SPI();						/* Release the SPI Lock. */
			return RADIO_TX_ERR;
		}
		cur_state = cc1120_get_state();					/* Get the current state. */
	}
	lbt_success = 1;									/* LBT was successful. */
	
#else /* RDC_CONF_HARDWARE_CSMA */
	PRINTFTX("\tTransmitting without LBT.\n");

	cur_state = cc1120_set_state(CC1120_STATE_TX);		/* Enter TX. */
	
	if(cur_state != CC1120_STATUS_TX) {
		radio_pending &= ~(TRANSMITTING);				/* We didn't TX... */
		LEDS_OFF(LEDS_GREEN);							/* Turn off TX LED. */	
		cc1120_flush_tx();								/* Flush TX FIFO. */
		
		PRINTFTXERR("!!! TX ERR: Not in TX. S= %02x !!!\n", cur_state);
		
		if(radio_on) {					// & !cc1120_driver_pending_packet()
			cc1120_set_state(CC1120_STATE_RX);		/* If radio is meant to be in RX, put it back in. */
		}
		CC1120_RELEASE_SPI();	
		return RADIO_TX_ERR;
	}
#endif /* WITH_SEND_CCA */	
	
	PRINTFTX("\tTX: in TX.");
	ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
	t0 = RTIMER_NOW();
	
	while(cc1120_arch_read_gpio3()) {
		watchdog_periodic();	/* Wait for CC1120 to finish TX. */
		PRINTFTX(".");
		
		if(radio_pending & TX_FIFO_ERROR) {
			cc1120_flush_tx();		/* TX FIFO has underflowed during TX.  Need to flush TX FIFO. */
			break;
		}
		
		if(RTIMER_CLOCK_LT((t0 + RTIMER_SECOND/20), RTIMER_NOW())) {
			/* Timeout for TX. At 802.15.4 50kbps data rate, the entire 
			 * TX FIFO (all 128 bits) should be transmitted in 0.02 
			 * seconds. Timeout set to 0.05 seconds to be sure.  If 
			 * the interrupt has not fired by this time then something 
			 * went wrong. 
			 * 
			 * This timeout needs to be adjusted if lower data rates 
			 * are used. */	 
			if((cc1120_read_txbytes() == 0) && !(radio_pending & TX_FIFO_ERROR)) {
				radio_pending |= TX_COMPLETE;			/* We have actually transmitted everything in the FIFO. */
				PRINTFTXERR("!!! TX ERR: Timeout reached, packet sent !!!\n");
			} else {
				cc1120_set_state(CC1120_STATE_IDLE);
				PRINTFTXERR("!!! TX ERR: Timeout reached !!!\n");						
			}
			break;
		}
	}

	ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	t0 = RTIMER_NOW();								/* Get TX End time. */
	LEDS_OFF(LEDS_GREEN);							/* Turn off TX LED. */
	radio_pending &= ~(TRANSMITTING);				/* We are not transmitting. */
		
	if(cc1120_read_txbytes() > 0) {	
		PRINTFTXERR("\tStray bytes in TX FIFO: %d.\n", cc1120_read_txbytes());	
		cc1120_flush_tx();							/* Flush TX FIFO. */
		cc1120_flush_rx();							/* Flush RX FIFO. */
		radio_pending &= ~(TX_ERROR | ACK_PENDING);	/* Clear error & no ACK pending. */
		
		if(radio_on) {
			cc1120_set_state(CC1120_STATE_RX);		/* If radio is meant to be in RX, put it back in. */
		}
		CC1120_RELEASE_SPI();						/* Release the SPI lock. */
		return RADIO_TX_ERR;
	} else {
		PRINTFTX("\tTX OK.\n");
		
		radio_pending &= ~(TX_ERROR);				/* No error. */
		cur_state = cc1120_get_state();				/* Get current state. */
		marc_state = cc1120_spi_single_read(CC1120_ADDR_MARCSTATE) & 0x1F;	
		
		if(cur_state == CC1120_STATUS_TX) {
			if (marc_state != CC1120_MARC_STATE_MARC_STATE_TX_END) { 
				printf("!!!!! TX ERR: Stuck in TX. !!!\n");
			}
			cc1120_set_state(CC1120_STATE_IDLE);	/* Put radio into IDLE. */
		}
		
		cc1120_flush_rx();							/* Make sure RX FIFO is empty. */	
		cc1120_flush_tx();							/* Make sure TX FIFO is empty. */
		
		if(broadcast) {
			PRINTFTX("\tBroadcast TX OK\n");		/* Broadcast TX OK.  No ACK expected. */
			radio_pending &= ~(ACK_PENDING);		/* NOT expecting and ACK. */	
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CC1120_INTER_PACKET_INTERVAL)) { 
				watchdog_periodic();				/* Wait for inter-packet interval. */
			}
			
		} else {			
			cc1120_set_state(CC1120_STATE_RX);		/* Need to be in RX for the ACK. */
			radio_pending |= ACK_PENDING;			/* ACK is pending. */
			
			if(tx_seq == 0) {
				ack_buf[2] = 128;					/* Handle wrapping sequence numbers. */
			} else {
				ack_buf[2] = 0;
			}
			
			while((RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CC1120_INTER_PACKET_INTERVAL))) {
				watchdog_periodic();				/* Wait till timeout or ACK is received. */
			}
			
			if(cc1120_get_state() != CC1120_STATUS_RX || cc1120_read_rxbytes() > 0) {
				transmit_len = cc1120_spi_single_read(CC1120_FIFO_ACCESS);	/* We have received something. */
				
				if(transmit_len == 3) {		
					cc1120_arch_spi_enable();		/* It is an ACK, read it. */
					cc1120_arch_rxfifo_read(ack_buf, transmit_len);
					cc1120_arch_spi_disable();
					ack_seq = ack_buf[2];			/* Get the sequence number. */
				}	
			}
			
			radio_pending &= ~(ACK_PENDING);		/* No ACK pending. */
			cc1120_flush_rx();						/* Flush the RX FIFO. */
	
			if(ack_buf[2] != tx_seq) {
				PRINTFTX("\tNo ACK received.\n");
				if(radio_on) {
					cc1120_set_state(CC1120_STATE_RX);		/* If radio is meant to be in RX, put it back in. */
				}
				CC1120_RELEASE_SPI();						/* Release the SPI lock. */
				return RADIO_TX_NOACK;
			}
		}	
	}
	if(radio_on) {
		cc1120_set_state(CC1120_STATE_RX);		/* If radio is meant to be in RX, put it back in. */
	}
	CC1120_RELEASE_SPI();						/* Release the SPI lock. */
	return RADIO_TX_OK;
}

int
cc1120_driver_send_packet(const void *payload, unsigned short payload_len)
{
	PRINTFTX("**** Radio Driver: Send ****\n");
	if(cc1120_driver_prepare(payload, payload_len) != RADIO_TX_OK) {	/* Load the TX FIFO. */
		return RADIO_TX_ERR;											/* Could not load the FIFO... */
	}
	return cc1120_driver_transmit(payload_len);							/* TX the packet. */
}

int
cc1120_driver_read_packet(void *buf, unsigned short buf_len)
{
	PRINTF("**** Radio Driver: Read ****\n");
	uint8_t length, i, rxbytes;
	rimeaddr_t dest;
	rtimer_clock_t t0;   
		
	//if(radio_pending & RX_FIFO_UNDER) {							/* The RX FIFO has underflowed. */
	//	cc1120_flush_rx();												/* Flush RX FIFO. */
	//	PRINTFRXERR("\tRX ERR: FIFO underflow.\n");		
	//	return 0;		
	//}
	
	rxbytes = cc1120_read_rxbytes();									/* Read the number of bytes in the RX FIFO. */
	
	if(rxbytes < CC1120_MIN_PAYLOAD) {							/* Not enough data in the RX FIFO. */
		cc1120_flush_rx();												/* Flush RX FIFO. */
		RIMESTATS_ADD(tooshort);										
		PRINTFRXERR("\tRX ERR: Packet too short\n");
		return 0;
	}
	
	CC1120_LOCK_SPI();
	length = cc1120_spi_single_read(CC1120_FIFO_ACCESS);		/* Read length byte. */
	
	if(((length + 2) > rxbytes) || (length > CC1120_MAX_PAYLOAD)) {		/* Packet too long, out of Sync? */
		cc1120_flush_rx();												/* Flush RX FIFO. */
		RIMESTATS_ADD(badsynch);
		PRINTFRXERR("\tRX ERR: Sync error. Packet is too long. ");
		length = 0;
	} else if((length) > buf_len) {								/* Packet is too long. */
		cc1120_flush_rx();												/* Flush RX FIFO. */
		RIMESTATS_ADD(toolong);
		PRINTFRXERR("\tERROR: Packet too long for buffer\n");	
		length = 0;
	} else {	
		PRINTFRX("\tPacket received.\n"); 
		watchdog_periodic();												/* Feed the dog to stop reboots. */
		
		cc1120_arch_spi_enable();											/* Read the packet fronm the FIFO. */
		cc1120_arch_spi_rw_byte(CC1120_FIFO_ACCESS | CC1120_BURST_BIT | CC1120_READ_BIT);
		for(i = 0; i < length; i++) {
			((uint8_t *)buf)[i] = cc1120_arch_spi_rw_byte(0);				/* Read a byte. */
			if((i == 13) && (((uint8_t *)buf)[0] & CC1120_802154_FCF_ACK_REQ)) { /* Handle any required ACK once the 13th byte has been read. */ 
				if((((uint8_t *)buf)[1] & 0x0C) == 0x0C) {					/* Get the address in the correct order. */
					dest.u8[7] = ((uint8_t *)buf)[5];						/* Long address. */
					dest.u8[6] = ((uint8_t *)buf)[6];
					dest.u8[5] = ((uint8_t *)buf)[7];
					dest.u8[4] = ((uint8_t *)buf)[8];
					dest.u8[3] = ((uint8_t *)buf)[9];
					dest.u8[2] = ((uint8_t *)buf)[10];
					dest.u8[1] = ((uint8_t *)buf)[11];
					dest.u8[0] = ((uint8_t *)buf)[12];
				} else if((((uint8_t *)buf)[1] & 0x08) == 0x08) {
					dest.u8[1] = ((uint8_t *)buf)[5];						/* Short address. */
					dest.u8[0] = ((uint8_t *)buf)[6];
				}
				
				if(rimeaddr_cmp(&dest, &rimeaddr_node_addr)) {				/* Work out if we need to send an ACK. */
					PRINTFRX("\tSending ACK\n");
					
					cc1120_arch_spi_disable();                                                
					watchdog_periodic();									/* Feed the dog to stop reboots. */
					
					ack_buf[0] = ACK_FRAME_CONTROL_LSO;						/* Populate ACK Frame buffer. */
					ack_buf[1] = ACK_FRAME_CONTROL_MSO;
					ack_buf[2] = ((uint8_t *)buf)[2];
					
					radio_pending &= ~(TX_COMPLETE);						/* Mark that TX is pending. */
					cc1120_flush_tx();										/* Flush TX FIFO. */
					cc1120_write_txfifo(ack_buf, ACK_LEN);					/* Write ACK to TX FIFO. */
					ack_tx = 1;												/* Mark that last packet written to FIFO was an ACK. */
					cc1120_spi_cmd_strobe(CC1120_STROBE_STX);				/* Transmit ACK WITHOUT LBT. */
					t0 = RTIMER_NOW(); 
					while(!(radio_pending & TX_COMPLETE)) {					/* Block till TX is complete. */	
						if(radio_pending & TX_FIFO_ERROR) {
							cc1120_flush_tx();								/* TX FIFO has underflowed during ACK TX.  Need to flush TX FIFO. */
							break;
						}
						
						if(RTIMER_CLOCK_LT((t0 + RTIMER_SECOND/20), RTIMER_NOW())) {
							/* Timeout for TX. At 802.15.4 50kbps data rate, the entire 
							 * TX FIFO (all 128 bits) should be transmitted in 0.02 
							 * seconds. Timeout set to 0.05 seconds to be sure.  If 
							 * the interrupt has not fired by this time then something 
							 * went wrong. 
							 * 
							 * This timeout needs to be adjusted if lower data rates 
							 * are used. */	 
							if((cc1120_read_txbytes() == 0) && !(radio_pending & TX_FIFO_ERROR)) {
								radio_pending |= TX_COMPLETE;				/* We have actually transmitted everything in the FIFO. */
								PRINTFTXERR("!!! TX ERROR: TX timeout reached but packet sent !!!\n");
							} else {
								cc1120_set_state(CC1120_STATE_IDLE);
								PRINTFTXERR("!!! TX ERROR: TX timeout reached !!!\n");						
							}
							break;
						}
					}
					cc1120_flush_tx();										/* Make sure that the TX FIFO is empty. */
					cc1120_arch_spi_enable();								/* Re-enable burst access to read remaining data. */
					(void) cc1120_arch_spi_rw_byte(CC1120_FIFO_ACCESS | CC1120_BURST_BIT | CC1120_READ_BIT);
				}
			}
		}
		cc1120_arch_spi_disable();												/* Disable the CC1120. */
		rx_rssi = cc1120_spi_single_read(CC1120_FIFO_ACCESS);					/* Read the RSSI. */
		rx_lqi = cc1120_spi_single_read(CC1120_FIFO_ACCESS) & CC1120_LQI_MASK;	/* Read the LQI. */
															/* Release SPI Lock. */
		PRINTFRX("\tRX OK - %d byte packet.\n", length);
	}
		
	if(radio_pending & RX_FIFO_UNDER) {										/* FIFO underflow */
		length = 0;															/* Set length to 0. */
		PRINTFRXERR("\tERROR: RX FIFO underflow.\n");
	}
	cc1120_flush_rx();														/* Make sure that the RX FIFO is empty. */
	CC1120_RELEASE_SPI();
	
	return length;
}

int
cc1120_driver_channel_clear(void)
{
	PRINTF("**** Radio Driver: CCA ****\n");
	uint8_t cca, cur_state, rssi0;
	rtimer_clock_t t0;

	if(!locked) {	
		cur_state = cc1120_get_state();							/* Get the current state. */
		if(cur_state == CC1120_STATUS_TX) {
			return 0;											/* Channel can't be clear in TX. */
		} 
		if(cur_state == CC1120_STATUS_RX || cur_state == CC1120_STATUS_SETTLING) {		
			rssi0 = cc1120_spi_single_read(CC1120_ADDR_RSSI0);		/* Get the current RSSI0 value. */
			t0 = RTIMER_NOW();										/* Get the current time. */
			while(!(rssi0 & CC1120_CARRIER_SENSE_VALID)) {
				if(RTIMER_CLOCK_LT((t0 + RTIMER_SECOND/10), RTIMER_NOW())) {
					printf("\t RSSI Timeout, s=%02x. r=%02x\n", cur_state, rssi0);					/* Wait till the CARRIER_SENSE is valid. */
					return 0;
				}
				rssi0 = cc1120_spi_single_read(CC1120_ADDR_RSSI0);	/* Get the current RSSI0 value. */
				watchdog_periodic();								/* Feed the dog. */
			}
			
			if(rssi0 & CC1120_RSSI0_CARRIER_SENSE) {
				PRINTF("\t Channel NOT clear.\n");
				return 0;
			}
		}
	}
	PRINTF("\t Channel clear.\n");
	return 1;
}

int
cc1120_driver_receiving_packet(void)
{
	PRINTF("**** Radio Driver: Receiving Packet? ");
	uint8_t pqt;
	if(!locked) {
		if(!((radio_pending & TRANSMITTING) || (cc1120_get_state() != CC1120_STATUS_RX))) {
			pqt = cc1120_spi_single_read(CC1120_ADDR_MODEM_STATUS1);	/* Check PQT. */
			if((pqt & CC1120_MODEM_STATUS1_SYNC_FOUND)) { // ((pqt & CC1120_MODEM_STATUS1_PQT_REACHED) && (pqt & CC1120_MODEM_STATUS1_PQT_VALID)) || || (cc1120_read_rxbytes() > 0)) //|| !(cc1120_arch_read_gpio3()))
				PRINTF(" Yes. ****\n");
				return 1;
			} 
		}
	}
	return 0;
}

int
cc1120_driver_pending_packet(void)
{
	PRINTFRX("**** Radio Driver: Pending Packet? ");
	if((packet_pending > 0)) {
		PRINTFRX(" yes ****\n");
		return 1;		
	}
	return 0;
}

int
cc1120_driver_on(void)
{
	PRINTF("**** Radio Driver: On ****\n");
	
	// TODO: If we are in SLEEP before this, do we need to do a cal and reg restore?
	
	if(locked) {
		lock_on = 1;		/* Mark to turn on after SPI Release. */
		lock_off = 0;
	} else {
		on();				/* Set CC1120 into RX. */
	}
	return 1;
}

int
cc1120_driver_off(void)
{
	PRINTF("**** Radio Driver: Off ****\n");

	if(locked) {
		lock_off = 1;	/* Radio is locked, indicate that we want to turn off. */
		lock_on = 0;
	} else {
		off();
	}
	return 1;
}


/* --------------------------- CC1120 Support Functions --------------------------- */
uint8_t
cc1120_set_channel(uint8_t channel)
{
	uint32_t freq_registers;
	
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
	if(cc1120_spi_single_read(CC1120_ADDR_PARTVERSION) == 0x21) {
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
		
		if(calResults_for_vcdac_start_high[0] > calResults_for_vcdac_start_mid[0]) {
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO2, calResults_for_vcdac_start_high[0]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_high[1]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_high[2]);
		} else {
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO2, calResults_for_vcdac_start_mid[0]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_mid[1]);
			cc1120_spi_single_write(CC1120_ADDR_FS_VCO4, calResults_for_vcdac_start_mid[2]);
		}
	} else {
		/* Strobe CAL and wait for completion. */
		cc1120_set_state(CC1120_STATE_CAL);
		while(cc1120_get_state() == CC1120_STATUS_CALIBRATE);
	}
	
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
	if((radio_pending & RX_FIFO_UNDER) || (radio_pending & RX_FIFO_OVER)) {
		/* RX FIFO has previously overflowed or underflowed, flush. */
		cc1120_flush_rx();
	}
	
	if((packet_pending == 0) && (cc1120_read_rxbytes > 0)) {
		/* Stray data in FIFO, flush. */
		cc1120_flush_rx();
	}
	
	if(!(radio_on && cc1120_get_state() == CC1120_STATUS_RX)) {
		radio_on = 1;
		cc1120_set_state(CC1120_STATE_RX);		/* Put radio into RX. */
		ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	}
}

static void
off(void)
{
	/* Wait for any current TX to end */
	BUSYWAIT_UNTIL((cc1120_get_state() != CC1120_STATUS_TX), RTIMER_SECOND/10);
	
	cc1120_set_state(CC1120_STATE_IDLE);		/* Set state to IDLE.  This will flush the RX FIFO if there is an error. */
	radio_on = 0;
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	
	cc1120_set_state(CC1120_OFF_STATE);			/* Put radio into the off state defined in platform-conf.h. */
}

void
CC1120_LOCK_SPI(void)
{
	locked++;
}

void 
CC1120_RELEASE_SPI(void)
{
	locked--;
	
	if(locked == 0) {
		if(lock_off) {
			off();
			lock_off = 0;
		} else if(lock_on) {
			on();
			lock_on = 0;
		}
	}
}


/* ---------------------------- CC1120 State Functions ---------------------------- */
uint8_t
cc1120_set_state(uint8_t state)
{
	uint8_t cur_state;
	
	cur_state = cc1120_get_state();			/* Get the current state. */
	if(cur_state == CC1120_STATUS_RX_FIFO_ERROR) {
		cur_state = cc1120_flush_rx();		/* If there is a RX FIFO Error, clear it. */
	} else if(cur_state == CC1120_STATUS_TX_FIFO_ERROR) {
		cur_state = cc1120_flush_tx();		/* If there is a TX FIFO Error, clear it. */
	}
	
	switch(state) {					/* Change state. */
		case CC1120_STATE_FSTXON:	/* Can only enter from IDLE, TX or RX. */
				PRINTFSTATE("\t\tEntering FSTXON (%02x) from %02x\n", state, cur_state);					
				if(!((cur_state == CC1120_STATUS_IDLE) || (cur_state == CC1120_STATUS_TX)
					|| (cur_state == CC1120_STATUS_FSTXON))) {
					cc1120_set_idle(cur_state);							/* Must be in IDLE, TX or FSTXON. If not, get to IDLE. */
				}
				cc1120_spi_cmd_strobe(CC1120_STROBE_SFSTXON);			/* Strobe FSTXON*/
				while(cc1120_get_state() != CC1120_STATUS_FSTXON);		/* Wait till we are in FSTXON. */
				return CC1120_STATUS_FSTXON;
								
		case CC1120_STATE_XOFF:		/* Can only enter from IDLE. */
				PRINTFSTATE("\t\tEntering XOFF (%02x) from %02x\n", state, cur_state);
				if(cur_state != CC1120_STATUS_IDLE) {
					cc1120_set_idle(cur_state);							/* If we are not in IDLE, get us there. */
				}
				cc1120_spi_cmd_strobe(CC1120_STROBE_SXOFF);				/* Strobe XOFF. */
				return CC1120_STATUS_IDLE;
								
		case CC1120_STATE_CAL:		/* Can only enter from IDLE. */
				PRINTFSTATE("\t\tEntering CAL (%02x) from %02x \n", state, cur_state);
				if(cur_state != CC1120_STATUS_IDLE) {
					cc1120_set_idle(cur_state);							/* If we are not in IDLE, get us there. */
				}
				cc1120_spi_cmd_strobe(CC1120_STROBE_SCAL);				/* Strobe CAL. */
				while(cc1120_get_state() != CC1120_STATUS_CALIBRATE);	/* Wait till CAL complete. */
				return CC1120_STATUS_CALIBRATE;
								
		case CC1120_STATE_RX:		/* Can only enter from IDLE, FSTXON or TX. */
				PRINTFSTATE("\t\tEntering RX (%02x) from %02x\n", state, cur_state);
				if (cur_state == CC1120_STATUS_RX) {
					cc1120_set_idle(cur_state);
					cc1120_flush_rx();
				} else if (!((cur_state == CC1120_STATUS_IDLE) || (cur_state == CC1120_STATUS_FSTXON)
					|| (cur_state == CC1120_STATUS_TX))) {
					while(!((cur_state == CC1120_STATUS_IDLE) || (cur_state == CC1120_STATUS_FSTXON)
						|| (cur_state == CC1120_STATUS_TX))) {
							cur_state = cc1120_get_state();				/* We are in a state that will end up in IDLE, FSTXON or TX. Wait till we are there. */
					}
				}
				return cc1120_set_rx();									/* Return RX state. */
				break;
								
		case CC1120_STATE_TX:		/* Can only enter from IDLE, FSTXON or RX. */
				PRINTFSTATE("\t\tEntering TX (%02x) from %02x\n", state, cur_state);
				if((cur_state == CC1120_STATUS_RX)){
					cur_state = cc1120_set_idle(cur_state);				/* Get us out of RX. */
				}

				if(!((cur_state == CC1120_STATUS_IDLE) || (cur_state == CC1120_STATUS_FSTXON))) {
					while(!((cur_state == CC1120_STATUS_IDLE) || (cur_state == CC1120_STATUS_FSTXON)
						|| (cur_state == CC1120_STATUS_RX)) ) {			/* In a state that will end up in IDLE, FSTXON or RX. Wait till there. */
							cur_state = cc1120_get_state();				/* Get the current state. */
					}
				}
				return cc1120_set_tx();									/* Return TX state. */
				break;
								
		case CC1120_STATE_IDLE:		/* Can enter from any state. */
				PRINTFSTATE("\t\tEntering IDLE (%02x) from %02x\n", state, cur_state);
				if(cur_state != CC1120_STATUS_IDLE) {
					cur_state = cc1120_set_idle(cur_state);				/* Set Idle. */
				}
				return cur_state;										/* Return IDLE state. */
								
		case CC1120_STATE_SLEEP:	/* Can only enter from IDLE. */
				PRINTFSTATE("\t\tEntering SLEEP (%02x) from %02x\n", state, cur_state);
				if(cur_state != CC1120_STATUS_IDLE)
				{
					cur_state = cc1120_set_idle(cur_state);				/* If we are not in IDLE, get us there. */
				}
				cc1120_spi_cmd_strobe(CC1120_STROBE_SPWD);				/* Strobe Sleep. */

				return cur_state;										/* return IDLE as there is no sleep state. */
				break;
								
		default:					/* An invalid state has been requested.  Do nothing. */
				printf("Invalid State Req\n"); 
				return CC1120_STATUS_STATE_MASK;
				break;	
	}
}

uint8_t
cc1120_get_state(void)
{
	return (cc1120_spi_cmd_strobe(CC1120_STROBE_SNOP) & CC1120_STATUS_STATE_MASK);	/* Get the current state from the status byte. */
}


/* -------------------------- CC1120 State Set Functions -------------------------- */
uint8_t
cc1120_set_idle(uint8_t cur_state)
{
	PRINTFSTATE("Entering IDLE ");
	cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);		/* Send IDLE strobe. */
	while(cur_state != CC1120_STATUS_IDLE) {
		clock_delay(10);							/* Spin until we are in IDLE. */
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR) {
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);		/* Handle TX FIFO Error. */
		} else if(cur_state == CC1120_STATUS_RX_FIFO_ERROR) {		
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);		/* Handle RX FIFO Error. */
		}
		cur_state = cc1120_get_state();				/* Get current state. */
	}
	
	return CC1120_STATUS_IDLE;						/* Return IDLE state. */
}

uint8_t
cc1120_set_rx(void)
{
	cc1120_spi_cmd_strobe(CC1120_STROBE_SRX);		/* Enter RX. */
	BUSYWAIT_UNTIL((cc1120_get_state() == CC1120_STATUS_RX), RTIMER_SECOND/10);		/* Spin until we are in RX. */

	return cc1120_get_state();
}

uint8_t
cc1120_set_tx(void)
{
	uint8_t cur_state;

	cc1120_spi_cmd_strobe(CC1120_STROBE_STX);		/* Enter TX. */
	cur_state = cc1120_get_state();					/* If we are NOT in TX, Spin until we are in TX. */
	while(cur_state != CC1120_STATUS_TX) {			// TODO: give this a timeout?
		clock_delay(1);								/* Wait for a little. */
		cur_state = cc1120_get_state();				/* Get the current state. */
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR) {	
			return cc1120_flush_tx();				/* TX FIFO Error - flush TX & return. */	
		}
	}		

	return CC1120_STATUS_TX;
}

uint8_t
cc1120_flush_rx(void)
{
	uint8_t cur_state;
	rtimer_clock_t t0;
	
	cur_state = cc1120_get_state();						/* Get the current state. */
	if((cur_state != CC1120_STATUS_IDLE) && (cur_state != CC1120_STATUS_RX_FIFO_ERROR)) {
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR) {
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);	/* TX FIFO Error.  Flush TX FIFO. */
		}
		cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);		/* If not in IDLE or RXERROR, get to IDLE. */
		t0 = RTIMER_NOW();								/* Get time for timeout. */
		while((cur_state != CC1120_STATUS_IDLE) 
				&& RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 10))) {
			cur_state = cc1120_get_state();				/* Wait till in IDLE. */
			watchdog_periodic();
		}
	}
	
	cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);			/* FLush RX FIFO. */
	BUSYWAIT_UNTIL((cc1120_get_state() == CC1120_STATUS_IDLE), RTIMER_SECOND/10);	/* Spin until we are in IDLE. */

	radio_pending &= ~(RX_FIFO_OVER | RX_FIFO_UNDER);	/* Mark as no RX FIFO errors. */
	packet_pending = 0;									/* Mark as no packet pending. */

	return cc1120_get_state();
}

uint8_t
cc1120_flush_tx(void)
{
	uint8_t cur_state;
	rtimer_clock_t t0;
	
	cur_state = cc1120_get_state();						/* Get the current state. */
	if((cur_state != CC1120_STATUS_IDLE) && (cur_state != CC1120_STATUS_TX_FIFO_ERROR)) {
		if(cur_state == CC1120_STATUS_RX_FIFO_ERROR) {
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);	/* RX FIFO Error.  Flush RX FIFO. */
		}
		cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);		/* If not in IDLE or TXERROR, get to IDLE. */
		t0 = RTIMER_NOW();								/* Get the current time. */
		while((cur_state != CC1120_STATUS_IDLE) 
				&& RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 10))) {
			cur_state = cc1120_get_state();				/* Wait till in IDLE. */
			watchdog_periodic();
		}
	}
	
	cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);			/* FLush TX FIFO. */
	BUSYWAIT_UNTIL((cc1120_get_state() == CC1120_STATUS_IDLE), RTIMER_SECOND/10);	/* Spin until we are in IDLE. */

	radio_pending &= ~(TX_FIFO_ERROR);					/* Clear TX_FIFO_ERROR Flag. */
	
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
	if(addr & CC1120_EXTENDED_MEMORY_ACCESS_MASK) {
		status = cc1120_arch_spi_rw_byte(CC1120_ADDR_EXTENDED_MEMORY_ACCESS | rw | burst); 
		cc1120_arch_spi_rw_byte(addr & CC1120_ADDRESS_MASK);
	} else {
		status = cc1120_arch_spi_rw_byte(addr | rw | burst);
	}
	
	return status;
}

static void
cc1120_write_txfifo(uint8_t *payload, uint8_t payload_len)
{
	cc1120_arch_spi_enable();											/* Enable the CC1120. */
	cc1120_arch_txfifo_load(payload, payload_len);						/* Load the TX FIFO with the payload. */
	cc1120_arch_spi_disable();											/* Disable the CC1120. */	
	
	PRINTFTX("\t%d bytes in fifo (%d + length byte requested)\n", cc1120_read_txbytes());
}


/* -------------------------- CC1120 Interrupt Handler --------------------------- */
int
cc1120_interrupt_handler(void)
{
	uint8_t marc_status;
	LEDS_ON(LEDS_BLUE);
	cc1120_arch_interrupt_acknowledge();
	
	if(cc1120_arch_spi_enabled()) {
		cc1120_arch_spi_disable();										/* Check if we have interrupted an SPI function, if so disable SPI. */
	}
	
	marc_status = cc1120_spi_single_read(CC1120_ADDR_MARC_STATUS1);
	if(marc_status != CC1120_MARC_STATUS_OUT_NO_FAILURE) {				/* Check that we don't have a "No Failure" interrupt. */
		PRINTFINT("\t CC1120 Int. %d\n", marc_status);
		
		if(marc_status == CC1120_MARC_STATUS_OUT_RX_FINISHED) {			/* We have received a packet.  This is done first to make RX faster. */
			if(!(radio_pending & ACK_PENDING)) {							/* Check that we are no waiting for an ACK. */
				packet_pending++;											/* Mark Packet Pending. */
				process_poll(&cc1120_process);								/* Poll the CC1120 process. */
			}
			
		} else if(marc_status == CC1120_MARC_STATUS_OUT_TX_FINISHED) {	/* TX has finished. */
			radio_pending |= TX_COMPLETE;									/* Mark that TX is complete. */
			radio_pending &= ~(TX_ERROR);									/* CLear any TX Error. */
			
		} else if(marc_status == CC1120_MARC_STATUS_OUT_TX_FINISHED) { 	/* RX FIFO has overflowed. */
			PRINTFRXERR("\t!!! RX FIFO Error: Overflow. !!!\n");
			cc1120_flush_rx();												/* Flush the RX FIFO. */
			if(radio_on){	
				cc1120_set_state(CC1120_STATE_RX);							/* Turn the radio back on if it is meant to be on. */
			}	
			
		} else if(marc_status == CC1120_MARC_STATUS_OUT_RX_UNDERFLOW) { /* RX FIFO has underflowed. */
			PRINTFRXERR("\t!!! RX FIFO Error: Underflow. !!!\n");
			radio_pending |= RX_FIFO_UNDER;									/* Mark that the RXFIFO has underflowed. */	
			printf("RUF\n\r");	
			
		} else if((marc_status == CC1120_MARC_STATUS_OUT_TX_OVERFLOW) ||
				(marc_status == CC1120_MARC_STATUS_OUT_TX_UNDERFLOW)) { /* TX FIFO has errored. */
			PRINTFTXERR("\t!!! TX FIFO Error !!!\n");
			radio_pending |= TX_FIFO_ERROR;									/* Mark that a TX FIFO error has occured. */
			printf("TFERR\n\r");
			
		} else if(marc_status == CC1120_MARC_STATUS_OUT_EWOR_SYNC_LOST) { /* EWOR Sync has been lost. */
			printf("EWOR\n\r");							
		}
		
		/* Other events:
		 * 				CC1120_MARC_STATUS_OUT_RX_TIMEOUT			RX termination due to timeout.
		 * 				CC1120_MARC_STATUS_OUT_RX_TERMINATION		RX Terminated on CS or PQT.
		 * 				CC1120_MARC_STATUS_OUT_PKT_DISCARD_LEN		Packet discarded due to being too long. Flush RX FIFO?
		 * 				CC1120_MARC_STATUS_OUT_PKT_DISCARD_ADR		Packet discarded due to bad address. 
		 * 				CC1120_MARC_STATUS_OUT_PKT_DISCARD_CRC		Packet discarded due to bad CRC.
		 * 				CC1120_MARC_STATUS_OUT_TX_ON_CCA_FAIL		TX on CCA Failed due to busy channel.
		 */
	}

	LEDS_OFF(LEDS_BLUE);
	return 1;
}


/* ----------------------------------- CC1120 Process ------------------------------------ */
PROCESS_THREAD(cc1120_process, ev, data)
{	
	PROCESS_POLLHANDLER(processor());									/* Register the Pollhandler. */
	
	PROCESS_BEGIN();
	printf("CC1120 Driver Start\n");	
	PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXIT);					/* Wait till the process is terminated. */
	printf("CC1120 Driver End\n");
	PROCESS_END();
}

/* ---------------------------- CC1120 Process Poll Handler ------------------------------ */	
void processor(void)
{			
	uint8_t len;	
	uint8_t buf[CC1120_MAX_PAYLOAD];
			
	PRINTFPROC("** Process Poll **\n");
	LEDS_ON(LEDS_RED);													/* Turn on the packet processing LED. */
	
	len = cc1120_driver_read_packet(buf, CC1120_MAX_PAYLOAD);			/* Read the packet from the radio. */
	if(len) {
		PRINTFPROC("\tPacket Length: %d\n", len);	
		packetbuf_clear(); 												/* Clear the packetbuffer. */
		memcpy(packetbuf_dataptr(), (void *)buf, len);					/* Load the packet buffer. */
		packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rx_rssi);				/* Read RSSI. */
		packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, rx_lqi);		/* Read LQI. */
		packetbuf_set_datalen(len);										/* Set Packetbuffer length. */
		RIMESTATS_ADD(llrx);
		NETSTACK_RDC.input();											/* Pass the packet up the stack for processing. */
	}
		
	LEDS_OFF(LEDS_RED);													/* Turn off the packet processing LED. */
	if(radio_on) {
		cc1120_set_state(CC1120_STATE_RX);		//on();					/* Turn the radio back on if it is meant to be on. */ 
	}
}
