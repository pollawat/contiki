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
#define ACK_FRAME_CONTROL_MSB	0x41
#define ACK_FRAME_CONTROL_LSB	0x88


/* -------------------- Internal Function Definitions. -------------------- */
static void on(void);
static void off(void);
static void LOCK_SPI(void);
static void RELEASE_SPI(void);
static void processor(void);

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
static uint8_t current_channel, packet_pending, fifo_access, broadcast, ack_seq, tx_seq, lbt_success, radio_pending, int_enabled, radio_on, txfirst, txlast = 0;
static uint8_t locked, lock_on, lock_off;

static uint8_t ack_buf[ACK_LEN];
static uint8_t tx_buf[CC1120_MAX_PAYLOAD];

static uint8_t tx_len;

/* ------------------- Radio Driver Functions ---------------------------- */

int 
cc1120_driver_init(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** CC1120 Radio  Driver: Init ****\n");
#endif
	uint8_t part = 0;
	
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
	
	/* Enable CC1120 interrupt. */
	cc1120_arch_interrupt_enable();
	radio_pending = 0;
	packet_pending = 0;
	tx_len = 0;
	locked = 0;
	
#if CC1120DEBUG || DEBUG
	printf("\tCC1120 Initialised and OFF\n");
#endif
	return 1;
}

int
cc1120_driver_prepare(const void *payload, unsigned short len)
{
	PRINTFTX("**** Radio Driver: Prepare ****\n");

	if(len > CC1120_MAX_PAYLOAD)
	{
		/* Packet is too large - max packet size is 125 bytes. */
		PRINTFTXERR("!!! PREPARE ERROR: Packet too large. !!!\n");
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
	
	/* Keep a local copy of the FIFO. */
	memcpy(tx_buf, payload, len);
	tx_len = len;
	RIMESTATS_ADD(lltx);
	
	if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) 
	{
		broadcast = 1;
		PRINTFTX("\tBroadcast\n");
	}
	else
	{
		broadcast = 0;
		tx_seq = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
		PRINTFTX("\tUnicast, Seqno = %d\n", tx_seq);
	}
	lbt_success = 0;
	
	return RADIO_TX_OK;
}

int
cc1120_driver_transmit(unsigned short transmit_len)
{
	PRINTFTX("\n\n**** Radio Driver: Transmit ****\n");
	uint8_t txbytes, cur_state, marc_state, retransmit;
	rtimer_clock_t t0;
	watchdog_periodic();	/* Feed the dog to stop reboots. */
	
	/* Check that the packet is not too large. */
	if(transmit_len > CC1120_MAX_PAYLOAD)
	{
		/* Packet is too large - max packet size is 125 bytes. */
		PRINTFTX("!!! TX ERROR: Packet too large. !!!\n");

		return RADIO_TX_ERR;
	}
	LOCK_SPI();
	radio_pending |= TRANSMITTING;
	radio_pending &= ~(TX_COMPLETE);
	
	LEDS_ON(LEDS_GREEN);
	
	/* check if this is a retransmission */
	txbytes = cc1120_read_txbytes();
	if(txbytes == 0)
	{
		PRINTFTX("\tRetransmit last packet.\n");

		/* Retransmit last packet. */
		cc1120_set_state(CC1120_STATE_IDLE);
		
		/* These registers should only be written in IDLE. */
		cc1120_spi_single_write(CC1120_ADDR_TXFIRST, txfirst);
		cc1120_spi_single_write(CC1120_ADDR_TXLAST, txlast);
	}
	else
	{
		PRINTFTX("\tStoring txfirst and txlast\n");
		
		/* Store TX Pointers. */
		txfirst =  cc1120_spi_single_read(CC1120_ADDR_TXFIRST);
		txlast = cc1120_spi_single_read(CC1120_ADDR_TXLAST);
		
		/* Set correct TXOFF mode. */
		if(broadcast)
		{
			/* Not expecting an ACK. */
			cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG0, 0x00);	/* Set TXOFF Mode to IDLE as we are not expecting ACK. */
		}
		else
		{
			/* Expecting an ACK. */
			cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG0, 0x30);	/* Set TXOFF Mode to RX as we are expecting ACK. */
		}
	}
	
	txbytes = cc1120_read_txbytes();
	
	if(txbytes != tx_len + 1)
	{
		/* Wrong amount of data in the FIFO. Re-write the FIFO. */
		printf("re-populate FIFO. %d, %d\n", txbytes, tx_len);
		cc1120_flush_tx();
		printf("txbytes = %d\n", cc1120_read_txbytes());
		cc1120_write_txfifo(tx_buf, tx_len);
		printf("txbytes = %d\n", cc1120_read_txbytes());
	}


#if RDC_CONF_HARDWARE_CSMA
	/* If we use LBT... */
	

	if(lbt_success == 0) 
	{
		PRINTFTX("\tTransmitting with LBT.\n");
		
		/* Set RX if radio is not already in it. */
		if(cc1120_get_state() != CC1120_STATUS_RX)
		{   
			PRINTFTX("\tEnter RX for CCA.\n");		
			cc1120_set_state(CC1120_STATE_RX);
		}
		
		PRINTFTX("\tWait for valid RSSI.");
		
		/* Wait for RSSI to be valid. */
		while(!(cc1120_spi_single_read(CC1120_ADDR_RSSI0) & (CC1120_RSSI_VALID)))
		{
			PRINTFTX(".");
			watchdog_periodic();	/* Feed the dog to stop reboots. */
		}
		PRINTFTX("\n");
		PRINTFTX("\tTX: Enter TX\n");
	}
	else
	{
		/* Retransmitting last packet not using LBT. */
		PRINTFTX("Retransmitting last packet.\n");
		if(cc1120_get_state() == CC1120_STATUS_RX)
		{   
			PRINTFTX("\tEnter IDLE.\n");		
			cc1120_set_state(CC1120_STATE_IDLE);
		}
	}

	t0 = RTIMER_NOW();
	cc1120_spi_cmd_strobe(CC1120_STROBE_STX);	/* Strobe TX. */
	cur_state = cc1120_get_state();
	
	/* Block until in TX.  If timeout is reached, strobe IDLE and 
	 * reset CCA to clear TX & flush FIFO. */
	while(cur_state != CC1120_STATUS_TX)
	{
		watchdog_periodic();	/* Feed the dog to stop reboots. */
		
		if(RTIMER_CLOCK_LT((t0 + CC1120_LBT_TIMEOUT), RTIMER_NOW()) )
		{
			/* Timeout reached. */
			cc1120_set_state(CC1120_STATE_IDLE);
			
			// TODO: Do we need to reset the CCA mode?
			
			/* Set Energest and TX flag. */
			ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
			radio_pending &= ~(TRANSMITTING);
				
			/* Turn off LED if it is being used. */
			LEDS_OFF(LEDS_GREEN);	

			cc1120_flush_tx();		
			RELEASE_SPI();	
			
			PRINTFTXERR("!!! TX ERROR: Collision before TX - Timeout reached. !!!\n");
			RIMESTATS_ADD(contentiondrop);
			lbt_success = 0;
			/* Return Collision. */
			return RADIO_TX_COLLISION;
		}
		else if (radio_pending & TX_FIFO_ERROR)
		{
			/* Set Energest and TX flag. */
			ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
			radio_pending &= ~(TRANSMITTING);
			cc1120_flush_tx();

			PRINTFTXERR("!!! TX ERROR: FIFO Error. !!!\n");	
			LEDS_OFF(LEDS_GREEN);		/* Turn off LED if it is being used. */
			
			RELEASE_SPI();
			lbt_success = 0;
			return RADIO_TX_ERR;
		}
		cur_state = cc1120_get_state();
	}
	lbt_success = 1;
	
#else /* RDC_CONF_HARDWARE_CSMA */
	PRINTFTX("\tTransmitting without LBT.\n");
	PRINTFTX("\tTX: Enter TX\n");

	/* Enter TX. */
	cur_state = cc1120_set_state(CC1120_STATE_TX);

	if(cur_state != CC1120_STATUS_TX)
	{
		/* We didn't TX... */
		radio_pending &= ~(TRANSMITTING);

		PRINTFTXERR("!!! TX ERROR: did not enter TX. Current state = %02x !!!\n", cur_state);
		
		if(radio_pending & TX_FIFO_ERROR)
		{
			cc1120_flush_tx();
		}

		RELEASE_SPI();
		LEDS_OFF(LEDS_GREEN);	/* Turn off LED if it is being used. */			
		
		return RADIO_TX_ERR;
	}
#endif /* WITH_SEND_CCA */	
	
	PRINTFTX("\tTX: in TX.");
	ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
	t0 = RTIMER_NOW();
	
	/* Block till TX is complete. */	
	while(!(radio_pending & TX_COMPLETE))
	{
		/* Wait for CC1120 interrupt handler to set TX_COMPLETE. */
		watchdog_periodic();	/* Feed the dog to stop reboots. */
		PRINTFTX(".");
		
		if(radio_pending & TX_FIFO_ERROR)
		{
			/* TX FIFO has underflowed during TX.  Need to flush TX FIFO. */
			cc1120_flush_tx();
			break;
		}
		
		if(RTIMER_CLOCK_LT((t0 + RTIMER_SECOND/20), RTIMER_NOW()))
		{
			/* Timeout for TX. At 802.15.4 50kbps data rate, the entire 
			 * TX FIFO (all 128 bits) should be transmitted in 0.02 
			 * seconds. Timeout set to 0.05 seconds to be sure.  If 
			 * the interrupt has not fired by this time then something 
			 * went wrong. 
			 * 
			 * This timeout needs to be adjusted if lower data rates 
			 * are used. */	 
			if((cc1120_read_txbytes() == 0) && !(radio_pending & TX_FIFO_ERROR))
			{
				/* We have actually transmitted everything in the FIFO. */
				radio_pending |= TX_COMPLETE;
				PRINTFTXERR("!!! TX ERROR: TX timeout reached but packet sent !!!\n");
			}
			else
			{
				cc1120_set_state(CC1120_STATE_IDLE);
				PRINTFTXERR("!!! TX ERROR: TX timeout reached !!!\n");						
			}
			break;
		}
	}
	
	PRINTFTX("\n");	
	ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);			
	LEDS_OFF(LEDS_GREEN);
	RELEASE_SPI();
	radio_pending &= ~(TRANSMITTING);
	t0 = RTIMER_NOW();
		
	if((!(radio_pending & TX_COMPLETE)) || (cc1120_read_txbytes() > 0))
	{	
		PRINTFTXERR("\tTX NOT OK %d, %d.\n",(radio_pending & TX_COMPLETE), cc1120_read_txbytes() );	
		cc1120_flush_tx();		/* Flush TX FIFO. */
		cc1120_flush_rx();		/* Flush RX FIFO. */
		
		printf("txbytes = %d\n", cc1120_read_txbytes());
		
		radio_pending &= ~(TX_ERROR | ACK_PENDING);
		return RADIO_TX_ERR;
	}
	else
	{
		PRINTFTX("\tTX OK.\n");
		
		radio_pending &= ~(TX_ERROR);
		cur_state = cc1120_get_state();
		marc_state = cc1120_spi_single_read(CC1120_ADDR_MARCSTATE) & 0x1F;	
		
		if((marc_state == CC1120_MARC_STATE_MARC_STATE_TX_END) && (cur_state == CC1120_STATUS_TX))
		{
			cc1120_set_state(CC1120_STATE_IDLE);
		}
		else if(cur_state == CC1120_STATUS_TX)
		{
			/* Should never get here, just here for security. */
			printf("!!!!! TX ERROR: Still in TX according to status byte. !!!\n");
			cc1120_set_state(CC1120_STATE_IDLE);
		}
		
		if(broadcast)
		{
			/* We have TX'd broadcast successfully. We are not expecting 
			 * an ACK so clear RXFIFO incase and return OK. */
			PRINTFTX("\tBroadcast TX OK\n");
			cc1120_flush_rx();	
			radio_pending &= ~(ACK_PENDING);	/* NOT expecting and ACK. */	
			return RADIO_TX_OK;	
		}		
		else 
		{
			/* We have successfully sent the packet but want an ACK. */
			if(packet_pending)
			{
				/* There is a packet inthe FIFO that should not be there */
				cc1120_flush_rx();
			}
			if(cc1120_get_state() != CC1120_STATUS_RX)
			{
				/* Need to be in RX for the ACK. */
				cc1120_set_state(CC1120_STATE_RX);
			}
			radio_pending |= ACK_PENDING;
			ack_seq = 0;
			
			/* Wait for the ACK. */
			while((RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CC1120_ACK_WAIT)) && (ack_seq != tx_seq))
			{
				/* Wait till timeout or ACK is received. */
				watchdog_periodic();		/* Feed the dog to stop reboots. */
			}
			cc1120_set_state(CC1120_STATE_IDLE);
			radio_pending &= ~(ACK_PENDING);
			
			if(ack_seq == tx_seq)
			{
				/* We have received the required ACK. */
				PRINTFTX("\tACK received. TX Seqno:%d, ACK Seqno:%d\n",tx_seq, ack_seq);
				return RADIO_TX_OK;
			}
			else
			{
				/* No ACK received. */
				PRINTFTX("\tNo ACK received.\n");
				return RADIO_TX_NOACK;
			}
			
		}	
		return RADIO_TX_OK;
	}
	return RADIO_TX_ERR;
}

int
cc1120_driver_send_packet(const void *payload, unsigned short payload_len)
{
	PRINTFTX("**** Radio Driver: Send ****\n");
	if(cc1120_driver_prepare(payload, payload_len) != RADIO_TX_OK)
	{
		return RADIO_TX_ERR;
	}
	
	return cc1120_driver_transmit(payload_len);
}

int
cc1120_driver_read_packet(void *buf, unsigned short buf_len)
{
	PRINTF("**** Radio Driver: Read ****\n");

	uint8_t length,  rxbytes = 0;
		
	if(radio_pending & RX_FIFO_UNDER)
	{
		/* FIFO underflow */
		cc1120_flush_rx();
		PRINTFRXERR("\tERROR: RX FIFO underflow.\n");		
		return 0;		
	}
	
	watchdog_periodic();
	rxbytes = cc1120_read_rxbytes();
	
	if(rxbytes < CC1120_MIN_PAYLOAD)
	{
		/* not enough data. */
		cc1120_flush_rx();
		RIMESTATS_ADD(tooshort);
		
		PRINTFRXERR("\tERROR: Packet too short\n");
		return 0;
	}
	
	/* Read length byte. */
	rxbytes = cc1120_read_rxbytes();
	length = cc1120_spi_single_read(CC1120_FIFO_ACCESS);
	if((length + 2) > rxbytes)
	{
		/* Packet too long, out of Sync? */
		cc1120_flush_rx();
			
		RIMESTATS_ADD(badsynch);
		PRINTFRXERR("\tERROR: not enough data in FIFO. Length = %d, rxbytes = %d\n", length, rxbytes);
		return 0;
	}
	
	if(length > CC1120_MAX_PAYLOAD)
	{
		/* Packet too long, out of Sync? */
		cc1120_flush_rx();
		
		RIMESTATS_ADD(badsynch);
		PRINTFRXERR("\tERROR: Packet longer than FIFO or bad sync\n");
		return 0;
	}
	
	if((length) > buf_len) 
	{
		/* Packet is too long. */
		cc1120_flush_rx();
		
		RIMESTATS_ADD(toolong);
		PRINTFRXERR("\tERROR: Packet too long for buffer\n");	
		return 0;
	}
	
	LOCK_SPI();
	if((length == ACK_LEN) && (radio_pending & ACK_PENDING))
	{
		PRINTFRX("\tACK Received.\n");
		/* We have received an ACK that we were expecting. */
		cc1120_arch_spi_enable();
		cc1120_arch_rxfifo_read(ack_buf, length);
		cc1120_arch_spi_disable();
		fifo_access = 0;
		
		watchdog_periodic();
		
		ack_seq = ack_buf[2];	
		radio_pending &= ~(ACK_PENDING);
		
		if(radio_pending & RX_FIFO_UNDER)
		{
			/* FIFO underflow */
			cc1120_flush_rx();	
		}
		RELEASE_SPI();
		return 0;
	}
	else
	{
		PRINTFRX("\tPacket received.\n");
		if(buf == packetbuf_dataptr())
		{
			/* Working with the packetbuffer. */
			packetbuf_clear(); /* Clear the packetbuffer. */	
		}		 
		watchdog_periodic();	/* Feed the dog to stop reboots. */
		
		/* We have received a normal packet. Read the packet. */
		PRINTFRX("\tRead FIFO.\n");
		cc1120_arch_spi_enable();
		cc1120_arch_rxfifo_read(buf, length);
		cc1120_arch_spi_disable();
		PRINTFRX("\tPacketRead\n");
		
		watchdog_periodic();	/* Feed the dog to stop reboots. */
		if(radio_pending & RX_FIFO_UNDER)
		{
			/* FIFO underflow */
			RELEASE_SPI();
			cc1120_flush_rx();
			PRINTFRXERR("\tERROR: RX FIFO underflow. Meant to have %d bytes\n", length);	
			return 0;		
		}
		
		RELEASE_SPI();
		
		if(buf == packetbuf_dataptr())
		{
			PRINTFRX("\tPopulate additional info.\n");
			/* Working with the packetbuffer. */
			packetbuf_set_datalen(length);		/* Set Packetbuffer length. */
			
			/* Read RSSI & LQI. */
			packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc1120_spi_single_read(CC1120_FIFO_ACCESS));
			packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, (cc1120_spi_single_read(CC1120_FIFO_ACCESS) & CC1120_LQI_MASK));
			
			if(radio_pending & RX_FIFO_UNDER)
			{
				/* FIFO underflow */
				cc1120_flush_rx();
				PRINTFRXERR("\tERROR: RX FIFO underflow.\n");
				return 0;		
			}
			
			
			
			/* Work out if we need to send an ACK. */
			if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_node_addr))
			{
				PRINTFRX("\tSending ACK\n");
				
				rtimer_clock_t t0;                                                  \ 
    
				/* Packet is for this node. */
				watchdog_periodic();	/* Feed the dog to stop reboots. */
				
				/* Populate ACK Frame buffer. */
				ack_buf[0] = ACK_FRAME_CONTROL_MSB;
				ack_buf[1] = ACK_FRAME_CONTROL_LSB;
				ack_buf[2] = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
				
				radio_pending &= ~(TX_COMPLETE);
				
				LOCK_SPI();
				
				/* Make sure that the TX FIFO is empty & write ACK to it. */
				cc1120_flush_tx();
				cc1120_write_txfifo(ack_buf, ACK_LEN);
				
				/* Transmit ACK WITHOUT LBT. */
				cc1120_spi_cmd_strobe(CC1120_STROBE_STX);
				t0 = RTIMER_NOW(); 
				
				/* Block till TX is complete. */	
				while(!(radio_pending & TX_COMPLETE))
				{
					/* Wait for CC1120 interrupt handler to set TX_COMPLETE. */
					watchdog_periodic();	/* Feed the dog to stop reboots. */
					PRINTFRX(".");
					
					if(radio_pending & TX_FIFO_ERROR)
					{
						/* TX FIFO has underflowed during ACK TX.  Need to flush TX FIFO. */
						cc1120_flush_tx();
						break;
					}
					
					if(RTIMER_CLOCK_LT((t0 + RTIMER_SECOND/20), RTIMER_NOW()))
					{
						/* Timeout for TX. At 802.15.4 50kbps data rate, the entire 
						 * TX FIFO (all 128 bits) should be transmitted in 0.02 
						 * seconds. Timeout set to 0.05 seconds to be sure.  If 
						 * the interrupt has not fired by this time then something 
						 * went wrong. 
						 * 
						 * This timeout needs to be adjusted if lower data rates 
						 * are used. */	 
						if((cc1120_read_txbytes() == 0) && !(radio_pending & TX_FIFO_ERROR))
						{
							/* We have actually transmitted everything in the FIFO. */
							radio_pending |= TX_COMPLETE;
							PRINTFTXERR("!!! TX ERROR: TX timeout reached but packet sent !!!\n");
						}
						else
						{
							cc1120_set_state(CC1120_STATE_IDLE);
							PRINTFTXERR("!!! TX ERROR: TX timeout reached !!!\n");						
						}
						break;
					}
				}
				
				/* Re-load data into TX FIFO. */
				cc1120_flush_tx();
				cc1120_write_txfifo(tx_buf, tx_len);
				
				RELEASE_SPI();
			}
		}
		
		RIMESTATS_ADD(llrx);
		PRINTFRX("\tRX OK - %d byte packet.\n", length);
	}
	
	if(packet_pending > 1)
	{
		cc1120_flush_rx();
	}
	else
	{
		packet_pending = 0;
	}

	/* Return read length. */
	return length;
}

int
cc1120_driver_channel_clear(void)
{
	PRINTF("**** Radio Driver: CCA ****\n");
	if(locked)
	{
		printf("SPI Locked\n");
		return 1;
	}
	else
	{
		uint8_t cca, cur_state;
		rtimer_clock_t t0;
		uint8_t rssi0 = cc1120_spi_single_read(CC1120_ADDR_RSSI0);
		
		cur_state = cc1120_get_state();
		
		if(cur_state == CC1120_STATUS_TX)
		{
			/* Channel can't be clear in TX. */
			PRINTF(" - NO, in TX. ****\n");
			return 0;
		}
		if(cur_state != CC1120_STATUS_RX)
		{
			/* Not in RX... */
			return 1;
		}
		
		/* Wait till the CARRIER_SENSE is valid. */
		t0 = RTIMER_NOW();
		
		while(!(rssi0 & CC1120_CARRIER_SENSE_VALID))
		{
			if(RTIMER_CLOCK_LT((t0 + RTIMER_SECOND/20), RTIMER_NOW()))
			{
				printf("\t RSSI Timeout.\n");		
				LEDS_OFF(LEDS_BLUE);
				return 0;
			}
			rssi0 = cc1120_spi_single_read(CC1120_ADDR_RSSI0);
			watchdog_periodic();
		}
		
		if(rssi0 & CC1120_RSSI0_CARRIER_SENSE)
		{
			cca = 0;
			PRINTF("\t Channel NOT clear.\n");
			LEDS_OFF(LEDS_BLUE);	
		}
		else
		{
			cca = 1;
			PRINTF("\t Channel clear.\n");
			LEDS_ON(LEDS_BLUE);
		}
		
		return cca;
	}
}

int
cc1120_driver_receiving_packet(void)
{
	PRINTF("**** Radio Driver: Receiving Packet? ");
	uint8_t pqt;
	if(locked)
	{
		PRINTF("SPI Locked\n");
		return 0;
	}
	else
	{
		if(radio_pending & TRANSMITTING)
		{
			/* Can't be receiving in TX. */
			PRINTF(" - NO, in TX. ****\n");
			return 0;
		}
		else if(cc1120_get_state() != CC1120_STATUS_RX)
		{
			PRINTF(" - NO, Radio OFF. ****\n");
			/* cannot be receiving with the radio off. */
			return 0;
		}
		else
		{
			pqt = cc1120_spi_single_read(CC1120_ADDR_MODEM_STATUS1);			/* Check PQT. */
			if(((pqt & CC1120_MODEM_STATUS1_PQT_REACHED) && (pqt & CC1120_MODEM_STATUS1_PQT_VALID))
				|| (pqt & CC1120_MODEM_STATUS1_SYNC_FOUND))// || (cc1120_read_rxbytes() > 0)) //|| !(cc1120_arch_read_gpio3()))
			{
				PRINTF(" Yes. ****\n");
				return 1;
			}
			else
			{
				/* Not receiving */
				PRINTF(" - NO. ****\n");
				return 0;
			}
		}
		return 0;
	}
}

int
cc1120_driver_pending_packet(void)
{
	PRINTFRX("**** Radio Driver: Pending Packet? ");

	if((packet_pending > 0))
	{
		PRINTFRX(" yes ****\n");
		return 1;		
	}
	else
	{
		PRINTFRX(" no ****\n");
		return 0;
	}
}

int
cc1120_driver_on(void)
{
	PRINTF("**** Radio Driver: On ****\n");
	/* Set CC1120 into RX. */
	// TODO: If we are in SLEEP before this, do we need to do a cal and reg restore?
	
	if(locked)
	{
		lock_on = 1;
		lock_off = 0;
		return 1;
	}
	
	on();
	return 1;
}

int
cc1120_driver_off(void)
{
	PRINTF("**** Radio Driver: Off ****\n");

	if(locked)
	{
		/* Radio is locked, indicate that we want to turn off. */
		lock_off = 1;
		lock_on = 0;
		return 1;
	}
	
	off();
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

	cc1120_spi_single_write(CC1120_ADDR_IOCFG3, CC1120_GPIO3_FUNC);

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
	
	/* Set RXEND to go into IDLE after good packet and to never timeout. */
	cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG1, 0x0F);
	
	/* Set TXOFF to go to RX for ACK and to stay in RX on bad packet. */
	cc1120_spi_single_write(CC1120_ADDR_RFEND_CFG0, 0x30);
	
	
	/* Set the RSSI Offset.  This is a two's compliment number and
	 * affects the value of anything involving the RSSI, including
	 * the Carrier Sense Threshold and RSSI 11:0. */
	cc1120_spi_single_write(CC1120_ADDR_AGC_GAIN_ADJUST, (CC1120_RSSI_OFFSET));   

	/* Set Carrier Sense Threshold. This is a two's compliment number. */
	cc1120_spi_single_write(CC1120_ADDR_AGC_CS_THR, (CC1120_CS_THRESHOLD));   

#if RDC_CONF_HARDWARE_CSMA	
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
	if((radio_pending & RX_FIFO_UNDER) || (radio_pending & RX_FIFO_OVER))
	{
		/* RX FIFO has previously overflowed or underflowed, flush. */
		cc1120_flush_rx();
	}
	
	if((packet_pending == 0) && (cc1120_read_rxbytes > 0))
	{
		/* Stray data in FIFO, flush. */
		cc1120_flush_rx();
	}
	
	if(!(radio_on && cc1120_get_state() == CC1120_STATUS_RX))
	{
		radio_on = 1;
		
		/* Put radio into RX. */
		cc1120_set_state(CC1120_STATE_RX);
		// TODO: Do we want to wait till radio osc is stable?

		ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	}
}

static void
off(void)
{
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
	locked--;
	//printf("Release %u\n", locked);
	
	if(locked == 0) 
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
	uint8_t cur_state;

	/* Enter TX. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_STX);
	
	cur_state = cc1120_get_state();
	
	/* If we are NOT in TX, Spin until we are in TX. */

	while(cur_state != CC1120_STATUS_TX)
	{
#if CC1120DEBUG || DEBUG || CC1120TXDEBUG
		printf(",");
#endif				
		cur_state = cc1120_get_state();
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
		{	
			/* TX FIFO Error - flush TX. */	
			return cc1120_flush_tx();
		}
		clock_delay(1);
	}		

	
	// TODO: give this a timeout?

	/* Return TX state. */
	return CC1120_STATUS_TX;
}

uint8_t
cc1120_flush_rx(void)
{
	uint8_t cur_state = cc1120_get_state();
	rtimer_clock_t t0 = RTIMER_NOW();
	
	if((cur_state != CC1120_STATUS_IDLE) || (cur_state != CC1120_STATUS_RX_FIFO_ERROR))
	{
		/* If not in IDLE or TXERROR, get to IDLE. */
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
		{
			/* TX FIFO Error.  Flush TX FIFO. */
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);
		}
		while((cur_state != CC1120_STATUS_IDLE) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 10)))
		{
			if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
			{
				/* TX FIFO Error.  Flush TX FIFO. */
				cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);
			}
			/* Get the current state and strobe IDLE. */
			cur_state = cc1120_get_state();
			cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);
			watchdog_periodic();
		}
	}
	
	/* FLush RX FIFO. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);

	/* Spin until we are in IDLE. */
	BUSYWAIT_UNTIL((cc1120_get_state() == CC1120_STATUS_IDLE), RTIMER_SECOND/10);

	radio_pending &= ~(RX_FIFO_OVER | RX_FIFO_UNDER);
	packet_pending = 0;
	LEDS_OFF(LEDS_RED);

	/* Return IDLE state. */
	return CC1120_STATUS_IDLE;
}

uint8_t
cc1120_flush_tx(void)
{
	uint8_t cur_state = cc1120_get_state();
	rtimer_clock_t t0 = RTIMER_NOW();
	
	if((cur_state != CC1120_STATUS_IDLE) || (cur_state != CC1120_STATUS_TX_FIFO_ERROR))
	{
		/* If not in IDLE or TXERROR, get to IDLE. */
		if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
		{
			/* RX FIFO Error.  Flush RX FIFO. */
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);
		}
		while((cur_state != CC1120_STATUS_IDLE) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 10)))
		{
			if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
			{
				/* RX FIFO Error.  Flush RX FIFO. */
				cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);
			}
			/* Get the current state and strobe IDLE. */
			cur_state = cc1120_get_state();
			cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);
			watchdog_periodic();
		}
	}
	
	/* FLush TX FIFO. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);
	watchdog_periodic();
	
	/* Spin until we have flushed TX and are in IDLE. */
	while((cur_state != CC1120_STATUS_IDLE) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 5)))
	{
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
		{
			/* (Another) TX FIFO error, flush. */
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);
		}
		else if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
		{
			/* RX FIFO Error. Flush it. */
			cc1120_spi_cmd_strobe(CC1120_STROBE_SFRX);
		}
		else if (cur_state != CC1120_STATUS_IDLE)
		{
			/* Not in IDLE - re-strobe. */
			cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);
		}
		/* Get the current state. */
		cur_state = cc1120_get_state();
		watchdog_periodic();
	}

	/* Clear TX_FIFO_ERROR Flag. */
	radio_pending &= ~(TX_FIFO_ERROR);
	
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
	
	PRINTFTX("\t%d bytes in fifo (%d + length byte requested)\n", cc1120_read_txbytes());

}



/* -------------------------- CC1120 Interrupt Handler --------------------------- */
int
cc1120_interrupt_handler(void)
{
	uint8_t marc_status = cc1120_spi_single_read(CC1120_ADDR_MARC_STATUS1);
	
	
	if(marc_status == CC1120_MARC_STATUS_OUT_NO_FAILURE)
	{
		cc1120_arch_interrupt_acknowledge();
		return 0;
	}
	
	PRINTFINT("\t CC1120 Int. %d\n", marc_status);
	
	if(marc_status == CC1120_MARC_STATUS_OUT_RX_FINISHED)
	{
		/* We have received a packet.  This is done first to make RX faster. */
		LEDS_ON(LEDS_RED);
		packet_pending++;
		
		/* Acknowledge the interrupt. */
		cc1120_arch_interrupt_acknowledge();
		
		process_poll(&cc1120_process);
		return 1;
	}	
	
	switch (marc_status){
		case CC1120_MARC_STATUS_OUT_RX_TIMEOUT:	
			/* RX terminated due to timeout.  Should not get here as there is  */							
			break;
			
		case CC1120_MARC_STATUS_OUT_RX_TERMINATION:	
			/* RX Terminated on CS or PQT. */								
			break;
			
		case CC1120_MARC_STATUS_OUT_EWOR_SYNC_LOST:	
			/* EWOR Sync lost. */								
			break;
			
		case CC1120_MARC_STATUS_OUT_PKT_DISCARD_LEN:	
			/* Packet discarded due to being too long. Flush RX FIFO? */	
			break;
			
		case CC1120_MARC_STATUS_OUT_PKT_DISCARD_ADR:	
			/* Packet discarded due to bad address - should not get here 
			 * as address matching is not being used. Flush RX FIFO? */						
			break;
			
		case CC1120_MARC_STATUS_OUT_PKT_DISCARD_CRC:	
			/* Packet discarded due to bad CRC. Should not need to flush 
			 * RX FIFI as CRC_AUTOFLUSH is set in FIFO_CFG*/			
			break;
			
		case CC1120_MARC_STATUS_OUT_TX_OVERFLOW:	
			/* TX FIFO has overflowed. */
			radio_pending |= TX_FIFO_ERROR;											
			break;
			
		case CC1120_MARC_STATUS_OUT_TX_UNDERFLOW:	
			/* TX FIFO has underflowed. */
			PRINTFTXERR("\t!!! TX FIFO Error: Underflow. !!!\n");
			radio_pending |= TX_FIFO_ERROR;					
			break;
			
		case CC1120_MARC_STATUS_OUT_RX_OVERFLOW:	
			/* RX FIFO has overflowed. */
			PRINTFRXERR("\t!!! RX FIFO Error: Overflow. !!!\n");
			cc1120_flush_rx();	
			//radio_pending |= RX_FIFO_OVER;									
			break;
			
		case CC1120_MARC_STATUS_OUT_RX_UNDERFLOW:	
			/* RX FIFO has underflowed. */
			PRINTFRXERR("\t!!! RX FIFO Error: Underflow. !!!\n");
			radio_pending |= RX_FIFO_UNDER;
			//cc1120_flush_rx();									
			break;	
			
		case CC1120_MARC_STATUS_OUT_TX_ON_CCA_FAIL:	
			/* TX on CCA Failed due to busy channel. */
			//cc1120_spi_cmd_strobe(CC1120_STROBE_STX);
			//radio_pending |= TX_ERROR;										
			break;
			
		case CC1120_MARC_STATUS_OUT_TX_FINISHED:	
			/* TX Finished. */
			radio_pending |= TX_COMPLETE;
			radio_pending &= ~(TX_ERROR);														
			break;
			
		default:
			break;
	}
	/* Acknowledge the interrupt. */
	cc1120_arch_interrupt_acknowledge();	
	return 1;
}





/* ----------------------------------- CC1120 Process ------------------------------------ */

PROCESS_THREAD(cc1120_process, ev, data)
{	
	PROCESS_POLLHANDLER(processor());	
	
	PROCESS_BEGIN();

	printf("cc1120_process: started\n");
	
	PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXIT);
	
	printf("cc1120_process: terminated\n");
	
	PROCESS_END();
}

		
void processor(void)
{		
	int len;	
			
	PRINTFPROC("** Process Poll **\n");

	watchdog_periodic();		
	LEDS_ON(LEDS_RED);
	
	PRINTFPROC("\tRead Packet\n");
	
	len = cc1120_driver_read_packet(packetbuf_dataptr(), PACKETBUF_SIZE);
	
	PRINTFPROC("\tPacket Length: %d\n", len);	
	
	if(len != 0)
	{		
		PRINTFPROC("\tProcess Packet\n");
		watchdog_periodic();	/* Feed the dog to stop reboots. */
		NETSTACK_RDC.input();
		PRINTFPROC("\tPacket Processed.\n");
	}	
	LEDS_OFF(LEDS_RED);
	if(locked)
	{
		printf("Locked %u\n", locked); 
	}
	
	if(radio_on)
	{
		on();
	}
}
	

