/**
 * \file
 *         TI CC1120 driver.
 * \author
 *         Graeme Bragg <g.bragg@ecs.soton.ac.uk>
 *         Phil Basford <pjb@ecs.soton.ac.uk>
 */
 

/* CC1120 headers. */
#include "cc1120.h"
#include "cc1120-arch.h"
#include "cc1120-config.h"


/* Internal variables. */
static volatile uint8_t current_channel, transmitting, radio_on, tx_error = 0;

/* Temporary SPI lock/unlock while we decide how to handle SPI exclusive access. */
#define LOCK_SPI() while(0) { }
#define RELEASE_SPI() while(0) { }


/* ---------------------------- Radio Driver Structure ---------------------------- */
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


/* ---------------------------- Radio Driver Functions ---------------------------- */
int 
cc1120_driver_init(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Init ****\n");
#endif
	uint8_t part = 0;
	
	/* Init arch */
	cc1120_arch_init();
	
	/* Reset CC1120 */
	cc1120_arch_reset();
	
	/* Check CC1120 - we read the part number register as a test. */
	part = cc1120_spi_single_read(CC1120_ADDR_PARTNUMBER);
	switch(part)
	{
		case CC1120_PART_NUM_CC1120:
									printf("CC1120 Detected - Radio OK\n");
									break;
		case CC1120_PART_NUM_CC1121:
									printf("CC1121 Detected - Radio OK\n");
									break;
		case CC1120_PART_NUM_CC1125:
									printf("CC1125 Detected - Radio OK\n");
									break;
		case CC1120_PART_NUM_CC1175:							
									printf("CC1175 Detected\n");
									printf("*** ERROR: CC1175 is a transmitter only. Replace radio with a supported type and reset. ***\n");
									while(1);	/* Spin ad infinitum as we cannot continue. */
									break;
		default:	/* Not a supported chip or no chip present... */
						printf("*** ERROR: Unsupported radio connected or no radio present (Part Number %02x detected) ***\n", part);
						printf("*** Check radio and reset ***\n");
						while(1);	/* Spin ad infinitum as we cannot continue. */
						break;
	}
	
	// TODO: Cover sync-word errata somewhere?
	
	/* Configure CC1120 */
	cc1120_register_config();
	cc1120_gpio_config();
	
	/* Set Channel */
	cc1120_set_channel(RF_CHANNEL);
	
    /* Set radio off */
	cc1120_driver_off();
#if CC1120DEBUG || DEBUG
	printf("\tCC1120 Initialised and OFF\n");
#endif
	return 1;
}

int
cc1120_driver_prepare(const void *payload, unsigned short len)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Prepare ****\n");
#endif

	uint8_t txbytes;

	if(len > CC1120_MAX_PAYLOAD)
	{
		/* Packet is too large - max packet size is 125 bytes. */
#if CC1120DEBUG || DEBUG
		printf("!!! PREPARE ERROR: Packet too large. !!!\n");
#endif
		return RADIO_TX_ERR;
	}
	
	/* Read number of bytes in TX FIFO. */
	txbytes = cc1120_read_txbytes();
#if CC1120DEBUG || DEBUG
		printf("\t%d bytes in TXFIFO, flushing.\n", txbytes);
#endif
	
	/* If the FIFO is not empty, flush it. Otherwise we might send multiple TXs. */
	if(txbytes != 0)
	{
		cc1120_set_state(CC1120_STATE_IDLE);
		cc1120_flush_tx();
	}
	
	/* Write to the FIFO. */
	if(cc1120_write_txfifo(payload, len))
	{
		return RADIO_TX_OK;
	}
	else
	{
		return RADIO_TX_ERR;
	}
}

int
cc1120_driver_transmit(unsigned short transmit_len)
{
	
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Transmit ****\n");
#endif
	
	uint8_t txbytes, cur_state;
	
	/* Check that the packet is not too large. */
	if(transmit_len > CC1120_MAX_PAYLOAD)
	{
		/* Packet is too large - max packet size is 125 bytes. */
#if CC1120DEBUG || DEBUG
		printf("!!! TX ERROR: Packet too large. !!!\n");
#endif
		return RADIO_TX_ERR;
	}
		
	/* check that we have enough in the FIFO */
	txbytes = cc1120_read_txbytes();
	if((transmit_len + 1) != txbytes)
	{
#if CC1120DEBUG || DEBUG
		printf("!!! TX ERROR: wrong number of bytes in FIFO. Wanted %d + 1, have %d !!!\n", transmit_len, txbytes);
#endif	
		return RADIO_TX_ERR;
	}
	
	transmitting = 1;
	
	/* Disable CC1120 interrupt to prevent a spurious trigger. */
	cc1120_arch_interrupt_disable();
	
	/* Enter TX. */
	cur_state = cc1120_set_state(CC1120_STATE_TX);
	if(cur_state == CC1120_STATUS_TX)
	{
		ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
		ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	
		/* wait till we leave TX. */
		while(cur_state == CC1120_STATUS_TX)
		{
			cur_state = cc1120_get_state();
		}
		
		ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
		transmitting = 0;
		
		/* Enable CC120 Interrupt. */
		cc1120_arch_interrupt_enable();
		
		if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
		{
			/* TX FIFO Error. Need to clear it... */
			tx_error = 1;
			cc1120_set_state(CC1120_STATE_IDLE);
			cc1120_flush_tx();
		}		
		
		if((cur_state != CC1120_STATUS_RX) && (radio_on))
		{
			cc1120_driver_on();
		}
		else if(radio_on)
		{
			ENERGEST_ON(ENERGEST_TYPE_LISTEN);
		}
		
		if(tx_error)
		{
			/* we have had a FIFO error. */
#if CC1120DEBUG || DEBUG
			printf("!!! TX ERROR: FIFO error. !!!\n");
#endif			
			tx_error = 0;
			return RADIO_TX_ERR;
		}
		
		
		/* Check that TX was successful. */
		txbytes = cc1120_read_txbytes();
		if(txbytes != 0)
		{
			/* we have not transmitted what we wanted to. */
#if CC1120DEBUG || DEBUG
			printf("!!! TX ERROR: have not transmitted everything: %d bytes left in TX FIFO !!!\n",  txbytes);
#endif			
			return RADIO_TX_ERR;
		}
		
		/* We have TX'ed successfully. */
#if CC1120DEBUG || DEBUG
		printf("!!! TX OK: Transmission sent !!!\n");
#endif	
		return RADIO_TX_OK;
	}
	else
	{
		/* We didn't TX... */
		transmitting = 0;
#if CC1120DEBUG || DEBUG
	printf("!!! TX ERROR: did not enter TX. Current state = %02x !!!\n", cur_state);
#endif			
		return RADIO_TX_ERR;
	}
}

int
cc1120_driver_send_packet(const void *payload, unsigned short payload_len)
{
#if CC1120DEBUG || DEBUG
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
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Read ****\n");
#endif

	/* Temporary return for TX testing. */
	return 0;
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
	
#if CC1120_CCA_PIN_PRESENT
	/* we have a CCA Pin so we can check it instead of reading RSSI register */
	cca = cc1120_arch_read_cca();
#endif
#if !CC1120_CCA_PIN_PRESENT 
	/* We don't have a CCA pin so we do a CCA the hard way - 
	 * Read CARRIER_SENSE and CARRIER_SENSE_VALID from RSSI0. */
	uint8_t rssi0 = c1120_spi_single_read(CC11xx_RSSI0);
	
	/* Wait till the CARRIER_SENSE is valid. */
    while(!(rssi0 & CC1120_CARRIER_SENSE_VALID))
	{
		rssi0 = c1120_spi_single_read(CC11xx_RSSI0);
	}
	//TODO: Timeout on this.
	
	if((rssi0 & CC1120_RSSI0_CARRIER_SENSE) == CC1120_RSSI0_CARRIER_SENSE)
	{
		cca = 0;
	}
	else
	{
		cca = 1;
	}
#endif
	
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
	printf("**** Radio Driver: Receiving Packet? ****\n");
#endif

	/* Temporary return for TX debug. */
	return 0;
}

int
cc1120_driver_pending_packet(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Pending Packet? ****\n");
#endif

	/* Temporary return for TX debug. */
	return 0;
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
	
	
	/* Enable CC1120 RX interrupt*/
	cc1120_arch_interrupt_enable();
	
	/* Radio on. */
	cc1120_set_state(CC1120_STATE_RX);
	radio_on = 1;
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	
	return 1;
}

int
cc1120_driver_off(void)
{
#if CC1120DEBUG || DEBUG
	printf("**** Radio Driver: Off ****\n");
#endif
	// TODO: If TXOFF_MODE is set not to go to IDLE, shall we set it to do so?
	
	/* Flush the RX and TX FIFOs. */
	cc1120_set_state(CC1120_STATE_IDLE);
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	
	cc1120_flush_tx();
	cc1120_flush_rx();
	
	/* Put CC1120 into IDLE or sleep? Leave it up to platform-conf.h*/
	cc1120_set_state(CC1120_OFF_STATE);
	
	/* Disable CC1120 RX interrupt. */
	cc1120_arch_interrupt_disable();
	
	/* Irrelevant return... */
	radio_on = 0;
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

uint8_t
cc1120_set_channel(uint8_t channel)
{
	uint32_t freq_registers;
	
	freq_registers = (channel * CC1120_CHANNEL_MULTIPLIER) + CC1120_BASE_FREQ;
	
	cc1120_spi_single_write(CC1120_ADDR_FREQ0, ((unsigned char*)&freq_registers)[0]);
	cc1120_spi_single_write(CC1120_ADDR_FREQ1, ((unsigned char*)&freq_registers)[1]);
	cc1120_spi_single_write(CC1120_ADDR_FREQ2, ((unsigned char*)&freq_registers)[2]);
	
	
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

uint8_t
cc1120_write_txfifo(uint8_t *payload, uint8_t payload_len)
{
	uint8_t fifo_len = 0;
	
	/* Write the length byte to the start of the FIFO. */
	cc1120_spi_single_write(CC1120_FIFO_ACCESS, payload_len);

	/* Write the payload to the FIFO. */
	cc1120_spi_burst_write(CC1120_FIFO_ACCESS, payload, payload_len);
	
	fifo_len = cc1120_read_txbytes();
	
#if CC1120DEBUG || DEBUG	
	printf("%d bytes in fifo (%d + length byte requested)\n", fifo_len, payload_len);
#endif

	if(fifo_len != (payload_len + 1))
	{
		/* We haven't written the right amount of data... */
		return 0;
	}
	else
	{
		return 1;
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
									cc1120_spi_cmd_strobe(CC1120_STROBE_SFTXON);	/* Intentional Error to catch warnings. */
									while(cc1120_get_state() != CC1120_STATUS_FSTXON);
								}
								return CC1120_STATUS_FSTXON;
								
		case CC1120_STATE_XOFF:		/* Can only enter from IDLE. */
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
											cur_state = cc1120_get_state;
										}
									
									/* Return RX state. */
									return cc1120_set_rx();
								}
								break;
								
		case CC1120_STATE_TX:		/* Can only enter from IDLE, FSTXON or RX. */
								if((cur_state == CC1120_STATUS_IDLE) 
								|| (cur_state == CC1120_STATUS_FSTXON)
								|| (cur_state == CC1120_STATUS_RX))
								{
									/* Return TX state. */
									// TODO: do we want to set PKT_CFG2.CCA_MODE = 100b (Listen Before Talk) so that TX is entered if in RX and channel is ! clear?
									
									return cc1120_set_tx();
								}
								else if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
								{
									/* If there is a RX FIFO Error, clear it and TX. */
									cc1120_flush_rx();
									return cc1120_set_tx();
								}
								else if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
								{
									/* If there is a TX FIFO Error, clear it and TX. */
									cc1120_flush_tx();
									return cc1120_set_tx();
								}
								else
								{
									/* We are in a state that will end up in IDLE, FSTXON or RX. Wait till we are there. */
									while(!((cur_state == CC1120_STATUS_IDLE)
										|| (cur_state == CC1120_STATUS_FSTXON)
										|| (cur_state == CC1120_STATUS_RX)) )
										{
											cur_state = cc1120_get_state;
										}
									/* Return TX state. */
									return cc1120_set_tx();
								}
								break;
								
		case CC1120_STATE_IDLE:		/* Can enter from any state. */
								/* If we are already in IDLE, do nothing and return the current state. */
								if(cur_state == CC1120_STATUS_RX_FIFO_ERROR)
								{
									/* If there is a RX FIFO Error, clear it. */
									cc1120_flush_rx();
								}
								else if(cur_state == CC1120_STATUS_TX_FIFO_ERROR)
								{
									/* If there is a TX FIFO Error, clear it. */
									cc1120_flush_tx();
								}
								else if(cur_state != CC1120_STATUS_IDLE)
								{
									/* Set Idle. */
									cc1120_set_idle();
								}
								/* Return IDLE state. */
								return CC1120_STATUS_IDLE;
								
		case CC1120_STATE_SLEEP:	/* Can only enter from IDLE. */
								/* If we are not in IDLE, get us there. */
								if(cur_state != CC1120_STATUS_IDLE)
								{
									cc1120_set_idle();
								}
								cc1120_spi_cmd_strobe(CC1120_STROBE_SPWD);
								return CC1120_STATUS_IDLE;
								break;
								
		default:				printf("!!! INVALID STATE REQUESTED !!!\n"); 
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
	/* Send IDLE strobe. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_SIDLE);

	/* Spin until we are in IDLE. */
	while(cc1120_get_state() != CC1120_STATUS_IDLE);
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
	while(cc1120_get_state() != CC1120_STATUS_RX);
	// TODO: give this a timeout?

	
	/* Return RX state. */
	return CC1120_STATUS_RX;
}

uint8_t
cc1120_set_tx(void)
{
	/* Enter TX. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_STX);

	/* Spin until we are in TX. */
	while(cc1120_get_state() != CC1120_STATUS_TX);
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
	while(cc1120_get_state() != CC1120_STATUS_IDLE);
	// TODO: give this a timeout?

	/* Return IDLE state. */
	return CC1120_STATUS_IDLE;
}

uint8_t
cc1120_flush_tx(void)
{
	/* FLush TX FIFO. */
	cc1120_spi_cmd_strobe(CC1120_STROBE_SFTX);

	/* Spin until we are in IDLE. */
	while(cc1120_get_state() != CC1120_STATUS_IDLE);
	// TODO: give this a timeout?

	/* Return IDLE state. */
	return CC1120_STATUS_IDLE;
}



/* ----------------------------- CC1120 SPI Functions ----------------------------- */
uint8_t
cc1120_spi_cmd_strobe(uint8_t strobe)
{
	LOCK_SPI();
	cc1120_arch_spi_enable();
	
	strobe = cc1120_arch_spi_rw_byte(strobe);
	
	cc1120_arch_spi_disable();
	RELEASE_SPI();
	
	return strobe;
}

uint8_t
cc1120_spi_single_read(uint16_t addr)
{
	LOCK_SPI();
	cc1120_arch_spi_enable();
	
	cc1120_spi_write_addr(addr, CC1120_STANDARD_BIT, CC1120_READ_BIT);
	addr = cc1120_arch_spi_rw_byte(0);		/* Get the value.  Re-use addr to save a byte. */ 
	
	cc1120_arch_spi_disable();
	RELEASE_SPI();
	
	return addr;
}

uint8_t
cc1120_spi_single_write(uint16_t addr, uint8_t val)
{
	LOCK_SPI();
	cc1120_arch_spi_enable();
	
	addr = cc1120_spi_write_addr(addr, CC1120_STANDARD_BIT, CC1120_WRITE_BIT);	/* Read the status byte. */
	cc1120_arch_spi_rw_byte(val);		
	
	cc1120_arch_spi_disable();
	RELEASE_SPI();
	
	return addr;
}

void
cc1120_spi_burst_read(uint16_t addr, uint8_t *buf, uint8_t len)
{
	LOCK_SPI();
	cc1120_arch_spi_enable();
	
	cc1120_spi_write_addr(addr, CC1120_BURST_BIT, CC1120_READ_BIT);
	cc1120_arch_spi_rw_buf(buf, NULL, len);
	
	cc1120_arch_spi_disable();
	RELEASE_SPI();
}

void
cc1120_spi_burst_write(uint16_t addr, uint8_t *buf, uint8_t len)
{
	LOCK_SPI();
	cc1120_arch_spi_enable();
	
	cc1120_spi_write_addr(addr, CC1120_BURST_BIT, CC1120_WRITE_BIT);
	cc1120_arch_spi_rw_buf(NULL, buf, len);
	
	cc1120_arch_spi_disable();
	RELEASE_SPI();
}

uint8_t
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


/* -------------------------- CC1120 Interrupt Handler --------------------------- */
int
cc1120_rx_interrupt(void)
{

	/* Temporary return for testing TX. */
	return 0;
}


/* --------------------------- CC1120 misc Functions - needed? --------------------------- */
/*
void
check_txfifo(void)
{

}

void
flushrx(void)
{

}

void
send_ack(uint8_t seqno)
{

}

void
restart_input(void)
{

}

int
is_receiving(void)
{

}

int
is_broadcast_addr(uint8_t mode, uint8_t *addr)
{

}

int
input_byte(uint8_t byte)
{

}

void
cc1120_reset(void)
{

}









*/
