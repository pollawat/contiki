/**
 * \file
 *         TI CC1120 driver.
 * \author
 *         Graeme Bragg <g.bragg@ecs.soton.ac.uk>
 *         Phil Basford <pjb@ecs.soton.ac.uk>
 */
 

/* CC1120 headers. */
#include "cc1120.h"

/* --------------------------- Radio Driver Structure --------------------------- */
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


/* --------------------------- Radio Driver Functions --------------------------- */
int 
cc1120_driver_init(void)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Init ****\n");
	#endif

}

int
cc1120_driver_prepare(const void *payload, unsigned short len)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Prepare ****\n");
	#endif
}

int
cc1120_driver_transmit(unsigned short transmit_len)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Transmit ****\n");
	#endif
}

int
cc1120_driver_send_packet(const void *payload, unsigned short payload_len)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Send ****\n");
	#endif
}

int
cc1120_driver_read_packet(void *buf, unsigned short buf_len)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Read ****\n");
	#endif
}

int
cc1120_driver_channel_clear(void)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: CCA ****\n");
	#endif
}

int
cc1120_driver_receiving_packet(void)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Receiving Packet? ****\n");
	#endif
}

int
cc1120_driver_pending_packet(void)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Pending Packet? ****\n");
	#endif
}

int
cc1120_driver_on(void)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: On ****\n");
	#endif
}

int
cc1120_driver_off(void)
{
	#if CC1120DEBUG || DEBUG
		printf("**** Radio Driver: Off ****\n");
	#endif
}


/* --------------------------- CC1120 Support Functions --------------------------- */
uint8_t
cc1120_set_channel(uint8_t channel)
{
	//TODO: Need to check country, should probably be set in platform?
	//TODO: Need to do manual calibration after setting channel due to errata...
}

uint8_t
cc1120_get_channel(void)
{

}

uint8_t
cc1120_set_state(uint8_t state)
{

}

uint8_t
cc1120_get_state(void)
{

}

uint8_t
read_txbytes(void)
{

}

uint8_t
read_rxbytes(void)
{

}




/* --------------------------- CC1120 SPI Functions --------------------------- */
uint8_t
cc1120_spi_single_read(uint16_t addr)
{
	LOCK_SPI();
	cc1120_arch_spi_enable();
	
	cc1120_spi_write_addr(
	
	addr = cc1120_arch_spi_rw_byte(0);		/* Get the value.  Re-use addr to save a byte. */ 
	
	cc1120_arch_spi_disable();
	RELEASE_SPI();
	
	return addr;
}

uint8_t
cc1120_spi_single_write(uint16_t addr, uint8_t val)
{

}

void
cc1120_spi_burst_read(uint16_t addr, uint8_t *buffer, uint8_t count)
{

}

void
cc1120_spi_burst_write(uint16_t addr, uint8_t *buffer, uint8_t count)
{

}

void
cc1120_spi_write_txfifo(uint8_t *data, uint8_t len)
{

}

uint8_t
cc1120_spi_write_addr(uint16_t addr, uint8_t burst, uint8_t rw)
{
	uint8_t status;
	if(addr & CC1120_EXTENDED_MEMORY_ACCESS_MASK)
	{
		status = cc1120_arch_spi_rw_byte(CC1120_EXTENDED_MEMORY_ACCESS | rw | burst); 
		(void) cc1120_arch_spi_rw_byte(addr & CC1120_ADDRESS_MASK);
	}
	else
	{
		status = cc1120_arch_spi_rw_byte(addr | rw | burst);
	}
	
	return status;
}

/* --------------------------- CC1120 misc Functions - needed? --------------------------- */

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
reset(void)
{

}



int
cc1120_rx_interrupt(void)
{

}




