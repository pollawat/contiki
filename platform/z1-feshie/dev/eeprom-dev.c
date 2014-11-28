#include "I2Croutines.h"				/* From application note SLAA208A */
#include "eeprom-dev.h"
#include <stdint.h>
#include <stdio.h>

/* The 24xx102x behaves like two 24xx51x's on the same bus. 
It has an inactive A2 pin. In the I2C address it has a "Block Select"
bit B0 where the A2 bit would be for a 24xx51x. It therefore requires
a new I2C address to be sent when crossing the 512-page boundary, so
to make life easier, let's treat each 24xx102x as two chips  */ 

uint8_t currentchip = 0;
const uint8_t i2c_address[] = I2C_ADDRESSES;

typedef struct 
{
	uint8_t chip;
	uint16_t page;
	uint16_t offset;
} address_t;

void 
select_chip(uint8_t chip)
{
	currentchip = chip;
	InitI2C(i2c_address[currentchip]);
#ifdef PRINTF_DEBUG
	printf("Selected chip %d, I2C address %x \r\n", currentchip, i2c_address[currentchip]);
#endif
}

eeprom_error_t 
eeprom_read(uint32_t logicaladdress, uint32_t size, uint8_t* data)
{
	uint8_t chip = 0;
	uint32_t offset, bytesleft;
	uint16_t bytestoread;

#ifdef PRINTF_DEBUG
	printf("eeprom_read(...) logicaladdress = 0x%lx \r\n", logicaladdress);
#endif

	if ( (logicaladdress + size) > LOGICAL_SIZE ){
		return EE_OUT_OF_RANGE;
	}

	chip = (uint8_t)(logicaladdress / CHIP_SIZE); /* Find the chip to start in */
	select_chip(chip);
	offset = logicaladdress - (chip * CHIP_SIZE); /* Find offset within chip */
	
	bytesleft = size;
	while (bytesleft){
		bytestoread = bytesleft;

		if (offset + bytestoread > CHIP_SIZE){
			bytestoread = CHIP_SIZE - offset;
		}

		EEPROM_SequentialRead(offset, &data[size - bytesleft], bytestoread);

		bytesleft -= bytestoread;
				
		if (bytesleft){
			offset = 0;
			chip++;
			select_chip(chip);
		}
	}
	return EE_OK;
}
								 
eeprom_error_t 
eeprom_write(uint32_t logicaladdress, uint32_t size, uint8_t* data)
{
	uint8_t chip = 0;
	uint32_t offset, bytesleft;
	volatile uint32_t endofpage;
	uint16_t bytestowrite;
	
#ifdef PRINTF_DEBUG
	printf("eeprom_write(...) logicaladdress = 0x%lx \r\n", logicaladdress);
#endif

	if ( (logicaladdress + size) > LOGICAL_SIZE ){
		return EE_OUT_OF_RANGE;
	}

	chip = (uint8_t)(logicaladdress / CHIP_SIZE); /* Find the chip to start in */
	select_chip(chip);
	offset = logicaladdress - (chip * CHIP_SIZE); /* Find offset within chip */

	bytesleft = size;
	while (bytesleft){
		endofpage = (((offset / PAGE_SIZE) * PAGE_SIZE) + PAGE_SIZE); /* Find address of last byte in current page */
		if ((offset + bytesleft) > endofpage){		/* Write no further than end of current page */
			bytestowrite = endofpage - offset;
		}else{
			bytestowrite = bytesleft;
		}
		if (offset + bytestowrite > CHIP_SIZE){ /* Select next chip if necessary */
				offset = 0;
				chip++;
				select_chip(chip);
				continue;
		}
		
		EEPROM_PageWrite(offset, &data[size - bytesleft], bytestowrite); 

		bytesleft -= bytestowrite; /* Update bytesleft and offset after write */
		offset += bytestowrite;
	}

	return EE_OK;
}


void 
eepromtest(void)
{
	printf("EEPROM TEST: Needs writing\n");
}

