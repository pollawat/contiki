#ifndef __EEPROM__H
#define __EEPROM__H

#include <stdint.h>
#include "I2Croutines.h" 

#define B0 (1<<2)								/* Block select bit for EEPROM command byte*/

#ifndef SNOW
    #define NUMBER_OF_CHIPS 8               
    /* Number of chips on ice-cape board's I2C bus (max 8) */
    #define I2C_ADDRESSES {0x50, (0x50|B0), 0x51, (0x51|B0), 0x52, (0x52|B0), 0x53, (0x53|B0)}
#else
    #define NUMBER_OF_CHIPS 4               
    /* Number of chips on snow-cape board's I2C bus (max 8) */
    #define I2C_ADDRESSES {0x50, (0x50|B0), 0x51, (0x51|B0)} 
#endif

#define PAGE_SIZE ((uint8_t)(MAXPAGEWRITE)) 					/* Number of bytes per page */
#define PAGES_PER_CHIP ((uint16_t)(512))			/* Number of pages per chip */
#define CHIP_SIZE ((uint32_t)PAGE_SIZE * (uint32_t)PAGES_PER_CHIP)
#define LOGICAL_SIZE ((uint32_t)(CHIP_SIZE * NUMBER_OF_CHIPS))

typedef enum {EE_OK = 0, EE_OUT_OF_RANGE} eeprom_error_t;
eeprom_error_t eeprom_write(uint32_t logicaladdress, uint32_t size, uint8_t* data);
eeprom_error_t eeprom_read(uint32_t logicaladdress, uint32_t size, uint8_t* data);
void eepromtest(void);
#endif
