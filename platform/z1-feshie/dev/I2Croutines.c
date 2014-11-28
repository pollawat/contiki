/******************************************************************************/
/*  Communication with an EEPROM (e.g. 2465) via I2C bus                      */
/*  The I2C module of the MSP430F2274 is used to communicate with the EEPROM. */
/*  The "Byte Write", "Page Write", "Current Address Read", "Random Read",    */
/*  "Sequential Read" and "Acknowledge Polling" commands or the EEPROM are    */
/*  realized.                                                                 */
/*                                                                            */
/*  developed with IAR Embedded Workbench V5.20.1                             */
/*                                                                            */
/*  Texas Instruments                                                         */
/*  William Goh                                                               */
/*  March 2011                                                                */
/*----------------------------------------------------------------------------*/
/*  updates                                                                   */
/*    Jan 2005:                                                               */
/*        - updated initialization sequence                                   */
/*    March 2009:                                                             */
/*        - updated code for 2xx USCI module                                  */
/*        - added Page Write and Sequential Read functions                    */
/*    March 2011:                                                             */
/*        - Fixed Random and Sequential Reads to Restart                      */
/*        - Added Page Write to support greater than 256 bytes                */

/*    Jeff Gough - August 2011																							 	*/
/* 				- modified code to support 5xx USCI module.  												*/
/* 				- added optional printf debug enabled by #define PRINTF_DEBUG	  		*/

/*    Phil Basford - Jan 2013 */
/*                  -  added stdint import 
*                   - now uses <legacymsp430.h>

      Phil Basford - Nov 2014
                   - contiki....
*/
/*******************************************************************************
;
; THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
; REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
; INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
; FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
; COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
; TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
; POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
; INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
; YOUR USE OF THE PROGRAM.
;
; IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
; CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
; THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
; OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
; OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
; EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
; REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
; OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
; USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
; AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
; YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
; (U.S.$500).
;
; Unless otherwise stated, the Program written and copyrighted
; by Texas Instruments is distributed as "freeware".  You may,
; only under TI's copyright in the Program, use and modify the
; Program without any charge or restriction.  You may
; distribute to third parties, provided that you transfer a
; copy of this license to the third party and the third party
; agrees to these terms by its first use of the Program. You
; must reproduce the copyright notice and any other legend of
; ownership on each copy or partial copy, of the Program.
;
; You acknowledge and agree that the Program contains
; copyrighted material, trade secrets and other TI proprietary
; information and is protected by copyright laws,
; international copyright treaties, and trade secret laws, as
; well as other intellectual property laws.  To protect TI's
; rights in the Program, you agree not to decompile, reverse
; engineer, disassemble or otherwise translate any object code
; versions of the Program to a human-readable form.  You agree
; that in no event will you alter, remove or destroy any
; copyright notice included in the Program.  TI reserves all
; rights not specifically granted under this license. Except
; as specifically provided herein, nothing in this agreement
; shall be construed as conferring by implication, estoppel,
; or otherwise, upon you, any license or other right under any
; TI patents, copyrights or trade secrets.
;
; You may not use the Program in non-TI devices.
;
******************************************************************************/ 
#include "uart1_i2c_master.h"
#include "I2Croutines.h"

//#define PRINTF_DEBUG
#ifdef PRINTF_DEBUG
#include <stdio.h>							/* for printf debug */
#endif

int PtrTransmit;
unsigned char I2CBufferArray[130];
unsigned char I2CBuffer;
static uint8_t address;

/*----------------------------------------------------------------------------*/
// Description:
//   Initialization of the I2C Module
/*----------------------------------------------------------------------------*/
void 
InitI2C(unsigned char eeprom_i2c_address)
{
	address = (uint8_t)eeprom_i2c_address;
}

/*---------------------------------------------------------------------------*/
// Description:
//   Initialization of the I2C Module for Write operation.
/*---------------------------------------------------------------------------*/
void 
I2CWriteInit(void)
{
    i2c_transmitinit(address);
}

/*----------------------------------------------------------------------------*/
// Description:
//   Initialization of the I2C Module for Read operation.
/*----------------------------------------------------------------------------*/
void 
I2CReadInit(void)
{
  i2c_receiveinit(address);
}

/*----------------------------------------------------------------------------*/
// Description:
//   Byte Write Operation. The communication via the I2C bus with an EEPROM
//   (2465) is realized. A data byte is written into a user defined address.
/*----------------------------------------------------------------------------*/
void 
EEPROM_ByteWrite(unsigned int Address, unsigned char Data)
{
    unsigned char adr_hi;
    unsigned char adr_lo;

    while (i2c_busy());                       // wait until I2C module has
                                              // finished all operations.

    adr_hi = Address >> 8;                    // calculate high byte
    adr_lo = Address & 0xFF;                  // and low byte of address

    I2CBufferArray[2] = adr_hi;               // Low byte address.
    I2CBufferArray[1] = adr_lo;               // High byte address.
    I2CBufferArray[0] = Data;
    PtrTransmit = 2;                          // set I2CBufferArray Pointer

    I2CWriteInit();
    i2c_transmit_n(3, (uint8_t *)&I2CBufferArray);
}

/*----------------------------------------------------------------------------*/
// Description:
//   Page Write Operation. The communication via the I2C bus with an EEPROM
//   (24xx65) is realized. A data byte is written into a user defined address.
/*----------------------------------------------------------------------------*/
void 
EEPROM_PageWrite(unsigned int StartAddress, unsigned char * Data, unsigned int Size)
{
    volatile unsigned int i = 0;
    volatile unsigned char counterI2cBuffer;
    unsigned char adr_hi;
    unsigned char adr_lo;
    unsigned int currentAddress = StartAddress;
    unsigned int currentSize = Size;
    unsigned int bufferPtr = 0;
    unsigned char moreDataToRead = 1;

  	#ifdef PRINTF_DEBUG
  	printf("EEPROM_PageWrite: writing %d bytes from StartAddress 0x%.2x\r\n", Size, StartAddress);
  	#endif

    while (i2c_busy());                // wait until I2C module has
                                              // finished all operations.
   
    // Execute until no more data in Data buffer
    while(moreDataToRead){
        adr_hi = currentAddress >> 8;           // calculate high byte
        adr_lo = currentAddress & 0xFF;         // and low byte of address

        // Chop data down to page-sized packets to be transmitted at a time
        // Maintain pointer of current startaddress
        if(currentSize > MAXPAGEWRITE){
            bufferPtr = bufferPtr + MAXPAGEWRITE;
            counterI2cBuffer = MAXPAGEWRITE - 1;
            PtrTransmit = MAXPAGEWRITE + 1;       // set I2CBufferArray Pointer
            currentSize = currentSize - MAXPAGEWRITE;
            currentAddress = currentAddress + MAXPAGEWRITE;

            // Get start address
            I2CBufferArray[MAXPAGEWRITE + 1] = adr_hi; // High byte address.
            I2CBufferArray[MAXPAGEWRITE] = adr_lo; // Low byte address.
        }else{
            bufferPtr = bufferPtr + currentSize;
            counterI2cBuffer = currentSize - 1;
            PtrTransmit = currentSize + 1;        // set I2CBufferArray Pointer.
            moreDataToRead = 0;
            currentAddress = currentAddress + currentSize;

            // Get start address
            I2CBufferArray[currentSize + 1] = adr_hi; // High byte address.
            I2CBufferArray[currentSize] = adr_lo; // Low byte address.
        }

        // Copy data to I2CBufferArray
        unsigned char temp;
        for(i ; i < bufferPtr ; i++){
            temp = Data[i];                       // Required or else IAR throws a
                                                  // warning [Pa082]
            I2CBufferArray[counterI2cBuffer] = temp;
            counterI2cBuffer--;
        }

        I2CWriteInit();
        I2CWriteInit();
        i2c_transmit_n(bufferPtr, (uint8_t *)&I2CBufferArray);
      
    	//EEPROM_AckPolling();                    // Ensure data is written in EEPROM
    }
}

/*----------------------------------------------------------------------------*/
// Description:
//   Current Address Read Operation. Data is read from the EEPROM. The current
//   address from the EEPROM is used.
/*----------------------------------------------------------------------------*/
unsigned char EEPROM_CurrentAddressRead(void)
{
    while(i2c_busy());                        // wait until I2C module has
                                              // finished all operations
    I2CReadInit();

    UCB0CTL1 |= UCTXSTT;                      // I2C start condition
    while(UCB0CTL1 & UCTXSTT);                // Start condition sent?
    UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
    while(UCB0CTL1 & UCTXSTP);                // Ensure stop condition got sent
    return I2CBuffer;
}

/*----------------------------------------------------------------------------*/
// Description:
//   Random Read Operation. Data is read from the EEPROM. The EEPROM
//   address is defined with the parameter Address.
/*----------------------------------------------------------------------------*/
unsigned char 
EEPROM_RandomRead(unsigned int Address)
{
    unsigned char adr_hi;
    unsigned char adr_lo;

    while (i2c_busy());                       // wait until I2C module has
                                              // finished all operations

    adr_hi = Address >> 8;                    // calculate high byte
    adr_lo = Address & 0xFF;                  // and low byte of address

    I2CBufferArray[1] = adr_hi;               // store single bytes that have to
    I2CBufferArray[0] = adr_lo;               // be sent in the I2CBuffer.
    PtrTransmit = 1;                          // set I2CBufferArray Pointer

    // Write Address first
    I2CWriteInit();
    UCB0CTL1 |= UCTXSTT;                      // start condition generation
                                              // => I2C communication is started
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts

    // Read Data byte
    I2CReadInit();

    UCB0CTL1 |= UCTXSTT;                      // I2C start condition
    while(UCB0CTL1 & UCTXSTT);                // Start condition sent?
    UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
    while(UCB0CTL1 & UCTXSTP);                // Ensure stop condition got sent
    return I2CBuffer;
}

/*----------------------------------------------------------------------------*/
// Description:
//   Sequential Read Operation. Data is read from the EEPROM in a sequential
//   form from the parameter address as a starting point. Specify the size to
//   be read and populate to a Data buffer.
/*----------------------------------------------------------------------------*/
void 
EEPROM_SequentialRead(unsigned int Address , unsigned char * Data , unsigned int Size)
{
    unsigned char adr_hi;
    unsigned char adr_lo;

  	#ifdef PRINTF_DEBUG
  	printf("EEPROM_SequentialRead: reading %d bytes from Address 0x%.2x\r\n", Size, Address);
  	#endif

    while (i2c_busy());                        // wait until I2C module has
                                              // finished all operations

    adr_hi = Address >> 8;                    // calculate high byte
    adr_lo = Address & 0xFF;                  // and low byte of address

    I2CBufferArray[1] = adr_hi;               // store single bytes that have to
    I2CBufferArray[0] = adr_lo;               // be sent in the I2CBuffer.
    PtrTransmit = 1;                          // set I2CBufferArray Pointer

    // Write Address first
    I2CWriteInit();
    i2c_transmit_n(2, (uint8_t *)&I2CBufferArray);
    I2CReadInit();
    i2c_receive_n(Size, (uint8_t *)Data);
}
