/*
 * -----------------------------------------------------------------
 *
 * Author  : Philip Basford <pjb@ecs.soton.ac.uk>
 * Created : 2015-03-09
 */
#ifndef SHT25_H
#define SHT25_H

#include <stdint.h>

 #define SHT25_ADDR                 0x40 // 1000 000 (7 bits)
 #define SHT25_MEASURE_T_HOLD       0xE3 //11100011
 #define SHT25_MEASURE_RH_HOLD      0xE5 //11100101
 #define SHT_25_MEASURE_T_NO_HOLD   0xF3 //11110011
 #define SHT_25_MEASURE_TH_NO_HOLD  0xF5 //11110101
 #define SHT_25_WRITE_REGISTER      0xE6 //11100110
 #define SHT_25_READ_REGISTER       0xE7 //11100111
 #define SHT_25_SOFT_RESET          0xFE //11111110

 #define TX_BUF_SIZE    5
 #define RX_BUF_SIZE    10

void sht25_init(void);
void sht25_off(void);

unsigned int sht25_temp(void);
unsigned int sht25_humidity(void);

float SHT2x_CalcTemperatureC(uint16_t u16sT);
float SHT2x_CalcRH(uint16_t u16sRH);
uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);


#endif