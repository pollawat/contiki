/*
 * -----------------------------------------------------------------
 * Based on sample code from http://www.sensirion.com/en/products/humidity-temperature/download-center/
 *
 * Author  : Philip Basford <pjb@ecs.soton.ac.uk>
 * Created : 2015-03-09
 */
#include "contiki.h"
#include <dev/sht25.h>
#include "sht25-arch.h"


//#define SHT_DEBUG
#ifdef SHT_DEBUG
    #include <stdio.h>
    #define PRINTF(...) printf(__VA_ARGS__)
#else
    #define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
void 
sht25_init(void){
    PRINTF("Starting SHT25 init\n");
    I2C_ENABLE();
    PRINTF("SHT25 init complete\n");
}
/*---------------------------------------------------------------------------*/
void 
sht25_off(void){

}
/*---------------------------------------------------------------------------*/
unsigned int 
sht25_temp(void){
    uint8_t tx_buf[TX_BUF_SIZE];
    uint8_t rx_buf[RX_BUF_SIZE];
 
    uint8_t i;
    uint8_t recvd;

    I2C_TRANSMIT_INIT(SHT25_ADDR);
    tx_buf[0] = SHT25_MEASURE_T_HOLD;
    I2C_TRANSMIT_N(1, &tx_buf);
    while(I2C_BUSY());
    I2C_RECEIVE_INIT(SHT25_ADDR);
    while(I2C_BUSY());
    recvd = I2C_RECEIVE_N(3, &rx_buf[0]);
    PRINTF("Recieved %d bytes\n", recvd);
#ifdef SHT_DEBUG    
    for(i = 0; i < 3; i++){
        printf("%x:", rx_buf[i]);
    }
    printf("\n");
#endif
    if(SHT2x_CheckCrc(rx_buf, 2, rx_buf[2])){
        PRINTF("Returning %lu\n", ((long unsigned)rx_buf[0] << 8) + rx_buf[1]);
        return ((uint16_t)rx_buf[0] << 8) + rx_buf[1];
    }else{
        return -9999;
    }
}
/*---------------------------------------------------------------------------*/
unsigned int
sht25_humidity(void){
    uint8_t tx_buf[TX_BUF_SIZE];
    uint8_t rx_buf[RX_BUF_SIZE];

    uint8_t i;
    uint8_t recvd;

    I2C_TRANSMIT_INIT(SHT25_ADDR);
    tx_buf[0] = SHT25_MEASURE_RH_HOLD;
    I2C_TRANSMIT_N(1, &tx_buf);
    while(I2C_BUSY());
    I2C_RECEIVE_INIT(SHT25_ADDR);
    while(I2C_BUSY());
    recvd = I2C_RECEIVE_N(3, &rx_buf[0]);
    PRINTF("Recieved %d bytes\n", recvd);
#ifdef SHT_DEBUG    
    for(i = 0; i < 3; i++){
        printf("%x:", rx_buf[i]);
    }
    printf("\n");
#endif
    if(SHT2x_CheckCrc(rx_buf, 2, rx_buf[2])){
        PRINTF("Returning %lu\n", ((long unsigned)rx_buf[0] << 8) + rx_buf[1]);
        return ((uint16_t)rx_buf[0] << 8) + rx_buf[1];
    }else{
        return -9999;
    }
}

/*---------------------------------------------------------------------------*/
float 
SHT2x_CalcRH(uint16_t u16sRH)
{
    float humidityRH;              // variable for result

    u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --

    humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    return humidityRH;
}

/*---------------------------------------------------------------------------*/
float 
SHT2x_CalcTemperatureC(uint16_t u16sT)
{
    float temperatureC;            // variable for result

    u16sT &= ~0x0003;           // clear bits [1..0] (status bits)

    //-- calculate temperature [°C] --
    temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
    return temperatureC;
}
/*---------------------------------------------------------------------------*/

uint8_t 
SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
    uint8_t crc;
    crc = 0;  
    uint8_t byteCtr;
    uint8_t bit;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr){
        crc ^= (data[byteCtr]);
        for (bit = 8; bit > 0; --bit){
            if (crc & 0x80){
                crc = (crc << 1) ^ 0x131;
            }else{
                crc = (crc << 1);
            }
        }
    }
    if (crc != checksum){
        PRINTF("CHECKSUM FAILURE %d != %d\n", crc, checksum);
        return 0;
    }else{
        PRINTF("CHECKSUM OK\n");
        return 1;
    }
}