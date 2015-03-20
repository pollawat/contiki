/*
 * -----------------------------------------------------------------
 *
 * Author  : Philip Basford <pjb@ecs.soton.ac.uk>
 * Created : 2015-03-09
 */

#ifndef SHT25_ARCH_H
#define SHT25_ARCH_H

 #include "dev/i2cmaster.h"

#define I2C_ENABLE(void) \
    i2c_enable()

#define I2C_DISABLE(void) \
     i2c_disable()

#define I2C_RECEIVE_INIT(slave_address) \
    i2c_receiveinit((uint8_t)slave_address);

#define I2C_RECEIVE_N(byte_ctr, rx_buf) \
     i2c_receive_n((uint8_t)byte_ctr, (uint8_t *)rx_buf)

#define I2C_TRANSMIT_INIT(slave_address) \
    i2c_transmitinit((uint8_t)slave_address)
#define I2C_TRANSMIT_N(byte_ctr, tx_buf) \
    i2c_transmit_n((uint8_t)byte_ctr, (uint8_t *)tx_buf);

#define I2C_BUSY(void) \
    i2c_busy()

#endif