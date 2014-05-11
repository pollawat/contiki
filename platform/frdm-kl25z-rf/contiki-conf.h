#ifndef CONTIKI_CONF_H_CDBB4VIH3I__
#define CONTIKI_CONF_H_CDBB4VIH3I__

#include <stdint.h>

#define CCIF
#define CLIF

//#define WITH_UIP 1
#define WITH_ASCII 1

#define PLATFORM_HAS_LEDS   0
#define PLATFORM_HAS_BUTTON 0

#define NETSTACK_CONF_RADIO   cc11xx_driver
#define NETSTACK_CONF_NETWORK     sicslowpan_driver
#define NETSTACK_RADIO_MAX_PAYLOAD_LEN 125
#define NETSTACK_CONF_FRAMER  framer_802154
#define NETSTACK_CONF_MAC     csma_driver
#define NETSTACK_CONF_RDC     contikimac_driver

#define RIMESTATS_CONF_ENABLED                      1
#define RIMESTATS_CONF_ON 
#define RIMEADDR_CONF_SIZE              8
#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        0
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#define UIP_CONF_TCP_SPLIT       0

#define CH_CURS_DOWN              -4
#define CH_CURS_UP                -1

#define CLOCK_CONF_SECOND 100

/* These names are deprecated, use C99 names. */
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

#define CC11xx_ARCH_SPI_ENABLE  cc1120_arch_spi_enable
#define CC11xx_ARCH_SPI_DISABLE cc1120_arch_spi_disable
#define CC11xx_ARCH_SPI_RW_BYTE cc1120_arch_spi_rw_byte
#define CC11xx_ARCH_SPI_RW      cc1120_arch_spi_rw

#define cc11xx_arch_spi_enable  cc1120_arch_spi_enable
#define cc11xx_arch_spi_disable cc1120_arch_spi_disable
#define cc11xx_arch_spi_rw_byte cc1120_arch_spi_rw_byte
#define cc11xx_arch_spi_rw      cc1120_arch_spi_rw
#define cc11xx_arch_interrupt_enable cc1120_arch_interrupt_enable

#define cc11xx_arch_init        cc1120_arch_init

/* uIP configuration */
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_BROADCAST       1
#define UIP_CONF_LOGGING 1
#define UIP_CONF_BUFFER_SIZE 116

#define UIP_CONF_TCP_FORWARD 1

/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

#define USB_EP1_SIZE 64
#define USB_EP2_SIZE 64

#define RAND_MAX 0x7fff
#endif /* CONTIKI_CONF_H_CDBB4VIH3I__ */
