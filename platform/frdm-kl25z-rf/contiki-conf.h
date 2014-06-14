#ifndef CONTIKI_CONF_H_CDBB4VIH3I__
#define CONTIKI_CONF_H_CDBB4VIH3I__

#include <stdint.h>

#include "platform-conf.h"

//#define WITH_UIP 1
#define WITH_ASCII 1



/* CPU target speed in Hz */
#define F_CPU 48000000uL /* 48MHz by default */

#define CLOCK_CONF_SECOND 100

#define CCIF
#define CLIF

/* the low-level radio driver */
#define NETSTACK_CONF_RADIO   cc1120_driver


/* --------------------------- UIP Configuration. --------------------------- */
#if WITH_UIP6

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver
#define NETSTACK_CONF_MAC     csma_driver
#define NETSTACK_CONF_RDC     contikimac_driver
#define NETSTACK_CONF_RADIO   cc1120_driver
#define NETSTACK_CONF_FRAMER  framer_802154

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD 63
#define CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER 0

#define CC1120_CONF_AUTOACK              1
#define NETSTACK_RDC_CHANNEL_CHECK_RATE  8
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 0
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0

#define QUEUEBUF_CONF_NUM                4 


#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */

#define NETSTACK_CONF_NETWORK rime_driver
#define NETSTACK_CONF_MAC     csma_driver
#define NETSTACK_CONF_RDC     contikimac_driver
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#define NETSTACK_CONF_FRAMER  framer_802154

//#define CC2420_CONF_AUTOACK              1

#define COLLECT_CONF_ANNOUNCEMENTS       1
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 0
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

#define CONTIKIMAC_CONF_COMPOWER         1
#define XMAC_CONF_COMPOWER               1
#define CXMAC_CONF_COMPOWER              1

#define COLLECT_NBR_TABLE_CONF_MAX_NEIGHBORS      32

#define QUEUEBUF_CONF_NUM          8

#endif /* WITH_UIP6 */

#define PACKETBUF_CONF_ATTRS_INLINE 1

#ifndef RF_CHANNEL
#define RF_CHANNEL              42
#endif /* RF_CHANNEL */

#define IEEE802154_CONF_PANID       0xABCD

#define SHELL_VARS_CONF_RAM_BEGIN 0x1100
#define SHELL_VARS_CONF_RAM_END 0x2000

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1

#ifdef WITH_UIP6

#define RIMEADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_ROUTER                 1
#define UIP_CONF_IPV6_RPL               1

/* Handle 10 neighbors */
#define NBR_TABLE_CONF_MAX_NEIGHBORS     15
/* Handle 10 routes    */
#define UIP_CONF_MAX_ROUTES   15

#define UIP_CONF_ND6_SEND_RA		0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#define UIP_CONF_IPV6                   1
#define UIP_CONF_IPV6_QUEUE_PKT         0
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0
#define UIP_CONF_BUFFER_SIZE		140

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC	1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#else /* WITH_UIP6 */
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     108
#endif /* WITH_UIP6 */

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_RECEIVE_WINDOW  48
#define UIP_CONF_TCP_MSS         48
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#define UIP_CONF_TCP_SPLIT       0


//#define CH_CURS_DOWN              -4
//#define CH_CURS_UP                -1

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

#define RAND_MAX 0x7fff
#endif /* CONTIKI_CONF_H_CDBB4VIH3I__ */
