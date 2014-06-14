#include <stdint.h>
#include <stdio.h>
#include <stdarg.h> 

#include <contiki.h>

#include <clock.h>
#include <etimer.h>

//#include <dev/leds.h>

#include <sys/process.h>
#include <sys/procinit.h>
#include <sys/autostart.h>
#include <sys/profile.h>
#include <sys/profile.h>

#include <lib/random.h>

#include <net/mac/frame802154.h>
#include <net/rime.h>

#include <MKL25Z4.h>

#include "nvic.h"
#include "debug-uart.h"
#include "cpu.h"

#include "cc1120.h"
#include "cc1120-arch.h"

#if WITH_UIP6
#include <net/uip-ds6.h>
#else
#include <net/rime.h>
#endif /* WITH_UIP6 */


#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


unsigned int idle_count = 0;
unsigned short node_id = 0;

int
main(void)
{
  cpu_init();
  
  port_enable(PORTB_EN_MASK | PORTC_EN_MASK | PORTD_EN_MASK | PORTE_EN_MASK);
  
  dbg_setup_uart();
  printf("Initialising\n");
  
  clock_init();
  
  
  process_init();
  process_start(&etimer_process, NULL);






  NETSTACK_RADIO.init();
  
  
  
  


  PRINTF(CONTIKI_VERSION_STRING " started. ");

  if(node_id > 0) {
    PRINTF("Node id is set to %u.\n", node_id);
  } else {
    PRINTF("Node id is not set.\n");
  }

#if WITH_UIP6
  memcpy(&uip_lladdr.addr, node_mac, sizeof(uip_lladdr.addr));
  /* Setup nullmac-like MAC for 802.15.4 */
/*   sicslowpan_init(sicslowmac_init(&cc2420_driver)); */
/*   printf(" %s channel %u\n", sicslowmac_driver.name, RF_CHANNEL); */

  /* Setup X-MAC for 802.15.4 */
  queuebuf_init();

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);

  process_start(&tcpip_process, NULL);

  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }
  
  if(!UIP_CONF_IPV6_RPL) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }

#else /* WITH_UIP6 */

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);
#endif /* WITH_UIP6 */








  autostart_start(autostart_processes);
  printf("Processes running\n");
  
  while(1) {
    do {
    } while(process_run() > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
  }
  return 0;
}




