
#include "contiki.h"
#include "contiki-net.h"

static struct psock ps;
static char psock_buffer[120];

PROCESS(example_psock_client_process, "Example protosocket client");

//PROCESS(start_process, "Start process");

//AUTOSTART_PROCESSES(&start_process);

/*PROCESS_THREAD(start_process, ev, data)
{
  process_start(&example_psock_client_process, NULL);
}*/

/*---------------------------------------------------------------------------*/
static int
handle_connection(struct psock *p)
{
  PSOCK_BEGIN(p);

  //PSOCK_SEND_STR(p, "GET /helloworld.html HTTP/1.0\r\n");
  //PSOCK_SEND_STR(p, "Server: Contiki example protosocket client\r\n");
  //PSOCK_SEND_STR(p, "\r\n");

  PSOCK_SEND_STR(p, "POST / HTTP/1.1\r\n");
  PSOCK_SEND_STR(p, "Content-Length: 12\r\n");
  PSOCK_SEND_STR(p, "\r\n");
  PSOCK_SEND_STR(p, "Hey, Kirk!\r\n");

  while(1) {
    PSOCK_READTO(p, '\n');
    printf("Got: %s", psock_buffer);
    printf("%s", psock_buffer);
    //memcpy(buffer, 0, 100);
  }
  
  PSOCK_END(p);
}
/*---------------------------------------------------------------------------*/

  //uint8_t = 0;

PROCESS_THREAD(example_psock_client_process, ev, data)
{
  uip_ipaddr_t addr;
  //printf("Starting...\n");
  
  PROCESS_BEGIN();

  //2001:630:d0:f200:212:7400:1465:d8aa
// while(1)
// {
  
  uip_ip6addr(&addr, 0x2001,0x630,0xd0,0xf111,0x224,0xe8ff,0xfe38,0x6cf2);
  tcp_connect(&addr, UIP_HTONS(8080), NULL);

  printf("Connecting...\n");
  PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);

  if(uip_aborted() || uip_timedout() || uip_closed()) {
    printf("Could not establish connection\n");
  } else if(uip_connected()) {
    printf("Connected\n");
    
    PSOCK_INIT(&ps, psock_buffer, sizeof(psock_buffer));

    do {
      handle_connection(&ps);
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    } while(!(uip_closed() || uip_aborted() || uip_timedout()));

    printf("\nConnection closed.\n");
  }
// }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
