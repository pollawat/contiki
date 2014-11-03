
#include "poster.h"



//#define POSTDEFBUG
#ifdef POSTDEFBUG
    #define PPRINT(...) printf(__VA_ARGS__)
#else
    #define PPRINT(...)
#endif



PROCESS(post_process, "POST Process");

static POSTConfig POST_config;
static uint8_t data[256] = {0};
static uint16_t data_length = 0;
static struct etimer timer;
static struct psock web_ps;
static const uint8_t web_buf[128];
static char psock_buffer[120];
static struct etimer timeout_timer;
static int http_status = 0;
static struct psock ps;





static int
handle_connection(struct psock *p)
{
  char content_length[8], tmpstr_handle[50];

  itoa(data_length, content_length, 10);
  strcpy(tmpstr_handle, "POST / HTTP/1.0\r\nContent-Length: ");
  strcat(tmpstr_handle, content_length);
  strcat(tmpstr_handle, "\r\n\r\n");

  PSOCK_BEGIN(p);

  PSOCK_SEND_STR(p, tmpstr_handle);
  PSOCK_SEND(p, data, data_length);

  while(1) {
    PSOCK_READTO(p, '\n');
    if(strncmp(psock_buffer, "HTTP/", 5) == 0)
    { // Status line
      http_status = atoi(psock_buffer + 9);
    }
  }

  PSOCK_END(p);
}

/*---------------------------------------------------------------------------*/

static 
void load_file(char *filename)
{
  static int fd;
  fd = cfs_open(filename, CFS_READ);
  if(fd >= 0)
  {
    data_length = cfs_read(fd, data, sizeof(data));
    cfs_close(fd);
    PPRINT("[LOAD] Read %d bytes from %s\n", data_length, filename);
  }
  else
  {
    PPRINT("[LOAD] ERROR: CAN'T READ FILE { %s }\n", filename);
  }
}


void
refreshPosterConfig(void)
{
    if(get_config(&POST_config, COMMS_CONFIG) == 1){ 
        // Config file does not exist! Use default and set file
        POST_config.interval = POST_INTERVAL;
        POST_config.ip_count = POST_IP_COUNT;
        POST_config.ip[0] = POST_IP0;
        POST_config.ip[1] = POST_IP1;
        POST_config.ip[2] = POST_IP2;
        POST_config.ip[3] = POST_IP3;
        POST_config.ip[4] = POST_IP4;
        POST_config.ip[5] = POST_IP5;
        POST_config.ip[6] = POST_IP6;
        POST_config.ip[7] = POST_IP7;
        POST_config.port = POST_PORT;
        set_config(&POST_config, COMMS_CONFIG);
        printf("POST config set to default and written\n");
    }else{
        PPRINT("POST Config file loaded\n");
    }

}

PROCESS_THREAD(post_process, ev, data)
{
  static uint8_t retries;

  static uip_ipaddr_t addr;

  static char filename[FILENAME_LENGTH];
  
  refreshPosterConfig();

  PROCESS_BEGIN();

  
  printf("Post interval set to: %d\n", POST_config.interval);
  while(1){
    retries = 0;
    etimer_set(&timer, CLOCK_SECOND * (POST_config.interval - (get_time() % POST_config.interval)));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    while((get_next_read_filename(filename)) !=0 && retries < CONNECTION_RETRIES){
      uip_ip6addr(&addr,
          POST_config.ip[0], POST_config.ip[1], POST_config.ip[2],
          POST_config.ip[3], POST_config.ip[4], POST_config.ip[5],
          POST_config.ip[6], POST_config.ip[7]);
      PPRINT("[POST][INIT] About to attempt POST with %s - RETRY [%d]\n", filename, retries);
      load_file(filename);
      tcp_connect(&addr, UIP_HTONS(POST_config.port), NULL);
      PPRINT("Connecting...\n");
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
      if(uip_aborted() || uip_timedout() || uip_closed()) {
        PPRINT("Could not establish connection\n");
        retries++;
      } else if(uip_connected()) {
        PPRINT("Connected\n");
        PSOCK_INIT(&ps, psock_buffer, sizeof(psock_buffer));
        etimer_set(&timeout_timer, CLOCK_SECOND*LIVE_CONNECTION_TIMEOUT);
        do {
          if(etimer_expired(&timeout_timer)){
            PPRINT("Connection took too long. TIMEOUT\n");
            PSOCK_CLOSE(&ps);
            retries++;
            break;
          } else if(data_length > 0) {
            PPRINT("[POST] Handle Connection\n");
            handle_connection(&ps);
            PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
          }
        } while(!(uip_closed() || uip_aborted() || uip_timedout()));
        PPRINT("\nConnection closed.\n");
        PPRINT("Status = %d\n", http_status);
        if(http_status/100 == 2) { // Status OK
          data_length = 0;
          retries = 0;
          cfs_remove(filename);
          PPRINT("[POST] Removing file\n");
        } else { // POST failed
          data_length = 0;
          retries++;
          PPRINT("[POST] Failed, not removing file\n");
        }
      }
    }
    PSOCK_CLOSE(&ps);
  }
  PROCESS_END();
}