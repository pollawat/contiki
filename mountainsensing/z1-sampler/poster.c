
#include "poster.h"



#define POSTDEFBUG
#ifdef POSTDEFBUG
    #define PPRINT(...) printf(__VA_ARGS__)
#else
    #define PPRINT(...)
#endif



PROCESS(post_process, "POST Process");

static POSTConfig POST_config;


static uint8_t psock_buffer[PSOCK_BUFFER_LENGTH];

static struct psock ps;





int
handle_connection(char *data_buffer, uint8_t data_length, uint8_t *http_status, struct psock *p)
{
    char content_length[8], tmpstr_handle[50];

    itoa(data_length, content_length, 10);
    strcpy(tmpstr_handle, "POST / HTTP/1.0\r\nContent-Length: ");
    strcat(tmpstr_handle, content_length);
    strcat(tmpstr_handle, "\r\n\r\n");

    PSOCK_BEGIN(p);

    PSOCK_SEND_STR(p, tmpstr_handle);
    PSOCK_SEND(p, data_buffer, data_length);

    while(1) {
        PSOCK_READTO(p, '\n');
        if(strncmp(psock_buffer, "HTTP/", 5) == 0){   
            // Status line
            *http_status = atoi(psock_buffer + 9);
        }
    }

    PSOCK_END(p);
}

/*---------------------------------------------------------------------------*/

//returns number of bytes loaded

uint8_t 
load_file(char *data_buffer, char *filename)
{
    int fd;
    uint8_t data_length;
#ifdef SPI_LOCKING
    LPRINT("LOCK: load file\n");
    NETSTACK_MAC.off(0);
    cc1120_arch_interrupt_disable();
    CC1120_LOCK_SPI();  
#endif
    fd = cfs_open(filename, CFS_READ);
    if(fd >= 0){
        data_length = cfs_read(fd, data_buffer, DATA_BUFFER_LENGTH);
        cfs_close(fd);
        PPRINT("[LOAD] Read %d bytes from %s\n", data_length, filename);
    }else{
        PPRINT("[LOAD] ERROR: CAN'T READ FILE { %s }\n", filename);
        data_length = 0;
    }
#ifdef SPI_LOCKING
    LPRINT("UNLOCK: load file\n");
    CC1120_RELEASE_SPI();
    cc1120_arch_interrupt_enable();
    NETSTACK_MAC.on();
#endif
    return data_length;
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
        PPRINT("POST config set to default and written\n");
    }else{
        PPRINT("POST Config file loaded\n");
    }
    PPRINT("Refeshed post config to:\n");
    print_comms_config(&POST_config);

}

PROCESS_THREAD(post_process, ev, data)
{

    static uint8_t retries;
    static uip_ipaddr_t addr;
    static char filename[FILENAME_LENGTH];
    static char data_buffer[DATA_BUFFER_LENGTH];
    static uint8_t http_status;
    static uint8_t data_length;
    /* These could be combined if needed to save space */
    static struct etimer post_timer;
    static struct etimer timeout_timer;
    static uip_ds6_addr_t *n_addr;

    PROCESS_BEGIN();
    refreshPosterConfig();
    data_length = 0;
    http_status = 0;
    

    while(1){
        retries = 0;
        etimer_set(&post_timer, CLOCK_SECOND * (POST_config.interval - (get_time() % POST_config.interval)));
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&post_timer));
        n_addr = uip_ds6_get_global(-1);
        if(n_addr == NULL){
            printf("Not associated so can't send\n");
            continue;
        }
        while((get_next_read_filename(filename)) !=0 && retries < CONNECTION_RETRIES){
            uip_ip6addr_u8(&addr,
                n_addr->ipaddr.u8[0], n_addr->ipaddr.u8[1],
                n_addr->ipaddr.u8[2], n_addr->ipaddr.u8[3],
                n_addr->ipaddr.u8[4], n_addr->ipaddr.u8[5],
                n_addr->ipaddr.u8[6], n_addr->ipaddr.u8[7],
                0, 0, 0, 0, 0, 0, 0, 1);
    #ifdef POSTDEFBUG
            printf("About to post to: ");
            uip_debug_ipaddr_print(&addr);
            printf("\n");
    #endif
            PPRINT("[POST][INIT] About to attempt POST with %s - RETRY [%d]\n", filename, retries);
            data_length = load_file(data_buffer, filename);
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
                        handle_connection(data_buffer, data_length, &http_status, &ps);
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
