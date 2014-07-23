/*
 * Extended from z1-websense.c
 */

// General
#include "contiki.h"
#include <stdio.h>
#include <stdlib.h>
#include "cfs/cfs.h"

// Networking
#include "httpd-simple.h"
#include "webserver-nogui.h"

// Protobuf
#include "pb_decode.h"
#include "pb_decode.c"
#include "pb_encode.h"
#include "pb_encode.c"
#include "message.pb.h"
#include "message.pb.c"

// Sensors
#include "dev/ds3231-sensor.h" // Clock

#define LIVE_CONNECTION_TIMEOUT 300

//#define READINGS_FILENAME "readings"
//#define CONFIG_FILENAME "config"

uint32_t comms_interval = 60;
uint32_t sample_interval = 1;

/*float floor(float x){
  if(x>=0.0f) return (float) ((int)x);
  else        return (float) ((int)x-1);
}*/

PROCESS(web_process, "Sense Web Demo");
PROCESS(sample_process, "Sample Process");
PROCESS(post_process, "Post Process");

AUTOSTART_PROCESSES(&web_process, &post_process, &sample_process);//, &tick_process);//, &post_process);

static const char *TOP = "<html><head><title>Sensor Node</title></head><body>\n";
static const char *BOTTOM = "</body></html>\n";

static uint32_t get_time()
{
  uint32_t time = (uint32_t)ds3231_sensor.value(DS3231_SENSOR_GET_EPOCH_SECONDS_MSB) << 16;
  time += (uint32_t)ds3231_sensor.value(DS3231_SENSOR_GET_EPOCH_SECONDS_LSB);
  return(time);
}

static char cfg_buf[128];
/*
 * Returns an `Example` struct containing the un-protobuf'd value in the "config" file
 */
Example get_config()
{
  memset(cfg_buf, 0, sizeof(cfg_buf));
  int read = cfs_open("config", CFS_READ);
  Example config;
  if(read != -1) {
    cfs_read(read, cfg_buf, sizeof(cfg_buf));
    cfs_close(read);
    pb_istream_t istream = pb_istream_from_buffer(cfg_buf, sizeof(cfg_buf));
    pb_decode(&istream, Example_fields, &config);
  } else {
    printf("ERROR: could not read from disk\n");
  }
  return config;
}

/*
 * Writes the protobuf'd value of an `Example` to the "config" file
 */
void set_config(Example config)
{
  memset(cfg_buf, 0, sizeof(cfg_buf));
  pb_ostream_t ostream = pb_ostream_from_buffer(cfg_buf, sizeof(cfg_buf));
  pb_encode(&ostream, Example_fields, &config);
  cfs_remove("config");
  int write = cfs_open("config", CFS_WRITE);
  if(write != -1) {
    cfs_write(write, cfg_buf, ostream.bytes_written);
    cfs_close(write);
  } else {
    printf("ERROR: could not write to disk\n");
  }
}

/*
 * Gets the value associated with the key in a URL
 *
 * Example:
 *   url = "http://ecs.soton.ac.uk?name=djap1g11&login=false"
 *   key = "name"
 *
 *   Returns "djap1g11"
 */
char* get_url_param(char* url, char* key)
{
  char* pch;
  uint8_t len = strlen(key);
  pch = strtok(url, "?&");
  while(pch != NULL) {
    if(strncmp(pch, key, len) == 0) {
      // If the token is key-value pair desired
      char* val = pch + len + 1;
      return val;
    }
    pch = strtok(NULL, "?&");
  }
  return NULL;
}

/* Only one single request at time */
static char buf[256];
static int blen;
#define ADD(...) do {                                                   \
    blen += snprintf(&buf[blen], sizeof(buf) - blen, __VA_ARGS__);      \
  } while(0)

static char form_html[180];

static char value[10];

static
PT_THREAD(send_values(struct httpd_state *s))
{
  PSOCK_BEGIN(&s->sout);

  SEND_STRING(&s->sout, TOP);

  printf("Request = %s\n", s->filename + 1);

  if(strncmp(s->filename, "/setup", 6) == 0) {
    Example cfg = get_config();
    //char form_html[180];
    memset(form_html, 0, 180);
    printf("Value stored in config = %d\n", cfg.value);
    sprintf(form_html,
        "<form name=\"config\" action=\"submit\" method=\"get\">"
        "Value: <input type=\"text\" name=\"value\" value=\"%d\">"
        "<input type=\"submit\" value=\"Submit\">"
        "</form>",
        cfg.value);
    printf("Value sending to client = %s\n", form_html);
    SEND_STRING(&s->sout, form_html);
  } else if(strncmp(s->filename, "/submit", 7) == 0) {
    strcpy(value, get_url_param(s->filename, "value"));
    Example cfg = {atol(value)};
    set_config(cfg);
    SEND_STRING(&s->sout, "Set value to ");
    SEND_STRING(&s->sout, value);
  }
  if(strncmp(s->filename, "/index", 6) == 0 ||
     s->filename[1] == '\0' || s->filename[1]=='i') {
    blen = 0;
    SEND_STRING(&s->sout, buf);
  }
  SEND_STRING(&s->sout, BOTTOM);

  PSOCK_END(&s->sout);
}

httpd_simple_script_t httpd_simple_get_script(const char* name)
{
  return send_values;
}

static struct psock ps;
//static uint8_t sock_buffer[120];

static uint8_t data[128] = {};//{'h','e','l','l','o',' ','w','o','r','l','d'};
static uint16_t data_length = 1;

static struct etimer timer;
static struct etimer timeout_timer;

static int status = 0;

static uint8_t attempting = 0;
static char psock_buffer[120];

static int handles = 0;

static handle_connection(struct psock *p)
{
  static uint8_t status_code[4];
  static char content_length[8];

  itoa(data_length, content_length, 10);

  PSOCK_BEGIN(p);

  PSOCK_SEND_STR(p, "POST / HTTP/1.0\r\n");
  PSOCK_SEND_STR(p, "Content-Length: ");
  PSOCK_SEND_STR(p, content_length);
  PSOCK_SEND_STR(p, "\r\n\r\n");
  PSOCK_SEND(p, data, data_length);

  while(1) {
    PSOCK_READTO(p, '\n');
    //printf("RX: %s\n", psock_buffer);
    if(strncmp(psock_buffer, "HTTP/", 5) == 0)
    { // Status line
      memcpy(status_code, psock_buffer + 9, 3);
      status = atoi(psock_buffer + 9);
    }
  }

  PSOCK_END(p);
}

PROCESS_THREAD(web_process, ev, data)
{
  PROCESS_BEGIN();
  process_start(&webserver_nogui_process, NULL);
  PROCESS_END();
}

PROCESS_THREAD(sample_process, ev, data)
{
  PROCESS_BEGIN();

  static struct etimer stimer;

//  static float test = 100;

  while(1)
  {
    etimer_set(&stimer, CLOCK_SECOND*sample_interval);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&stimer));
//    test *= 1.5;
//    printf("Test value ~= %d\n", (int)test);
  }

  PROCESS_END();
}

PROCESS_THREAD(post_process, ev, data)
{
  PROCESS_BEGIN();

  static uip_ipaddr_t addr;
//  static int fd;
  printf("Starting tick process...\n");
  while(1)
  {
    /*fd = cfs_open("readings", CFS_READ);
    if(fd != -1)
    {
      printf("Length of `readings` file is %d bytes\n", cfs_seek(fd, 0, CFS_SEEK_END));
    }
    else
    {
      printf("Could not open `readings`\n");
    }*/

    printf("Setting timer\n");
    etimer_set(&timer, CLOCK_SECOND * (comms_interval - (get_time() % comms_interval)));
    printf("Waiting %d seconds...\n", (comms_interval - (get_time() % comms_interval)));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    printf("Setting IP address\n");
    uip_ip6addr(&addr, 0x2001,0x630,0xd0,0xf111,0x224,0xe8ff,0xfe38,0x6cf2);
    printf("TCP connecting...\n");
    tcp_connect(&addr, UIP_HTONS(8081), NULL);
    printf("Connecting...\n");
    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    printf("Event fired\n");
    if(uip_aborted() || uip_timedout() || uip_closed()) {
      printf("Could not establish connection\n");
    }
    else if(uip_connected()) {
      printf("Connected\n");
      PSOCK_INIT(&ps, psock_buffer, sizeof(psock_buffer));
      etimer_set(&timeout_timer, CLOCK_SECOND*LIVE_CONNECTION_TIMEOUT);
      do {
        if(etimer_expired(&timeout_timer))
        {
          printf("Connection took too long. TIMEOUT\n");
          PSOCK_CLOSE(&ps);
          break;
        }
        else
        {
          handle_connection(&ps);
          PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
        }
      } while(!(uip_closed() || uip_aborted() || uip_timedout()));
      printf("\nConnection closed.\n");
      printf("Status = %d\n", status);
      if(status/100 == 2)
      {
        // Status OK
        // TODO: REMOVE FILE
      }
      else
      {
        // POST failed
      }
    }

    PSOCK_CLOSE(&ps);

    //etimer_reset(&timer);
  }
  PROCESS_END();
}
