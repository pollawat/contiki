/*
 * Based on Z1-Websense, which has the following licence:
 *
 * Copyright (c) 2011, Zolertia(TM) is a trademark by Advancare,SL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *         Battery and Temperature IPv6 Demo for Zolertia Z1
 * \author
 *         Niclas Finne    <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Joel Hoglund    <joel@sics.se>
 *         Enric M. Calvo  <ecalvo@zolertia.com>
 */

#include "contiki.h"
#include "httpd-simple.h"
#include "webserver-nogui.h"
#include "dev/temperature-sensor.h"
#include "dev/battery-sensor.h"

#ifndef CC11xx_CC1120
#include "dev/cc2420.h"
#endif

#include "dev/leds.h"
#include <stdio.h>

#include "z1-sampler-config-defaults.h"
#include "web_defines.h"

// General
#include "contiki.h"
#include <stdio.h>
#include <stdlib.h>
#include "cfs/cfs.h"

// Networking
#include "contiki-net.h"

// Protobuf
#include "pb_decode.h"
#include "pb_encode.h"

// Config
#include "settings.pb.h"
#include "readings.pb.h"

// Sensors
#include "sampling-sensors.h"

#define MAX_POST_SIZE 30

#define DEBUG 1

#if DEBUG == 1
    #define DPRINT(...) printf(__VA_ARGS__)
#else
    #define DPRINT(...)
#endif

#define LIVE_CONNECTION_TIMEOUT 300
#define CONNECTION_RETRIES 3

float floor(float x){ 
  if(x>=0.0f) return (float) ((int)x);
  else        return (float) ((int)x-1);
}

PROCESS(web_sense_process, "Sense Web Demo");
PROCESS(web_process, "Web Server Process");
PROCESS(sample_process, "Sample Process");
PROCESS(post_process, "POST Process");

AUTOSTART_PROCESSES(&web_sense_process);

/*---------------------------------------------------------------------------*/
// CONFIG CODE 

static SensorConfig sensor_config;
static POSTConfig POST_config;

#define SAMPLE_CONFIG 1
#define COMMS_CONFIG 2

#if SensorConfig_size > PostConfig_size
    static char cfg_buf[SensorConfig_size + 4];
#else
    static char cfg_buf[POSTConfig_size + 4];
#endif

/*
 * Returns 0 upon success, 1 on failure
 */
uint8_t set_config(uint8_t config)
{
  memset(cfg_buf, 0, sizeof(cfg_buf));
  static pb_ostream_t ostream;
  ostream = pb_ostream_from_buffer(cfg_buf, sizeof(cfg_buf));
  static int write;
  if(config == SAMPLE_CONFIG) {
    pb_encode_delimited(&ostream, SensorConfig_fields, &sensor_config);
    cfs_remove("sampleconfig");
    write = cfs_open("sampleconfig", CFS_WRITE);
  } else {
    pb_encode_delimited(&ostream, POSTConfig_fields, &POST_config);
    cfs_remove("commsconfig");
    write = cfs_open("commsconfig", CFS_WRITE);
  }
  if(write != -1) {
    cfs_write(write, cfg_buf, ostream.bytes_written);
    cfs_close(write);
    DPRINT("[WCFG] Writing %d bytes to config file\n", ostream.bytes_written);
    return 0;
  } else {
    DPRINT("[WCFG] ERROR: could not write to disk\n");
    return 1;
  }
}

/*
 * Returns 0 upon success, 1 upon failure
 */
static uint8_t get_config(uint8_t config)
{
  memset(cfg_buf, 0, sizeof(cfg_buf));
  static int read;
  if(config == SAMPLE_CONFIG) {
    DPRINT("[RCFG] Opening `sampleconfig`\n");
    read = cfs_open("sampleconfig", CFS_READ);
    DPRINT("[RCFG] Opened\n");
  } else {
    DPRINT("[RCFG] Opening `commsconfig`\n");
    read = cfs_open("commsconfig", CFS_READ);
  }
  DPRINT("[RCFG] Attmepting to read\n");
  //static Example config;
  if(read != -1) {
    DPRINT("[RCFG] Reading...\n");
    cfs_read(read, cfg_buf, sizeof(cfg_buf));
    cfs_close(read);

    pb_istream_t istream = pb_istream_from_buffer(cfg_buf, sizeof(cfg_buf));

    DPRINT("[RCFG] Bytes left = %d\n", istream.bytes_left);

    if(config == SAMPLE_CONFIG) {
      pb_decode_delimited(&istream, SensorConfig_fields, &sensor_config);
    } else {
      pb_decode_delimited(&istream, POSTConfig_fields, &POST_config);
    }
    return 0;
  } else {
    DPRINT("[RCFG] ERROR: could not read from disk\n");
    return 1;
  }
}

/*---------------------------------------------------------------------------*/

static uint8_t data[256] = {0};
static uint16_t data_length = 0;

static load_file(char *filename)
{
  static int fd;
  fd = cfs_open(filename, CFS_READ);
  if(fd >= 0)
  {
    data_length = cfs_read(fd, data, sizeof(data));
    cfs_close(fd);
    DPRINT("[LOAD] Read %d bytes from %s\n", data_length, filename);
  }
  else
  {
    DPRINT("[LOAD] ERROR: CAN'T READ FILE { %s }\n", filename);
  }
}

/*---------------------------------------------------------------------------*/
// 

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
  static char str[100];
  strcpy(str, url);
  static char* pch;
  static uint8_t len;
  len = strlen(key);
  static char* val;
  pch = strtok(str, "?&");
  while(pch != NULL) {
    if(strncmp(pch, key, len) == 0) {
      // If the token is key-value pair desired
      val = pch + len + 1;
      return val;
    }
    pch = strtok(NULL, "?&");
  }
  return NULL;
}

static struct psock ps;

static struct etimer timer;
static struct etimer timeout_timer;

static int http_status = 0;

static uint8_t attempting = 0;
static char psock_buffer[120];

static struct psock web_ps;
static const uint8_t web_buf[128];
static char *url;

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
    if(strncmp(psock_buffer, "HTTP/", 5) == 0)
    { // Status line
      memcpy(status_code, psock_buffer + 9, 3);
      http_status = atoi(psock_buffer + 9);
    }
  }

  PSOCK_END(p);
}




static
PT_THREAD(web_handle_connection(struct psock *p))
{

  static uint8_t i;
  static char* param;

  DPRINT("[WEBD] Reading HTTP request line...\n");

  PSOCK_BEGIN(p);
  PSOCK_READTO(p, '\n');

  if(strncmp("GET ", web_buf, 4) == 0)
  {
    url = web_buf + 4;
    strtok(url, " ");
    DPRINT("[WEBD] Got request for %s\n", url);
    static char num[8];
    if(strncmp(url, "/clock", 6) == 0)
    { // Serve clock form
      static uint16_t y;
      static uint8_t mo, d, h, mi, se;
      static bool submitted;
      static const char* ZERO = "0";
      DPRINT("[WEBD] Serving /clock\n");
      submitted = 0;
      if(get_url_param(url, "submit") != NULL) {
        submitted = 1;
        param = get_url_param(url, "y");
        y = atol(param == NULL ? ZERO : param);
        param = get_url_param(url, "mo");
        mo = atoi(param == NULL ? ZERO : param);
        param = get_url_param(url, "d");
        d = atoi(param == NULL ? ZERO : param);
        param = get_url_param(url, "h");
        h = atoi(param == NULL ? ZERO : param);
        param = get_url_param(url, "mi");
        mi = atoi(param == NULL ? ZERO : param);
        param = get_url_param(url, "s");
        se = atoi(param == NULL? ZERO : param);
        set_time(y, mo, d, h, mi, se);
      }
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      if(submitted) {
        PSOCK_SEND_STR(p, "<h1>Success! Time set</h1>");
      }
      PSOCK_SEND_STR(p, CLOCK_FORM);
      PSOCK_SEND_STR(p, BOTTOM);
    }
    else if(strncmp(url, "/sample", 7) == 0)
    {
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      PSOCK_SEND_STR(p, SENSOR_FORM_1);
      //Interval
      ltoa(sensor_config.interval, num, 10);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, SENSOR_FORM_2);
      //AVR IDs
      DPRINT("[WEBD] Producing AVR IDs from config\n");
      for(i = 0; i < sensor_config.avrIDs_count; i++) {
        DPRINT(".");
        itoa(sensor_config.avrIDs[i], num, 10);
        PSOCK_SEND_STR(p, num);
        if(i != sensor_config.avrIDs_count - 1) {
          PSOCK_SEND_STR(p, ".");
        }
      }
      DPRINT("\n[WEBD] Producing other data\n");
      PSOCK_SEND_STR(p, SENSOR_FORM_3);
      //Rain?
      if(sensor_config.hasRain) {
        PSOCK_SEND_STR(p, " checked");
      }
      DPRINT("  RAIN DONE\n");
      PSOCK_SEND_STR(p, SENSOR_FORM_4);
      //ADC1?
      if(sensor_config.hasADC1) {
        PSOCK_SEND_STR(p, " checked");
      }
      DPRINT("  ADC1 DONE\n");
      PSOCK_SEND_STR(p, SENSOR_FORM_5);
      //ADC2?
      if(sensor_config.hasADC2) {
        PSOCK_SEND_STR(p, " checked");
      }
      DPRINT("  ADC2 DONE\n");
      PSOCK_SEND_STR(p, SENSOR_FORM_6);
      PSOCK_SEND_STR(p, BOTTOM);
      DPRINT("[WEBD] Closing connection.\n");
    }
    else if(strncmp(url, "/sensub", 7) == 0)
    {
      param = get_url_param(url, "sample");
      sensor_config.interval = (param == NULL ? 900 : atol(param));

      param = get_url_param(url, "AVR");
      static char AVRs[32];
      if(param != NULL)
      {
        strcpy(AVRs, param);
        static char *pch;
        pch = strtok(AVRs, ".");
        i = 0;
        while(pch != NULL) {
          sensor_config.avrIDs[i++] = atoi(pch);
          pch = strtok(NULL, ".");
        }
        sensor_config.avrIDs_count = i;
      }

      param = get_url_param(url, "rain");
      if(param != NULL && strcmp(param, "y") == 0) {
        sensor_config.hasRain = 1;
      } else {
        sensor_config.hasRain = 0;
      }

      param = get_url_param(url, "adc1");
      if(param != NULL && strcmp(param, "y") == 0) {
        sensor_config.hasADC1 = 1;
      } else {
        sensor_config.hasADC1 = 0;
      }

      param = get_url_param(url, "adc2");
      if(param != NULL && strcmp(param, "y") == 0) {
        sensor_config.hasADC2 = 1;
      } else {
        sensor_config.hasADC2 = 0;
      }

      set_config(SAMPLE_CONFIG);

      DPRINT("[WEBD] Stored Data\n");
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      PSOCK_SEND_STR(p, "<h1>Success</h1>");
      PSOCK_SEND_STR(p, BOTTOM);
    }
    else if(strncmp(url, "/comms", 6) == 0)
    {
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      PSOCK_SEND_STR(p, COMMS_FORM_1);
      // Interval
      ltoa(POST_config.interval, num, 10);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2A);
      ltoa(POST_config.ip[0], num, 16);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2B);
      ltoa(POST_config.ip[1], num, 16);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2C);
      ltoa(POST_config.ip[2], num, 16);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2D);
      ltoa(POST_config.ip[3], num, 16);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2E);
      ltoa(POST_config.ip[4], num, 16);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2F);
      ltoa(POST_config.ip[5], num, 16);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2G);
      ltoa(POST_config.ip[6], num, 16);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_2H);
      ltoa(POST_config.ip[7], num, 16);
      PSOCK_SEND_STR(p, num);

      PSOCK_SEND_STR(p, COMMS_FORM_3);

      ltoa(POST_config.port, num, 10);
      PSOCK_SEND_STR(p, num);
      PSOCK_SEND_STR(p, COMMS_FORM_4);

      PSOCK_SEND_STR(p, BOTTOM);
    }
    else if(strncmp(url, "/comsub", 7) == 0)
    {
      param = get_url_param(url, "interval");
      POST_config.interval = (param == NULL ? POST_INTERVAL : atol(param));

      static char chr[2] = {'a',0};

      for(i = 0; i < 8; i++) {
        chr[0] = 'a' + i;
        param = get_url_param(url, chr);
        POST_config.ip[i] = (param == NULL ? 0 : strtol(param, NULL, 16));
      }

      param = get_url_param(url, "port");
      POST_config.port = (param == NULL ? POST_PORT : atol(param));

      set_config(COMMS_CONFIG);

      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      PSOCK_SEND_STR(p, "<h1>Success!</h1>");
      PSOCK_SEND_STR(p, BOTTOM);
    }
    else
    {
      DPRINT("Serving / \"INDEX\"\n");
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      PSOCK_SEND_STR(p, INDEX_BODY);
      PSOCK_SEND_STR(p, BOTTOM);
    }
  }

  PSOCK_CLOSE(p);
  PSOCK_END(p);
}

PROCESS_THREAD(web_process, ev, data)
{
  PROCESS_BEGIN();

  tcp_listen(UIP_HTONS(80));
  while(1)
  {
    DPRINT("[WEBD] Now listening for connections\n");
    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    if(uip_connected())
    {
      DPRINT("[WEBD] Connected!\n");
      PSOCK_INIT(&web_ps, web_buf, sizeof(web_buf));
      while(!(uip_aborted() || uip_closed() || uip_timedout()))
      {
        DPRINT("[WEBD] Waiting for TCP Event\n");
        PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
        DPRINT("[WEBD] Handle connection\n");
        web_handle_connection(&web_ps);
        DPRINT("[WEBD] Handle COMPLETE!\n");
      }
    }
  }
  PROCESS_END();
}

PROCESS_THREAD(web_sense_process, ev, data)
{
  static struct etimer timer;
  PROCESS_BEGIN();
  #ifndef CC11xx_CC1120
  cc2420_set_txpower(31);
  #endif

  process_start(&web_process, NULL);
  process_start(&sample_process, NULL);
  process_start(&post_process, NULL);


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*
 * Returns the filename that is to be read for POSTing
 * Returns NULL if no files can be POSTed
 */
static char* get_next_read_filename()
{
  static char filename[8];
  static struct cfs_dirent dirent;
  static struct cfs_dir dir;

  if(cfs_opendir(&dir, "/") == 0) {
    while(cfs_readdir(&dir, &dirent) != -1) {
      if(strncmp(dirent.name, "r_", 2) == 0) {
        strcpy(filename, dirent.name);
        return filename;
      }
    }
  }
  DPRINT("[NEXT] No file found. NULL\n");
  return NULL;
}

/*
 * Returns the filename of the next file where it is safe to store `length` bytes of data
 */
static char* get_next_write_filename(uint8_t length)
{
  static char filename[8];
  static struct cfs_dirent dirent;
  static struct cfs_dir dir;
  static uint8_t file_num, i;
  static uint16_t file_size;
  static int16_t max_num;
  file_num = 0;
  max_num = -1;
  file_size = 0;

  filename[0] = 'r';
  filename[1] = '_';

  if(cfs_opendir(&dir, "/") == 0) {
    while(cfs_readdir(&dir, &dirent) != -1) {
      if(strncmp(dirent.name, "r_", 2) == 0) {
        i = atoi(dirent.name + 2);
        if(i > max_num) {
          max_num = i;
          file_size = (uint16_t)dirent.size;
        }
      }
    }
    if(max_num == -1) {
      filename[2] = '0';
      filename[3] = 0;
    }
    else if((uint16_t)file_size + (uint16_t)length > MAX_POST_SIZE) {
      itoa(max_num + 1, filename + 2, 10);
    }
    else {
      itoa(max_num, filename + 2, 10);
    }
    return filename;
  }
  DPRINT("[ERROR] UNABLE TO OPEN ROOT DIRECTORY!!!\n");
  return NULL;
}

PROCESS_THREAD(sample_process, ev, data)
{
  PROCESS_BEGIN();

  if(get_config(SAMPLE_CONFIG) == 1)
  { // Config file does not exist! Use default and set file
    sensor_config.interval = SENSOR_INTERVAL;
    sensor_config.avrIDs_count = SENSOR_AVRIDS_COUNT;
    sensor_config.hasADC1 = SENSOR_HASADC1;
    sensor_config.hasADC2 = SENSOR_HASADC2;
    sensor_config.hasRain = SENSOR_HASRAIN;
    set_config(SAMPLE_CONFIG);
  }

  static struct etimer stimer;
  static uint8_t pb_buf[64];
  static int fd;

  static Sample sample;

  static int i;

  static char* filename;

  SENSORS_ACTIVATE(battery_sensor);
  SENSORS_ACTIVATE(temperature_sensor);

  DPRINT("[SAMP] Sampling sensors activated\n");

  while(1)
  {
    etimer_set(&stimer, CLOCK_SECOND * (sensor_config.interval - (get_time() % sensor_config.interval)));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&stimer));

    sample.time = get_time();

    sample.batt = get_sensor_batt();
    sample.has_batt = 1;

    sample.temp = get_sensor_temp();
    sample.has_temp = 1;

    sample.accX = get_sensor_acc_x();
    sample.accY = get_sensor_acc_y();
    sample.accZ = get_sensor_acc_z();

    sample.has_accX = 1;
    sample.has_accY = 1;
    sample.has_accZ = 1;

    if(sample.has_ADC1 = sensor_config.hasADC1) {
      sample.ADC1 = get_sensor_ADC1();
    }
    if(sample.has_ADC2 = sensor_config.hasADC2) {
      sample.ADC2 = get_sensor_ADC2();
    }
    if(sample.has_rain = sensor_config.hasRain) {
      sample.rain = get_sensor_rain();
    }
    if(sensor_config.avrIDs_count > 0) {
      sample.AVR.size = get_sensor_AVR(sensor_config.avrIDs, sensor_config.avrIDs_count, sample.AVR.bytes);
    }
    static pb_ostream_t ostream;
    ostream = pb_ostream_from_buffer(pb_buf, sizeof(pb_buf));
    pb_encode_delimited(&ostream, Sample_fields, &sample);

    filename = get_next_write_filename(ostream.bytes_written);
    if(filename == NULL) {
      continue;
    }

    DPRINT("[SAMP] Writing %d bytes to %s...\n", ostream.bytes_written, filename);

    fd = cfs_open(filename, CFS_WRITE | CFS_APPEND);
    if(fd >= 0)
    {
      DPRINT("  [1/3] Writing to file...\n");
      cfs_write(fd, pb_buf, ostream.bytes_written);
      DPRINT("  [2/3] Closing file...\n");
      cfs_close(fd);
      DPRINT("  [3/3] Done\n");
    }
    else
    {
      DPRINT("[SAMP] Failed to open file %s\n", filename);
    }
  }

  PROCESS_END();
}

PROCESS_THREAD(post_process, ev, data)
{
  PROCESS_BEGIN();

  static uint8_t retries;

  static uip_ipaddr_t addr;

  static char* filename;

  if(get_config(COMMS_CONFIG) == 1)
  { // Config file does not exist! Use default and set file
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
    set_config(COMMS_CONFIG);
  }

  while(1)
  {
    retries = 0;
    etimer_set(&timer, CLOCK_SECOND * (POST_config.interval - (get_time() % POST_config.interval)));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    while((filename = get_next_read_filename()) != NULL && retries < CONNECTION_RETRIES)
    {
      uip_ip6addr(&addr,
          POST_config.ip[0], POST_config.ip[1], POST_config.ip[2],
          POST_config.ip[3], POST_config.ip[4], POST_config.ip[5],
          POST_config.ip[6], POST_config.ip[7]);
      DPRINT("[POST][INIT] About to attempt POST with %s - RETRY [%d]\n", filename, retries);
      tcp_connect(&addr, UIP_HTONS(POST_config.port), NULL);
      load_file(filename);
      DPRINT("Connecting...\n");
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
      if(uip_aborted() || uip_timedout() || uip_closed()) {
        DPRINT("Could not establish connection\n");
        retries++;
      }
      else if(uip_connected()) {
        DPRINT("Connected\n");
        PSOCK_INIT(&ps, psock_buffer, sizeof(psock_buffer));
        etimer_set(&timeout_timer, CLOCK_SECOND*LIVE_CONNECTION_TIMEOUT);
        do {
          if(etimer_expired(&timeout_timer))
          {
            DPRINT("Connection took too long. TIMEOUT\n");
            PSOCK_CLOSE(&ps);
            retries++;
            break;
          }
          else if(data_length > 0)
          {
            DPRINT("[POST] Handle Connection\n");
            handle_connection(&ps);
            PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
          }
        } while(!(uip_closed() || uip_aborted() || uip_timedout()));
        DPRINT("\nConnection closed.\n");
        DPRINT("Status = %d\n", http_status);
        if(http_status/100 == 2)
        { // Status OK
          data_length = 0;
          retries = 0;
          cfs_remove(filename);
          DPRINT("[POST] Removing file\n");
        }
        else
        { // POST failed
          data_length = 0;
          retries++;
          DPRINT("[POST] Failed, not removing file\n");
        }
      }
    }
    PSOCK_CLOSE(&ps);
  }
  PROCESS_END();
}
