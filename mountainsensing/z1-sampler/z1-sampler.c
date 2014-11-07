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
 *         adapted from Battery and Temperature IPv6 Demo for Zolertia Z1
 * \author
 *          Dan Playle      <djap1g12@soton.ac.uk>
 *          Philip Basford  <pjb@ecs.soton.ac.uk>
 *          Graeme Bragg    <gmb1g08@ecs.soton.ac.uk>
 *          Tyler Ward      <tw16g08@ecs.soton.ac.uk>
 *          Kirk Martinez   <km@ecs.soton.ac.uk>
 */

#include "contiki.h"
#include "httpd-simple.h"
#include "webserver-nogui.h"

#include "dev/reset-sensor.h"

#include "dev/event-sensor.h"
#include "net/netstack.h"

#ifndef CC11xx_CC1120
  #include "dev/cc2420.h"
#endif

#include "dev/leds.h"

#include "config.h"
#include "sampler.h"
#include "poster.h"

#include "sampling-sensors.h"
#include "z1-sampler-config-defaults.h"
#include "web_defines.h"
#include "z1-sampler.h"
// General
#include "contiki.h"
#include <stdio.h>
#include <stdlib.h>
#include "cfs/cfs.h"

// Networking
#include "contiki-net.h"

// Protobuf
#include "dev/pb_decode.h"
#include "dev/pb_encode.h"

// Config
#include "settings.pb.h"
#include "readings.pb.h"

#include "filenames-old.h"

#include "platform-conf.h"


#define DEBUG 

#ifdef DEBUG 
    #define DPRINT(...) printf(__VA_ARGS__)
#else
    #define DPRINT(...)
#endif






//#define SENSE_ON /*Do not turn sensor power off */


PROCESS(web_sense_process, "Feshie Sense");
PROCESS(web_process, "Web Server");


AUTOSTART_PROCESSES(&web_sense_process);

/*---------------------------------------------------------------------------*/
// CONFIG CODE 


static struct psock web_ps;


/* returns the number of files and disk used on Flash
*/
int flash_du(int *filec, uint32_t *bytes)
{
struct cfs_dir dir;
struct cfs_dirent dirent;
static int count = 0;
static uint32_t used = 0;

if(cfs_opendir(&dir, "/") == 0) {
   while(cfs_readdir(&dir, &dirent) != -1) {
	count++;
	used += dirent.size;
   }
   *bytes = used;
   *filec = count;
   //cfs_closedir(&dir);
   return(0);
 }
else {
    return(-1);
 }

//cfs_closedir(&dirp);
*bytes = used;
*filec = count;
return(0);
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
uint8_t
get_url_param(char* par, char *url, char *key)
{
  
  char str[100];
  char *pch;
  uint8_t len;
  
  strcpy(str, url);
  len = strlen(key);
  pch = strtok(str, "?&");
  
  while(pch != NULL) {
      if(strncmp(pch, key, len) == 0) {
        // If the token is key-value pair desired
        par = pch + len + 1;
        return 1;

      }
      pch = strtok(NULL, "?&");
  }
  par = NULL;
  return 0;
}






static
PT_THREAD(web_handle_connection(struct psock *p))
{

  static uint8_t i;
  static char num[16], tmpstr[80];
  static uint16_t y;
  static uint8_t mo, d, h, mi, se;
  static bool submitted;
  static const char* ZERO = "0";
  static uint8_t clock_ret;
  static char AVRs[32];
  static SensorConfig sensor_config;
  static POSTConfig POST_config;
  static char web_buf[128];
  static char *url;
  static char param[URL_PARAM_LENGTH];

  DPRINT("[WEBD] Reading HTTP request line...\n");

  PSOCK_BEGIN(p);
  PSOCK_READTO(p, '\n');

  if(strncmp("GET ", (char*)web_buf, 4) == 0){
    url = web_buf + 4;
    strtok(url, " ");
    DPRINT("[WEBD] Got request for %s\n", url);
    
    if(strncmp(url, "/clock", 6) == 0)
    { // Serve clock form
      DPRINT("[WEBD] Serving /clock\n");
      submitted = 0;
      if(get_url_param(param, url, "submit") == 1) {
        submitted = 1;
        get_url_param(param, url, "y");
        y = atol(param == NULL ? ZERO : param);
        get_url_param(param, url, "mo");
        mo = atoi(param == NULL ? ZERO : param);
        get_url_param(param, url, "d");
        d = atoi(param == NULL ? ZERO : param);
        get_url_param(param, url, "h");
        h = atoi(param == NULL ? ZERO : param);
        get_url_param(param, url, "mi");
        mi = atoi(param == NULL ? ZERO : param);
        get_url_param(param, url, "s");
        se = atoi(param == NULL? ZERO : param);
        clock_ret = set_time(y, mo, d, h, mi, se);
      }
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      if(submitted) {
        if(clock_ret == 0){
          PSOCK_SEND_STR(p, "<h1>Success! Time set</h1>");
        }else{
          PSOCK_SEND_STR(p, "<h1>Warning, set time returned non-zero status</h1>");
        }
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
      ltoa(sensor_config.interval, tmpstr, 10);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, SENSOR_FORM_2);
      //AVR IDs
      DPRINT("[WEBD] Producing AVR IDs from config\n");
      for(i = 0; i < sensor_config.avrIDs_count; i++) {
        DPRINT(".");
        itoa(sensor_config.avrIDs[i], tmpstr, 10);
        PSOCK_SEND_STR(p, tmpstr);
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
      if(get_url_param(param, url, "sample") == 1){
        sensor_config.interval = atol(param);
      }else{
        //no value specified using default
        sensor_config.interval = SENSOR_INTERVAL;
      }

      if(get_url_param(param, url, "AVR") ==1)
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

      if(get_url_param(param, url, "rain") == 1 && strcmp(param, "y") == 0) {
        sensor_config.hasRain = 1;
      } else {
        sensor_config.hasRain = 0;
      }

      if(get_url_param(param, url, "adc1") == 1 && strcmp(param, "y") == 0) {
        sensor_config.hasADC1 = 1;
      } else {
        sensor_config.hasADC1 = 0;
      }

      
      if(get_url_param(param, url, "adc2") == 1 && strcmp(param, "y") == 0) {
        sensor_config.hasADC2 = 1;
      } else {
        sensor_config.hasADC2 = 0;
      }

      set_config(&sensor_config, SAMPLE_CONFIG);
      refreshSensorConfig();

      DPRINT("[WEBD] Stored Data\n");
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      PSOCK_SEND_STR(p, "<h1>OK</h1>");
      PSOCK_SEND_STR(p, BOTTOM);
    }
	/* Sets communications parameters
	* should put a lot of text into a string and send rather than 
	* like this
	*/
    else if(strncmp(url, "/comms", 6) == 0)
    {
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
      PSOCK_SEND_STR(p, COMMS_FORM_1);
      // Interval
      ltoa(POST_config.interval, tmpstr, 10);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2A);
      ltoa(POST_config.ip[0], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2B);
      ltoa(POST_config.ip[1], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2C);
      ltoa(POST_config.ip[2], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2D);
      ltoa(POST_config.ip[3], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2E);
      ltoa(POST_config.ip[4], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2F);
      ltoa(POST_config.ip[5], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2G);
      ltoa(POST_config.ip[6], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_2H);
      ltoa(POST_config.ip[7], tmpstr, 16);
      PSOCK_SEND_STR(p, tmpstr);

      PSOCK_SEND_STR(p, COMMS_FORM_3);

      ltoa(POST_config.port, tmpstr, 10);
      PSOCK_SEND_STR(p, tmpstr);
      PSOCK_SEND_STR(p, COMMS_FORM_4);

      PSOCK_SEND_STR(p, BOTTOM);
    }
    else if(strncmp(url, "/comsub", 7) == 0)
    {
      if(get_url_param(param, url, "interval") == 1){
        POST_config.interval = atol(param);
      }else{
        POST_config.interval = POST_INTERVAL;
      }

      static char chr[2] = {'a',0};

      for(i = 0; i < 8; i++) {
        chr[0] = 'a' + i;
        get_url_param(param, url, chr);
        POST_config.ip[i] = (param == NULL ? 0 : strtol(param, NULL, 16));
      }

      if(get_url_param(param, url, "port") == 1){
        POST_config.port = atol(param);
      }else{
        POST_config.port = POST_PORT;
      }

      set_config(&POST_config, COMMS_CONFIG);
      refreshPosterConfig();
      PSOCK_SEND_STR(p, HTTP_RES);
	     strcpy(tmpstr,TOP);
      strcat(tmpstr, "<h1>OK</h1>");
      strcat(tmpstr, BOTTOM);
      PSOCK_SEND_STR(p, tmpstr);
    }

    /******** debug call to see flash disk usage */
    else if(strncmp(url, "/du", 3) == 0)
    {
	static uint32_t bytesused;
	static int filecount;
	if( flash_du(&filecount, &bytesused) == -1){
		PSOCK_SEND_STR(p, "failed\n");
		return(-1);
		}

	sprintf(tmpstr, "%s\n%d files %ld bytes\n",TEXT_RES,filecount,bytesused);	
	PSOCK_SEND_STR(p, tmpstr);
    }
    /******** debug call to list all files on flash = SLOW! */
    else if(strncmp(url, "/ls", 3) == 0)
    {
	struct cfs_dir dir;
	struct cfs_dirent dirent;
	//char bigtmpstr[1024];

	PSOCK_SEND_STR(p, TEXT_RES);
	PSOCK_SEND_STR(p, "printing on serial\r\n");
	if(cfs_opendir(&dir, "/") == 0) {
	    while(cfs_readdir(&dir, &dirent) != -1) {
		printf("%s %ld\n", dirent.name, (long)dirent.size);
		/*
		if((strlen(bigtmpstr) + strlen(tmpstr) ) > 1023){
			strcat(bigtmpstr, tmpstr);
			}
		*/
	    }
	//PSOCK_SEND_STR(p, bigtmpstr);
	//cfs_closedir(&dir);
	}
	/* nicer version which may make spi clashes
	PSOCK_SEND_STR(p, TEXT_RES);
	if(cfs_opendir(&dir, "/") == 0) {
	    while(cfs_readdir(&dir, &dirent) != -1) {
		sprintf(tmpstr,"%s %ld\n", dirent.name, (long)dirent.size);
		PSOCK_SEND_STR(p, tmpstr);
	    }
	//cfs_closedir(&dir);
	}
	*/
    }


    /******** debug GET for the node settings
    * gives sampleinterval adc1 adc2 rain
    */
    else if(strncmp(url, "/settings", 9) == 0)
    {
      PSOCK_SEND_STR(p, HTTP_RES);
      PSOCK_SEND_STR(p, TOP);
	// get time
	ltoa(get_time(), tmpstr, 10);
	strcat(tmpstr, " ");
      // sample Interval
      ltoa(sensor_config.interval, num, 10);
	strcat(tmpstr, num);
	strcat(tmpstr, "s ");
      // POST Interval
      ltoa(POST_config.interval, num, 10);
	strcat(tmpstr, num);
	strcat(tmpstr, "P ");
      if( sensor_config.hasADC1 == 1)
	      strcat(tmpstr, "A1 ");
      if( sensor_config.hasADC2 == 1)
	      strcat(tmpstr, "A2 ");
      if( sensor_config.hasRain == 1)
	      strcat(tmpstr, "R ");
	ltoa(reset_sensor.value(0), num, 10);
	strcat(tmpstr, num);
      PSOCK_SEND_STR(p, tmpstr);
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
  static uint8_t web_buf[128];
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
  //static struct etimer timer;
  PROCESS_BEGIN();
  #ifndef CC11xx_CC1120
  cc2420_set_txpower(31);
  #endif
  #ifdef SPI_LOCKING
    printf("SPI Locking enabled\n");
  #endif

  process_start(&web_process, NULL);
  process_start(&sample_process, NULL);
  process_start(&post_process, NULL);


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/




