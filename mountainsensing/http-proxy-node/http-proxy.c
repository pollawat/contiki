/*
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
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         border-router
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "dev/tmp102.h"     // Include sensor driver
//#include "webserver.h"
//#include "webserver-nogui.h"
//#include "wget.c"

#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"

//PROCESS(webserver_nogui_process, "Web server");
PROCESS (temp_process, "Test Temperature process");
PROCESS(wget_process, "Wget");

AUTOSTART_PROCESSES (&wget_process);


extern process_event_t serial_line_event_message;


/*---------------------------------------------------------------------------
 * TEMPRATURE CODE
 *--------------------------------------------------------------------------*/
#define TMP102_READ_INTERVAL (CLOCK_SECOND*4)  // Poll the sensor every 500 ms
static struct etimer et;
 
PROCESS_THREAD (temp_process, ev, data)
{
  PROCESS_BEGIN ();
 
  {
    int16_t  tempint;
    uint16_t tempfrac;
    int16_t  raw;
    uint16_t absraw;
    int16_t  sign;
    char     minus = ' ';
 
    tmp102_init();
 
    while (1)
      {
        etimer_set(&et, TMP102_READ_INTERVAL);          // Set the timer
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // wait for its expiration
 
        sign = 1;
 
	raw = tmp102_read_temp_raw();  // Reading from the sensor
 
        absraw = raw;
        if (raw < 0) { // Perform 2C's if sensor returned negative data
          absraw = (raw ^ 0xFFFF) + 1;
          sign = -1;
        }
	tempint  = (absraw >> 8) * sign;
        tempfrac = ((absraw>>4) % 16) * 625; // Info in 1/10000 of degree
        minus = ((tempint == 0) & (sign == -1)) ? '-'  : ' ' ;
	printf ("Temp = %c%d.%04d\n", minus, tempint, tempfrac);
      }
  }
  PROCESS_END ();
}

/*-----------------------------------------------------------------------------
 * WEBSERVER CODE
 *----------------------------------------------------------------------------*/


/*
PROCESS_THREAD(webserver_nogui_process, ev, data)
{
  PROCESS_BEGIN();

  httpd_init();

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    httpd_appcall(data);
  }
  
  PROCESS_END();
}

AUTOSTART_PROCESSES (&temp_process, &webserver_nogui_process);
*/
static const char *TOP = "<html><head><title>ContikiRPL</title></head><body>\n";
static const char *BOTTOM = "</body></html>\n";
#if BUF_USES_STACK
static char *bufptr, *bufend;
#define ADD(...) do {                                                   \
    bufptr += snprintf(bufptr, bufend - bufptr, __VA_ARGS__);      \
  } while(0)
#else
static char buf[256];
static int blen;
#define ADD(...) do {                                                   \
    blen += snprintf(&buf[blen], sizeof(buf) - blen, __VA_ARGS__);      \
  } while(0)
#endif

/*---------------------------------------------------------------------------*/
static void
ipaddr_add(const uip_ipaddr_t *addr)
{
  uint16_t a;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) ADD("::");
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        ADD(":");
      }
      ADD("%x", a);
    }
  }
}
/*---------------------------------------------------------------------------
static
PT_THREAD(generate_routes(struct httpd_state *s))
{
}
/*---------------------------------------------------------------------------
httpd_simple_script_t
httpd_simple_get_script(const char *name)
{

  return generate_routes;
}

/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTA("Server IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINTA(" ");
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTA("\n");
    }
  }
}


/*---------------------------------------------------------------------------
 * WGET CODE
 *--------------------------------------------------------------------------*/
PROCESS_THREAD(wget_process, ev, data)
{
  static char url[128];						//store url to be requested
  uip_ipaddr_t addr;						//store target ip address
  unsigned char i;						//counter
  static char host[32];						//store host name
  char *file;							//store file name
  register char *urlptr;					//pointer to point at characters


  PROCESS_BEGIN();

  while(1)							//we want to request multiple pages so continue forever
  {
     PROCESS_YIELD();						//wait for a serial interupt
     if(ev == serial_line_event_message) {			//if we have recieved a line on serial port
        printf("received request: %s\n", (char *)data);		//print wget request
	strcpy(url, (char *)data);				//coppy data to url


        /* Trim off any spaces in the end of the url. */
        urlptr = url + strlen(url) - 1;				//work out length of url
        while(*urlptr == ' ' && urlptr > url) {			//if url has whitespace
          *urlptr = 0;						//remove whitespace
          --urlptr;						//reduce length
        }


  	if(urlptr != url) {					//if there is a url

		/* See if the URL starts with http://, otherwise prepend it. */
		if(strncmp(url, "http://", 7) != 0) {		//if string dosent start with http://
			while(urlptr >= url) {			//for every character starting at the end
				*(urlptr + 7) = *urlptr;	//move character along
				--urlptr;			//move to previous character
			}
			strncpy(url, "http://", 7);		//prepend http://
		}

	
		/* Find host part of the URL. */
		urlptr = &url[7];				//start after http://
		for(i = 0; i < sizeof(host); ++i) {		//for each character starting at beginning
			if(*urlptr == 0 ||			//if is not EOS
			*urlptr == '/' ||			//if is start of path
			*urlptr == ' ' ||			//is it is a space
			*urlptr == ':') {			//if is start of port
				host[i] = 0;			//add EOS terminator
				break;				//break out of for loop
			}
			host[i] = *urlptr;			//add to strinf
			++urlptr;				//increase length
		}

		
		printf("host: %s\n", (char *)host);		//print host name

	

		/* XXX: Here we should find the port part of the URL, but this isn't
		 * currently done because of laziness from the programmer's side
		 * :-) */ 

		/* Find file part of the URL. */
		while(*urlptr != '/' && *urlptr != 0) {		//if not start of file
			++urlptr;				//increment pointer
		}
		if(*urlptr == '/') {				//if found file start
			file = urlptr;				//store file
		} else {					//else found EOS
			file = "/";				//no file
		}


		printf("file: %s\n", (char *)file);		//print file name


  	}

     }
  }

#ifdef LALALALALA

#if UIP_UDP
  /* First check if the host is an IP address. */
  if(uiplib_ipaddrconv(host, &addr) == 0) {    
    uip_ipaddr_t *addrptr;
    /* Try to lookup the hostname. If it fails, we initiate a hostname */
       lookup and print out an informative message on the
       statusbar. 
    if(resolv_lookup(host, &addrptr) != RESOLV_STATUS_CACHED) {
      resolv_query(host);
      puts("Resolving host...");
      return;
    }
    uip_ipaddr_copy(&addr, addrptr);
  }
#else /* UIP_UDP */
  uiplib_ipaddrconv(host, &addr);
#endif /* UIP_UDP */

  /* The hostname we present in the hostname table, so we send out the */
     initial GET request. 
  if(webclient_get(host, 80, file) == 0) {
    puts("Out of memory error");
  } else {
    puts("Connecting...");
  }
}


  while(1) {

    PROCESS_WAIT_EVENT();
  
    if(ev == tcpip_event) {
      webclient_appcall(data);
#if UIP_UDP
    } else if(ev == resolv_event_found) {
      /* Either found a hostname, or not. */
      if((char *)data != NULL &&
        resolv_lookup((char *)data, NULL) == RESOLV_STATUS_CACHED) {
        start_get();
      } else {
        puts("Host not found");
        app_quit();
      }
#endif /* UIP_UDP */ 
    }
  }

#endif

  PROCESS_END();
}
/*-----------------------------------------------------------------------------------*/

