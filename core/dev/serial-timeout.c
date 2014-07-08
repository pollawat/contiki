/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 * This file is part of the Contiki operating system.
 *
 */
#include "dev/serial-timeout.h"
#include <string.h> /* for memcpy() */
#include <stdio.h>
#include "lib/ringbuf.h"
#include "contiki.h"
#include "contiki-conf.h"

#define SERIAL_TIMEOUT_DEBUG

#ifdef SERIAL_TIMEOUT_CONF_BUFSIZE
#define BUFSIZE SERIAL_TIMEOUT_CONF_BUFSIZE
#else /* SERIAL_LINE_CONF_BUFSIZE */
#define BUFSIZE 128
#endif /* SERIAL_LINE_CONF_BUFSIZE */

#if (BUFSIZE & (BUFSIZE - 1)) != 0
#error SERIAL_TIMEOUT_CONF_BUFSIZE must be a power of two (i.e., 1, 2, 4, 8, 16, 32, 64, ...).
#error Change SERIAL_TIMEOUT_CONF_BUFSIZE in contiki-conf.h.
#endif


static uint8_t rxbuf_data[BUFSIZE];
static rtimer_clock_t r0;
static uint8_t bytes;

PROCESS(serial_timeout_process, "Serial timeout driver");

process_event_t serial_timeout_event_message;

/*---------------------------------------------------------------------------*/
int
serial_timeout_input_byte(unsigned char c)
{
#ifdef SERIAL_TIMEOUT_DEBUG
  printf("Byte recieved...");
#endif
  r0 = RTIMER_NOW();
  if(bytes ==0){
#ifdef SERIAL_TIMEOUT_DEBUG
  printf("first\n");
#endif
    /* First byte of transmission*/
    memset(rxbuf_data, 0, sizeof(rxbuf_data)); 
    rxbuf_data[bytes++] = c;
    process_poll(&serial_timeout_process);
  }else{
#ifdef SERIAL_TIMEOUT_DEBUG
  printf("not first\n");
#endif
    /* Already read atleast one byte */
    if(bytes >= BUFSIZE){
      /* Overflow */
    }else{
      rxbuf_data[bytes++] = c;

    }
  }

  /* Wake up consumer process */
  process_poll(&serial_timeout_process);
  return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_timeout_process, ev, data)
{
  PROCESS_BEGIN();
  printf("Serial timeout process started\n");
  static uint8_t buf[BUFSIZE];
  while (1){
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    

    while (RTIMER_CLOCK_LT(RTIMER_NOW(), (r0 + SERIAL_TIMEOUT_VALUE)));
    if (bytes > BUFSIZE){
      printf("Serial recieve overflow");
    }else{
#ifdef SERIAL_TIMEOUT_DEBUG
      printf("Timeout reached\n");
#endif
      memcpy( buf, rxbuf_data, sizeof(rxbuf_data)); 
      bytes = 0;
      /* Broadcast event */
      process_post(PROCESS_BROADCAST, serial_timeout_event_message, buf);
      if(PROCESS_ERR_OK ==
        process_post(PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL)) {
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
serial_timeout_init(void)
{
  bytes = 0;
  memset(rxbuf_data, 0, sizeof(rxbuf_data)); 
  process_start(&serial_timeout_process, NULL);
}
/*---------------------------------------------------------------------------*/
