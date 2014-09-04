#include <stdio.h>
#include "contiki.h"
#include "dev/i2cmaster.h"  // Include IC driver
#include "dev/ms1-io.h"
#include "dev/protobuf-handler.h"
 
PROCESS (temp_process, "Test protocol buffer rs485 process");
AUTOSTART_PROCESSES (&temp_process);

/*---------------------------------------------------------------------------*/
static struct etimer et;
static process_event_t protobuf_event;
static struct ctimer timeout_timer;

static void timer_handler(void *p){
    process_post(&temp_process, protobuf_event, (process_data_t)NULL);
}

#define TIMEOUT_SECONDS 10

PROCESS_THREAD (temp_process, ev, data)
{
  PROCESS_BEGIN ();
  ms1_io_init();
  ms1_sense_on();
  uart1_init(0);
  uint8_t data_out[4];
  data_out[0] = 0x10;
  data_out[1] = 0x11;
  data_out[2] = 0x12;
  data_out[3] = 0x13;
  protobuf_event = process_alloc_event();
  protobuf_register_process_callback(&temp_process, protobuf_event) ;
  {
    while (1)
      {
        protobuf_send_message(0x01, OPCODE_LIST, data_out, NULL);
        printf("Sent message\n"); 
        ctimer_set(&timeout_timer, CLOCK_SECOND * TIMEOUT_SECONDS, timer_handler, NULL);
        PROCESS_YIELD_UNTIL(ev == protobuf_event);
        if(data != NULL){
	    printf("data recieved\n");
            ctimer_stop(&timeout_timer);
            //process data
        }else{
            printf("AVR timedout\n");
        }


        //Wait delay before looping to stop very fast output
//        etimer_set(&et, RTIMER_SECOND);          // Set the timer
//        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // wait for its expiration
 
      }
  }
  ms1_sense_off();
  PROCESS_END ();
}

