#include <stdio.h>
#include "contiki.h"
#include "dev/i2cmaster.h"  // Include IC driver
#include "dev/ms1-io.h"
#include "dev/protobuf-handler.h"
 
PROCESS (temp_process, "Test protocol buffer rs485 process");
AUTOSTART_PROCESSES (&temp_process);

/*---------------------------------------------------------------------------*/
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
  static uint8_t i = 0;
  static uint8_t recieved = 0;
  static uint8_t retry_count=0;
  protobuf_event = process_alloc_event();
  protobuf_register_process_callback(&temp_process, protobuf_event) ;
    while (1)
      {
        recieved = 0;
        retry_count = 0;
        data = NULL;
        do{
          protobuf_send_message(0x01, PROTBUF_OPCODE_GET_DATA, NULL , (int)NULL);
          printf("Sent message %d\n", i); 
          i = i+1;
          ctimer_set(&timeout_timer, CLOCK_SECOND * TIMEOUT_SECONDS, timer_handler, NULL);
          PROCESS_YIELD_UNTIL(ev == protobuf_event);
          if(data != NULL){
              printf("\tdata recieved on retry %d\n", retry_count);
              ctimer_stop(&timeout_timer);
              protobuf_data_t *pbd;
              pbd = data;
              printf("\tRecieved %d bytes\t", pbd->length);
              uint8_t j;
              for(j=0; j<pbd->length;j++){
                printf("%d:", pbd->data[j]);
              }
              printf("\n");
              //process data
              recieved = 1;
          }else{
              printf("AVR timedout\n");
              retry_count++;
          }
        }while(recieved ==0 && retry_count < PROTOBUF_RETRIES);
        printf("**********************\n");

        //Wait delay before looping to stop very fast output
//        etimer_set(&et, RTIMER_SECOND);          // Set the timer
//        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // wait for its expiration
 
    }
  ms1_sense_off();
  PROCESS_END ();
}

