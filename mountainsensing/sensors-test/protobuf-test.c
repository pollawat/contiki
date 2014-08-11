#include <stdio.h>
#include "contiki.h"
#include "dev/i2cmaster.h"  // Include IC driver
#include "dev/ms1-io.h"
#include "dev/protobuf-handler.h"
 
PROCESS (temp_process, "Test protocol buffer rs485 process");
AUTOSTART_PROCESSES (&temp_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;

PROCESS_THREAD (temp_process, ev, data)
{
  PROCESS_BEGIN ();
  ms1_sense_on();
  {
    while (1)
      {
        etimer_set(&et, RTIMER_SECOND);          // Set the timer
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // wait for its expiration
 
        protobuf_send_message(0x01, OPCODE_GET_DATA, 0x00, 0);
	printf("Sent request\n");
      }
  }
  ms1_sense_off();
  PROCESS_END ();
}
