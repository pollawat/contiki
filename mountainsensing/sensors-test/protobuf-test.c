#include <stdio.h>
#include "contiki.h"
#include "dev/i2cmaster.h"  // Include IC driver
#include "dev/ms1-io.h"

 
PROCESS (temp_process, "Test protocol buffer rs485 process");
AUTOSTART_PROCESSES (&temp_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;

PROCESS_THREAD (temp_process, ev, data)
{
  PROCESS_BEGIN ();
  ms1_io_init();
  ms1_sense_on();
  uart1_init(0);
  {
    while (1)
      {
        etimer_set(&et, RTIMER_SECOND);          // Set the timer
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // wait for its expiration
 
	uart1_writearray((unsigned char*)"look sending",12);
	printf ("sent");
	
      }
  }
  ms1_sense_off();
  PROCESS_END ();
}
