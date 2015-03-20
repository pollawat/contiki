#include "contiki.h"
#include "dev/sht11.h"
 
#include <stdio.h>
 
PROCESS(test_sht11_process, "SHT11 test");
AUTOSTART_PROCESSES(&test_sht11_process);
 
PROCESS_THREAD(test_sht11_process, ev, data)
{
  static struct etimer et;
  static unsigned rh;
 
  PROCESS_BEGIN();
  sht11_init();
 
  for (etimer_set(&et, CLOCK_SECOND);; etimer_reset(&et)) {
    PROCESS_YIELD();
    printf("Temperature:   %u degrees Celsius\n",
    (unsigned) (-39.60 + 0.01 * sht11_temp()));
    rh = sht11_humidity();
    printf("Rel. humidity: %u%%\n",
    (unsigned) (-4 + 0.0405*rh - 2.8e-6*(rh*rh)));
  }
 
  PROCESS_END();
}
