#include "contiki.h"
#include <stdio.h>
#include "sys/rtimer.h"
#define     PERIOD_T     RTIMER_SECOND

static struct rtimer my_timer;

PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);

static uint32_t counts = 0;

  // the function which gets called each time the rtimer triggers
static char periodic_rtimer(struct rtimer *rt, void* ptr){
  uint8_t ret;

  rtimer_clock_t time_now = RTIMER_NOW();

  counts++;

  if((counts % 10) == 0)
  {
    printf("%d seconds since startup\n", counts);
  }

  ret = rtimer_set(&my_timer, time_now + PERIOD_T, 1,
        (void (*)(struct rtimer *, void *))periodic_rtimer, NULL);
  if(ret){
   printf("Error Timer: %u\n", ret);
  }
  return 1;
}

PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();

  printf("Starting the application...\n");

  periodic_rtimer(&my_timer, NULL);

  while(1) {
    PROCESS_YIELD();
  }
  PROCESS_END();
}
