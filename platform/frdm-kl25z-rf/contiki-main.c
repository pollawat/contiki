#include <stdint.h>
#include <stdio.h>

#include <clock.h>
#include <etimer.h>

#include <sys/process.h>
#include <sys/procinit.h>
#include <sys/autostart.h>

#include <MKL25Z4.h>

#include "nvic.h"
#include "debug-uart.h"
#include "cpu.h"

#include <cc11xx.h>


unsigned int idle_count = 0;

int
main(void)
{
  cpu_init();
  
  port_enable(PORTB_EN_MASK | PORTC_EN_MASK | PORTD_EN_MASK | PORTE_EN_MASK);
  
  dbg_setup_uart();
  printf("Initialising\n");
  
  clock_init();
  process_init();
  process_start(&etimer_process, NULL);
  cc11xx_arch_init();
  autostart_start(autostart_processes);
  printf("Processes running\n");
  while(1) {
    do {
    } while(process_run() > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
  }
  return 0;
}




