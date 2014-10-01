#include <stdio.h>
#include "contiki.h"
#include "dev/i2cmaster.h"  // Include IC driver
 
#define TMP102_READ_INTERVAL (CLOCK_SECOND/2)  // Poll the sensor every 500 ms
 
PROCESS (temp_process, "Test rs485 process");
PROCESS(debug_process, "Testing interupt status Z1 Feshie");

AUTOSTART_PROCESSES (&temp_process, &debug_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;

PROCESS_THREAD (temp_process, ev, data)
{
  PROCESS_BEGIN ();

  P4DIR|=0x00000001;
  P4OUT|=0x00000001;
  // turn on pin 4.0 (vsense control)
  uart1_init(0);
  {
    while (1)
      {
        etimer_set(&et, TMP102_READ_INTERVAL);          // Set the timer
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // wait for its expiration
 
	uart1_writearray((unsigned char*)"look sending\r\n",14);
	printf ("sent");
	
      }
  }
  PROCESS_END ();
}

static struct etimer int_debug_timer;

PROCESS_THREAD(debug_process, ev, data)
{

  PROCESS_BEGIN();
  printf("Interupt debug process started\n");

  while(1) {
  etimer_set(&int_debug_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&int_debug_timer));
    printf("IFG1:%d\n", IFG1);
    printf("TACTL IE:%d, IFG:%d\n", TACTL & TAIE, TACTL & TAIFG);
    printf("TACCTL0 IE:%d IFG:%d \n", TACCTL0 & CCIE, TACCTL0 & CCIFG);
    printf("TBCTL IE:%d, IFG:%d\n", TBCTL & TBIE, TBCTL & TBIFG);
    printf("TBCCTL0 IE:%d IFG:%d \n", TBCCTL0 & CCIE, TBCCTL0 & CCIFG);
    printf("TBIV:%d \n", TBIV);
    printf("CACATL1 IE:%d, IFG:%d\n", CACTL1 & CAIE, CACTL1 & CAIFG);
    printf("UCA0CTL1 RXEIE:%d RXBRKIE:%d\n", UCA0CTL1 & UCRXEIE, UCA0CTL1 &
UCBRKIE);
    printf("IE2 TXIE:%d RXIE %d\n ", IE2 & UCA0TXIE, IE2 & UCA0RXIE);
    printf("UC1 IE:%d IFG:%d\n",  UC1IE & UCA1RXIE, UC1IFG & UCA1RXIFG);
  }


  PROCESS_END();
}
