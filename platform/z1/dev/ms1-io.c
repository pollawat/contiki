#include "contiki.h"
#include "contiki-conf.h"



/* Platform Headers */
#include "platform-conf.h"

#include "ms1-io.h"
#define MS1_IO_DEBUG
#ifdef MS1_IO_DEBUG
  #include <stdio.h>
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...)
#endif

void ms1_io_init(void){
  PRINTF("MS1 IO Init\n");
  // Set up sense control pin
  SENSE_EN_PORT(SEL) &= ~BV(SENSE_EN_PIN);
  SENSE_EN_PORT(DIR) |= BV(SENSE_EN_PIN);
  SENSE_EN_PORT(REN) &= ~BV(SENSE_EN_PIN);
  SENSE_EN_PORT(OUT) &= ~BV(SENSE_EN_PIN);

  // Set up radio control pin
  RADIO_EN_PORT(SEL) &= ~BV(RADIO_EN_PIN);
  RADIO_EN_PORT(DIR) |= BV(RADIO_EN_PIN);
  RADIO_EN_PORT(REN) &= ~BV(RADIO_EN_PIN);
  PRINTF("\tTurning on radio\n");
  //Turn on by default
  RADIO_EN_PORT(OUT) |= BV(RADIO_EN_PIN);
  PRINTF("\tP4 status\n");
  PRINTF("\tSEL: %i\n", (int)P4SEL);
  PRINTF("\tDIR: %i\n", (int)P4DIR);
  PRINTF("\tREN: %i\n", (int)P4REN);
  PRINTF("\tOUT: %i\n", (int)P4OUT);
  PRINTF("\tComplete\n");
}

void ms1_radio_on(void){
  PRINTF("Turning on radio\n");
  RADIO_EN_PORT(OUT) |= BV(RADIO_EN_PIN);
}

void ms1_radio_off(void){
  PRINTF("Turning off radio\n");
  RADIO_EN_PORT(OUT) &= ~BV(RADIO_EN_PIN);
}

void ms1_sense_on(void){
  PRINTF("Turning on sense\n");
  SENSE_EN_PORT(OUT) |= BV(SENSE_EN_PIN);
}

void ms1_sense_off(void){
  PRINTF("Turning off sense\n");
  SENSE_EN_PORT(OUT) &= ~BV(SENSE_EN_PIN);
}
