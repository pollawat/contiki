

#include <stdio.h>
#include "contiki.h"
#include "contiki-conf.h"
#include <string.h>
#include "dev/protobuf-handler.h"

#define PROTOBUF_HANDLER_DEBUG


void protobuf_process_message(uint8_t *buf, uint8_t bytes)
{
#ifdef PROTOBUF_HANDLER_DEBUG
  printf("Bytes recieved: %i\n", bytes);
  int i = 0;
  while (i < bytes){
    printf("%i,", (int)buf[i++]);
  }
  printf("\n");
#endif

}
