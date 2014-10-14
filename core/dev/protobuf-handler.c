
#include <stdio.h>
#include <string.h>

#include "contiki.h"
#include "contiki-conf.h"

#include "dev/protobuf-handler.h"
//#include "dev/pb_decode.h"
//#include "dev/pb_encode.h"
//#include "dev/protocol-buffers/buffer.h"

#define PROTOBUF_HANDLER_DEBUG
#ifdef PROTOBUF_HANDLER_DEBUG
	#define PRINTF(...) printf(__VA_ARGS__)
#else
	#define PRINTF(...)
#endif
static int (*writebyte)(unsigned char c);

static process_event_t callback_event;
static struct process *callback_process;
static uint8_t processed_data[PROTBUF_MAX_MESSAGE_LENGTH -4]; //doesn't have src/dst or crc
static protobuf_data_t callback_data;


uint16_t crc16_up(uint16_t crc, uint8_t a);

/*
 * The exact crc function used in the AVR & python
 * There is a contiki CRC function but not sure it'll
 * behave in the same way.
 */
uint16_t 
crc16_up(uint16_t crc, uint8_t a)
{
    uint8_t i;
    crc ^= a;
    for (i = 0; i < 8; ++i){
        if (crc & 1){
            crc = (crc >> 1) ^ 0xA001;
        }else{
            crc = (crc >> 1);
        }
    }
    return crc;
}

void 
protobuf_init(void)
{
    writebyte = NULL;
    callback_event = (int)NULL;
    callback_process = NULL;


}


void 
protobuf_process_message(uint8_t *buf, uint8_t bytes)
{
    uint16_t rec_crc, cal_crc;
    uint8_t processed_data_length;
    uint8_t i;
    i = 0;
    cal_crc = 0xFFFF;
    if(bytes == 0){
      PRINTF("Spurious interrupt, ignoring\n");
      return;
    }else if(bytes < 4){
      PRINTF("TOO small for valid protocol buffer\n");
      return;
    }

#ifdef PROTOBUF_HANDLER_DEBUG
  printf("Bytes recieved: %i\n", bytes);
  while (i < bytes){
    printf("%i,", (int)buf[i++]);
  }
  printf("\n");
#endif

    if(buf[1] != PROTBUF_OPCODE_RESPONSE){
        PRINTF("not a response packet so ignoring\n");
    }else if(buf[0] != PROTBUF_MASTER_ADDR){
        printf("not for me: ignoring");
    }else{
        rec_crc = (buf[bytes - 1] << 8) | buf[bytes-2];
        PRINTF("Recieved CRC: %d\n", rec_crc);
        for(i=0; i < bytes-2; i++){
            cal_crc = crc16_up(cal_crc, buf[i]);
        }
        PRINTF("Calculated CRC: %d\n", cal_crc);
        if (rec_crc == cal_crc){
          PRINTF("CRCs match\n");
          processed_data_length = bytes -4;
          memcpy(buf+2, (void *)processed_data, processed_data_length);
        //first 2 and last 2 bytes are not wanted for storage
#ifdef PROTOBUF_HANDLER_DEBUG
          printf("Callback_data\n");
          for(i=0; i<processed_data_length; i++){
            printf("%d:",processed_data[i]);
          }
          printf("\n");
#endif
          callback_data.length = processed_data_length;
          callback_data.data = processed_data;    
          if(callback_process != NULL){
            process_post(callback_process, callback_event, &callback_data);
            PRINTF("Process posted\n");
          }
        }else{
          printf("CRCs do not match: Ignoring\n");
          return;
        }
    }

    
}




void protobuf_send_message(uint8_t addr, uint8_t opcode, uint8_t *payload,
        int8_t payload_length){
   PRINTF("protobuf_send_message\n");
    uint8_t buf[PROTBUF_MAX_MESSAGE_LENGTH];
    uint8_t buf_length = 0;       
    uint16_t crc = 0xFFFF;
    uint8_t i=0;
#ifdef PROTOBUF_HANDLER_DEBUG
    printf("Dest: %i\n", addr);
    printf("Optcode: %i\n", opcode);
    printf("Payload length: %i\n", payload_length); 
#endif	 


    if (payload_length +4 > PROTBUF_MAX_MESSAGE_LENGTH){
        //allow bytes for crc, dst and opcode
        //Message too long for avr at other end to handle...
        printf("Message too long to be sent\n");
        return;
    }
    buf[buf_length++] = addr;
    buf[buf_length++] = opcode;

    if (payload_length == 0){
        //No need to worry about including the payload in the CRC
    }else{
    	PRINTF("DATA:\n");
	    for(i=0; i < payload_length; i++){
  	    PRINTF("payload:%i,", payload[i]);
        buf[buf_length++] = payload[i];
	      PRINTF("buf:%i\n", buf[buf_length-1]);
      }
    }
    for(i=0; i <  buf_length; i++){
        crc = crc16_up(crc, buf[i]);
    }
    buf[buf_length++] = crc & 0xFF; //Get the low order bits
    buf[buf_length++] = (crc >> 8) & 0xFF;
    PRINTF("CRC: %lu\n", (long unsigned)crc);
    PRINTF("CRC low: %i\n", crc & 0xFF);
    PRINTF("CRC high: %i\n", (crc >>8) & 0xFF); 
    //ready to send    

    if(writebyte != NULL){
        for(i=0; i < buf_length; i++){
        writebyte(buf[i]);
        }
    }else{
	printf("No writebyte specified\n");
    }
}

void protobuf_handler_set_writeb(int (*wb)(unsigned char c)){
    writebyte = wb;    
}


void protobuf_register_process_callback(struct process *p, process_event_t ev){
    PRINTF("Process registered\n");    
    callback_event =ev;
    callback_process=p;

}
