
#include <stdio.h>
#include <string.h>

#include "contiki.h"
#include "contiki-conf.h"

#include "dev/protobuf-handler.h"
#include "dev/pb_decode.h"
#include "dev/pb_encode.h"
#include "dev/protocol-buffers/buffer.h"

#define MAX_MESSAGE_LENGTH 512

#define PROTOBUF_HANDLER_DEBUG
#ifdef PROTOBUF_HANDLER_DEBUG
	#define PRINTF(...) printf(__VA_ARGS__)
#else
	#define PRINTF(...)
#endif
static int (*writebyte)(unsigned char c) = NULL;

static process_event_t callback_event = NULL;
static struct process *callback_process = NULL;

uint16_t crc16_up(uint16_t crc, uint8_t a);

/*
 * The exact crc function used in the AVR & python
 * There is a contiki CRC function but not sure it'll
 * behave in the same way.
 */
uint16_t crc16_up(uint16_t crc, uint8_t a){
    int i;
//    PRINTF("CRC UP\n");
    crc ^= a;
//    PRINTF("crc = %lu\n", crc);
    for (i = 0; i < 8; ++i){
        if (crc & 1){
            crc = (crc >> 1) ^ 0xA001;
        }else{
            crc = (crc >> 1);
        }
    }
    return crc;
}

void protobuf_process_message(uint8_t *buf, uint8_t bytes){
    uint16_t rec_crc;
    uint16_t cal_crc = 0xFFFF;
    uint8_t i;
    process_data_t callback_data = NULL;

#ifdef PROTOBUF_HANDLER_DEBUG
  printf("Bytes recieved: %i\n", bytes);
  while (i < bytes){
    printf("%i,", (int)buf[i++]);
  }
  printf("\n");
#endif

    if(buf[1] != OPCODE_RESPONSE){
        PRINTF("not a response packet so ignoring\n");
    }else if(buf[1] != MASTER_ADDR){
        printf("not for me: ignoring");
    }else{
        rec_crc = (buf[bytes - 1] << 8) | buf[bytes-2];
        PRINTF("Recieved CRC: %d\n", rec_crc);
        for(i=0; i < bytes-2; i++){
            cal_crc = crc16_up(cal_crc, buf[i]);
        }
        PRINTF("Calculated CRC: %d", rec_crc);
        if (rec_crc == cal_crc){
            PRINTF("CRCs match\n");
        

        //put processing in here
            //strip out the first 2 and last 2 bytes
            if(callback_process != NULL){
                process_post(callback_process, callback_event, callback_data);
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
    uint8_t buf[MAX_MESSAGE_LENGTH];
    uint8_t buf_length = 0;       
    uint16_t crc = 0xFFFF;
    uint8_t i=0;
#ifdef PROTOBUF_HANDLER_DEBUG
    printf("Dest: %i\n", addr);
    printf("Optcode: %i\n", opcode);
    printf("Payload length: %i\n", payload_length); 
#endif	 


    if (payload_length +4 > MAX_MESSAGE_LENGTH){
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
//	PRINTF("%lu\n", crc);
    }
    buf[buf_length++] = crc & 0xFF; //Get the low order bits
    buf[buf_length++] = (crc >> 8) & 0xFF;
    PRINTF("CRC: %lu\n", crc);
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
    callback_event =ev;
    callback_process=p;

}
void read_devices(uint8_t *data, uint32_t *avrIDs, size_t count){
    int i;
    for(i=0; i < count; i++){
        protobuf_send_message(avrIDs[i], OPCODE_GET_DATA, 0, 0);
        
    }
}


