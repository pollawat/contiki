
#include <stdio.h>
#include "contiki.h"
#include "contiki-conf.h"
#include <string.h>
#include <stdio.h>
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
static int (*writebyte)(unsigned char c);


uint16_t crc16_up(uint16_t crc, uint8_t a){
    int i;

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

void protobuf_process_message(uint8_t *buf, uint8_t bytes){
    uint16_t rec_crc;
    uint16_t cal_crc = 0xFFFF;
    uint8_t i;

#ifdef PROTOBUF_HANDLER_DEBUG
  printf("Bytes recieved: %i\n", bytes);
  while (i < bytes){
    printf("%i,", (int)buf[i++]);
  }
  printf("\n");
#endif
    if(buf[1] != 0xFF){
        PRINTF("not a response packet so ignoring\n");
    }else{
        rec_crc = (buf[bytes - 1] << 8) | buf[bytes-2];
        PRINTF("Recieved CRC: %d\n", rec_crc);
        for(i=0; i < bytes-2; i++){
            cal_crc = crc16_up(cal_crc, buf[i]);
        }
        PRINTF("Calculated CRC: %d", rec_crc);
        if (rec_crc == cal_crc){
            PRINTF("CRCs match\n");
        }else{
            printf("CRCs do not match: Ignoring\n");
        }
    }
}




void protobuf_send_message(uint8_t addr, uint8_t opcode, uint8_t *payload,
        int8_t payload_length){
    uint8_t buf[MAX_MESSAGE_LENGTH];
    uint8_t buf_length = 0;       
    uint16_t crc = 0xFFFF;
    uint8_t i;
#ifdef PROTOBUF_HANDLER_DEBUG
    printf("Dest: %i\n", addr);
    printf("Optcode: %i\n", opcode);
    printf("Payload length: %i\n", opcode); 
#endif	 


    if (payload_length +4 > MAX_MESSAGE_LENGTH){
        //allow bytes for crc, dst and opcode
        //Message too long for avr at other end to handle...
        printf("Message too long to be sent\n");
        return;
    }
    buf[buf_length++] = addr;
    buf[buf_length++] = opcode;
    buf_length++;

    if (payload_length == 0){
        //No need to worry about including the payload in the CRC
    }else{
        for(i=0; i < payload_length; i++){
            buf[buf_length++] = payload[i];
        }
    }
    for(i=0; i <  buf_length; i++){
        crc16_up(crc, buf[i]);
    }
    buf[buf_length++] = crc & 0xFF; //Get the low order bits
    buf[buf_length++] = (crc >> 8) & 0xFF;
    PRINTF("CRC: %lu\n", crc);
    
    //ready to send    

}

void protobuf_handler_set_writeb(int (*wb)(unsigned char c)){
    writebyte = wb;    
}
