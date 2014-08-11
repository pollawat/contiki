
void protobuf_process_message(uint8_t *buf, uint8_t bytes);
void protobuf_send_message(uint8_t addr, uint8_t opcode, uint8_t *payload, int8_t payload_length);
uint16_t crc16_up(uint16_t crc, uint8_t a);
