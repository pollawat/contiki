#define AVR_TIMEOUT_SECONDS 10

static void avr_timer_handler(void *p);

static process_event_t protobuf_event;
static struct ctimer timeout_timer;
