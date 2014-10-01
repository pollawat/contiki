#define AVR_TIMEOUT_SECONDS 10
#define LIVE_CONNECTION_TIMEOUT 300
#define CONNECTION_RETRIES 3
#define MAX_POST_SIZE 30

#define SAMPLE_CONFIG 1
#define COMMS_CONFIG 2

static void avr_timer_handler(void *p);

static process_event_t protobuf_event;
static struct ctimer avr_timeout_timer;
static uint8_t avr_recieved = 0;
static uint8_t avr_retry_count = 0;
static uint8_t avr_id = 0;
