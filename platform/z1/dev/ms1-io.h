#ifndef MS1IO
#define MS1IO

#define SENSE_EN_PORT(type) P4##type
#define SENSE_EN_PIN 0

#define RADIO_EN_PORT(type) P4##type
#define RADIO_EN_PIN 2

extern void ms1_io_init(void);
extern void ms1_sense_on(void);
extern void ms1_sense_off(void);
extern void ms1_radio_on(void);
extern void ms1_radio_off(void);
#endif
