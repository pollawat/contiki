#ifndef SAMPLER_CONFIG_H
#define SAMPLER_CONFIG_H

#define SAMPLE_CONFIG 1
#define COMMS_CONFIG 2

#include <stdlib.h>

uint8_t set_config(void* pb, uint8_t config);
uint8_t get_config(void* pb, uint8_t config);

#endif