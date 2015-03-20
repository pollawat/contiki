#ifndef WAIS_SAMPLING_H
#define WAIS_SAMPLING_H

#include "contiki.h"
#include "dev/temperature-sensor.h"
#include "dev/battery-sensor.h"
#include "dev/cc2420.h"
#include "adxl345.h"
#include <stdint.h>
#include <stdio.h>
#include "project-conf.h"

void setup_sensors(void);

float floor(float x);

int get_battery(void);
float get_mybatt(void);

int get_internal_temp(void);
float get_internal_temp_converted(void);

int16_t get_sensor_acc_x(void);
int16_t get_sensor_acc_y(void);
int16_t get_sensor_acc_z(void);

#ifdef HAS_SHT25
	#include "dev/sht25.h"
	#include "dev/sht25-sensor.h"
	uint16_t get_sht_temperature(void);
	float get_sht_temperature_converted(void);
	uint16_t get_sht_humidity(void);
	float get_sht_humidity_converted(void);
#endif

#endif