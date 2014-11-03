#ifndef SAMPLER_H
#define SAMPLER_H

	#include "contiki.h"
	#include "contiki-conf.h"
	#include <stdio.h>

	#include "config.h"
	// Sensors
	#include "sampling-sensors.h"
	#include "ms1-io.h"
	#include "filenames-old.h"

	// Config
	#include "settings.pb.h"
	#include "readings.pb.h"
	// Protobuf
	#include "dev/pb_decode.h"
	#include "dev/pb_encode.h"


	#include "z1-sampler-config-defaults.h"


	#include "dev/temperature-sensor.h"
	#include "dev/battery-sensor.h"
	#include "dev/protobuf-handler.h"
	#include "dev/event-sensor.h"
	#include "cfs/cfs.h"


	void avr_timer_handler(void *p);
	void refreshSensorConfig(void);

	PROCESS_NAME(sample_process);



	#define AVR_TIMEOUT_SECONDS 10

#endif