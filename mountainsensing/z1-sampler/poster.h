#ifndef POSTER_H
#define POSTER_H
	#include "filenames-old.h"
	#include "config.h"
	#include "contiki.h"
	#include "contiki-conf.h"
	#include <stdio.h>
	#include <stdint.h>
	#include "z1-sampler-config-defaults.h"
	#include "settings.pb.h"
	#include "cfs/cfs.h"
// Networking
#include "contiki-net.h"

	void refreshPosterConfig(void);
	#define CONNECTION_RETRIES 3
	#define LIVE_CONNECTION_TIMEOUT 300

	PROCESS_NAME(post_process);

#endif