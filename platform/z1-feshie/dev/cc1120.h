
#ifndef CC1120_H
#define CC1120_H

/* Contiki Headers. */
#include "contiki.h"
#include "contiki-conf.h"

/* Network Headers. */
#include "dev/radio.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

/* CC1120 headers. */
#include "cc1120-const.h"
#include "cc1120-config.h"
#include "cc1120-arch.h"

/* Platform Headers */
#include platform-conf.h

/* Misc Headers. */
#include <string.h>
#include <stdio.h>

#define CC1120_MAX_PAYLOAD 125

#if CC1120_FHSS_FCC_50 && CC1120_FHSS_ETSI_50
#error Error: FHSS, both CC1120_FHSS_ETSI_50 and CC1120_FHSS_FCC_50 defined. Please set only one.
#endif

#if CC1120_FHSS_ETSI_50
/* ETSI EN 300 220, 50 channels: Channel 0 863 Mhz, channel 49 869.125MHz */
#define CC1120_CARRIER_FREQUENCY 13808 /* Channel 0: 863 Mhz (in 16th MHz) */
#define CC1120_CHANNEL_SPACING 2       /* Channel spacing: 125kHz (in 16th MHz) */
#define CC1120_FREQ_OSC 512            /* in 16th MHz */
#define CC1120_LO_DIVIDER 4

#elif CC1120_FHSS_FCC_50
/* FHSS 902 -- 928 MHz (FCC Part 15.247; 15.249) */
#define CC1120_CARRIER_FREQUENCY 14432 /* Channel 0: 902 Mhz (in 16th MHz) */
#define CC1120_CHANNEL_SPACING 2       /* Channel spacing: 125kHz (in 16th MHz) */
#define CC1120_FREQ_OSC 512            /* in 16th MHz */
#define CC1120_LO_DIVIDER 4

#else
#error Unknown FHSS frequencies, please define CC1120_FHSS_ETSI_50 or CC1120_FHSS_FCC_50
#endif

extern const struct radio_driver cc11xx_driver;





#endif /* CC11xx_H */