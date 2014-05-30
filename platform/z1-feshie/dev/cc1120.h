
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


extern const struct radio_driver cc11xx_driver;





#endif /* CC11xx_H */