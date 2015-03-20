#ifndef LAB_SENSOR_H
#define LAB_SENSOR_H

#include "contiki.h"
#include "httpd-simple.h"
#include "webserver-nogui.h"
#include "dev/cc2420.h"
#include <stdio.h>
#include <string.h>
#include "sampling.h"
#include "project-conf.h"


#define WEBSERVER_CONF_LOADTIME 1
#define WEBSERVER_CONF_FILESTATS 1
#define WEBSERVER_CONF_NEIGHBOR_STATUS 1
/* Adding links requires a larger RAM buffer. To avoid static allocation
 * the stack can be used for formatting; however tcp retransmissions
 * and multiple connections can result in garbled segments.
 * TODO:use PSOCk_GENERATOR_SEND and tcp state storage to fix this.
 */
#define WEBSERVER_CONF_ROUTE_LINKS 1




void ipaddr_add(const uip_ipaddr_t *addr);
httpd_simple_script_t httpd_simple_get_script(const char *name);

const char *TOP = "<html><head><title>Contiki Web Sense</title></head><body>\n";
const char *BOTTOM = "</body></html>\n";

#endif