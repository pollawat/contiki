/*
 * Copyright (c) 2011, Zolertia(TM) is a trademark by Advancare,SL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *         WaIS lab sensor
 *         Philip Basford <pjb@ecs.soton.ac.uk>
 * Based on work by
 * \author
 *         Niclas Finne    <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Joel Hoglund    <joel@sics.se>
 *         Enric M. Calvo  <ecalvo@zolertia.com>
 */


 #include "lab-sensor.h"




PROCESS(lab_sense_process, "WaIS Lab sensors");

AUTOSTART_PROCESSES(&lab_sense_process);

#define HISTORY 16

static char buf[256];
static int blen;




/*---------------------------------------------------------------------------*/
/* Only one single request at time */

#define ADD(...) do {                                                   \
    blen += snprintf(&buf[blen], sizeof(buf) - blen, __VA_ARGS__);      \
} while(0)


/*---------------------------------------------------------------------------*/
void
ipaddr_add(const uip_ipaddr_t *addr)
{
    uint16_t a;
    int i, f;
    for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
        a = (addr->u8[i] << 8) + addr->u8[i + 1];
        if(a == 0 && f >= 0) {
            if(f++ == 0){
                ADD("::");
            }
        } else {
            if(f > 0) {
                f = -1;
            } else if(i > 0) {
                ADD(":");
            }
            ADD("%x", a);
        }
    }
}
/*---------------------------------------------------------------------------*/

static
PT_THREAD(send_values(struct httpd_state *s))
{
    PSOCK_BEGIN(&s->sout);
    float mybatt;
    float mytemp;
    if(strncmp(s->filename, "/t", 2) == 0 ||
        s->filename[1] == '\0') {
        /* Default page: show latest sensor values as text (does not
           require Internet connection to Google for charts). */
        blen = 0;
        mybatt = get_mybatt();
        mytemp = get_mytemp();
        SEND_STRING(&s->sout, TOP);
        ADD("<h1>Current readings</h1>\n"
            "Battery: %ld.%03d V<br>"
            "Internal Temperature: %ld.%03d &deg; C",
            (long) mybatt, (unsigned) ((mybatt-floor(mybatt))*1000), 
            (long) mytemp, (unsigned) ((mytemp-floor(mytemp))*1000)); 
        SEND_STRING(&s->sout, buf);
        SEND_STRING(&s->sout, BOTTOM);
    }else if (strncmp(s->filename, "/r", 2) == 0) {
        static uip_ds6_route_t *r;
        static uip_ds6_nbr_t *nbr;
#if WEBSERVER_CONF_LOADTIME
        static clock_time_t numticks;
        numticks = clock_time();
#endif


        SEND_STRING(&s->sout, TOP);
        blen = 0;
        ADD("Neighbors<pre>");

        for(nbr = nbr_table_head(ds6_neighbors);
                nbr != NULL;
                nbr = nbr_table_next(ds6_neighbors, nbr)) {

#if WEBSERVER_CONF_NEIGHBOR_STATUS

            uint8_t j=blen+25;
            ipaddr_add(&nbr->ipaddr);
            while (blen < j){
                ADD(" ");
            }
            switch (nbr->state) {
                case NBR_INCOMPLETE: ADD(" INCOMPLETE");break;
                case NBR_REACHABLE: ADD(" REACHABLE");break;
                case NBR_STALE: ADD(" STALE");break;      
                case NBR_DELAY: ADD(" DELAY");break;
                case NBR_PROBE: ADD(" NBR_PROBE");break;
            }
#else
            ipaddr_add(&nbr->ipaddr);
#endif

            ADD("\n");

            if(blen > sizeof(buf) - 45) {
                SEND_STRING(&s->sout, buf);
                blen = 0;
            }
        }
        ADD("</pre>Routes<pre>");
        SEND_STRING(&s->sout, buf);

        blen = 0;

        for(r = uip_ds6_route_head(); r != NULL; r = uip_ds6_route_next(r)) {
#if WEBSERVER_CONF_ROUTE_LINKS
            ADD("<a href=http://[");
            ipaddr_add(&r->ipaddr);
            ADD("]/status.shtml>");
            SEND_STRING(&s->sout, buf); //TODO: why tunslip6 needs an output here, wpcapslip does not
            blen = 0;
            ipaddr_add(&r->ipaddr);
            ADD("</a>");
#else
            ipaddr_add(&r->ipaddr);
#endif
            ADD("/%u (via ", r->length);
            ipaddr_add(uip_ds6_route_nexthop(r));
            if(1 || (r->state.lifetime < 600)) {
                ADD(") %lus\n", r->state.lifetime);
            } else {
                ADD(")\n");
            }
            SEND_STRING(&s->sout, buf);

            blen = 0;
        }
        ADD("</pre>");

#if WEBSERVER_CONF_FILESTATS
        static uint16_t numtimes;
        ADD("<br><i>This page sent %u times</i>",++numtimes);
#endif

#if WEBSERVER_CONF_LOADTIME
        numticks = clock_time() - numticks + 1;
        ADD(" <i>(%lu.%02lu sec)</i>",numticks/CLOCK_SECOND,(100*(numticks%CLOCK_SECOND))/CLOCK_SECOND);
#endif

        SEND_STRING(&s->sout, buf);
        SEND_STRING(&s->sout, BOTTOM);

    }else{
        blen=0;
        mybatt = get_mybatt();
        mytemp = get_mytemp();
        ADD("{\"reading\":{");//start of json
        ADD("\"internal\": %ld.%03d,\"battery\":%ld.%03d,",
            (long) mytemp, (unsigned) ((mytemp-floor(mytemp))*1000),
            (long) mybatt, (unsigned) ((mybatt-floor(mybatt))*1000));
        ADD("\"x\":%d,\"y\":%d,\"z\":%d",
            (int) get_sensor_acc_x,
            (int) get_sensor_acc_y,
            (int) get_sensor_acc_z);
        ADD("}}");//end of json
        SEND_STRING(&s->sout, buf);

    }

  

  PSOCK_END(&s->sout);
}
/*---------------------------------------------------------------------------*/
httpd_simple_script_t
httpd_simple_get_script(const char *name)
{
    return send_values;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lab_sense_process, ev, data)
{
    PROCESS_BEGIN();
    cc2420_set_txpower(31);

    process_start(&webserver_nogui_process, NULL);

    setup_sensors();

  

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
