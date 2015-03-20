/*
 * -----------------------------------------------------------------
 *
 * Author  : Philip Basford <pjb@ecs.soton.ac.uk>
 * Created : 2015-03-09
 */

#include <stdlib.h>

#include "contiki.h"
#include "lib/sensors.h"
#include "dev/sht25.h"
#include "dev/sht25-sensor.h"

const struct sensors_sensor sht25_sensor;

enum {
  ON, OFF
};
static uint8_t state = OFF;
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
    switch(type) {
        case SHT25_SENSOR_TEMP:
            return sht25_temp();;
        case SHT25_SENSOR_HUMIDITY:
            return sht25_humidity();

    }
    return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
    switch(type) {
        case SENSORS_ACTIVE:
        case SENSORS_READY:
            return (state == ON);
    }
    return 0;
}



/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
  switch(type) {
  case SENSORS_ACTIVE:
    if(c) {
      if(!status(SENSORS_ACTIVE)) {
        rtimer_clock_t t0;
        sht25_init();
        state = ON;

        /* For for about 15 ms before the SHT25 can be used. */
        t0 = RTIMER_NOW();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + RTIMER_SECOND / 65));
      }
    } else {
      sht25_off();
      state = OFF;
    }
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(sht25_sensor, "sht25",
           value, configure, status);
