/*
 * -----------------------------------------------------------------
 *
 * Author  : Philip Basford <pjb@ecs.soton.ac.uk>
 * Created : 2015-03-09
 */

#ifndef __SHT25_SENSOR_H__
#define __SHT25_SENSOR_H__

#include "lib/sensors.h"

extern const struct sensors_sensor sht25_sensor;

#define SHT25_SENSOR_TEMP              0
#define SHT25_SENSOR_HUMIDITY          1

#endif