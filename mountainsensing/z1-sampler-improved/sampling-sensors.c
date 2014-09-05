// Sensors
#include "dev/uart1_i2c_master.h"

#include "dev/ds3231-sensor.h" 	// Clock
#include "dev/ds3231-sensor.c"
#include "dev/adc1-sensor.h" 	// ADC 1
#include "dev/adc2-sensor.h" 	// ADC 2
#include "dev/temperature-sensor.h" // Temp
#include "dev/battery-sensor.h" // Batt
#include "adxl345.h" 		// Accel
#include "dev/event-sensor.h"	//event sensor (rain)

/*
 * AVR_count - number of AVR IDs
 * avrIDs - array of AVR IDs
 * data - The data array to populate with the data
 *
 * Returns the lenght of the data
 */
static uint16_t get_sensor_AVR(uint8_t AVR_count, uint8_t *avrIDs, uint8_t *data)
{
  printf("sampling-sensors.c: get_sensor_AVR(): NOT IMPLEMENTED");
  return 0;
}

static uint16_t get_sensor_rain()
{
  return event_sensor.value(1);
}

static uint16_t get_sensor_ADC1()
{
  return adc1_sensor.value(0);
}

static uint16_t get_sensor_ADC2()
{
  return adc2_sensor.value(0);
}

static float get_sensor_temp()
{
  return (float)(((temperature_sensor.value(0)*2.500)/4096)-0.986)*282;
}

static float get_sensor_batt()
{
  return (float)((battery_sensor.value(0)*2.500*2)/4096);
}

static int16_t get_sensor_acc_x()
{
  return accm_read_axis(X_AXIS);
}

static int16_t get_sensor_acc_y()
{
  return accm_read_axis(Y_AXIS);
}

static int16_t get_sensor_acc_z()
{
  return accm_read_axis(Z_AXIS);
}

static uint32_t get_time()
{
  uint32_t time = (uint32_t)ds3231_sensor.value(DS3231_SENSOR_GET_EPOCH_SECONDS_MSB) << 16;
  time += (uint32_t)ds3231_sensor.value(DS3231_SENSOR_GET_EPOCH_SECONDS_LSB);
  return time;
}

void set_time(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s)
{
  static tm t;

  t.tm_year = y - 1900;
  t.tm_mon = mo - 1;
  t.tm_mday = d;
  t.tm_hour = h;
  t.tm_min = mi;
  t.tm_sec = s;

  ds3231_set_time(&t);
}
