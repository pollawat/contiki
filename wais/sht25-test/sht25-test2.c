#include "contiki.h"
#include "dev/sht25-sensor.h"
#include "dev/sht25.h"
#include <stdio.h>

PROCESS(test_sht25_process, "SHT25 test");
AUTOSTART_PROCESSES(&test_sht25_process);

float
floor(float x)
{
    if(x>=0.0f){
        return (float) ((int)x);
    } else{
        return (float) ((int)x-1);
    }
}

PROCESS_THREAD(test_sht25_process, ev, data)
{
    static struct etimer et;
    static uint16_t raw_temp;
    static uint16_t raw_hum;
    static float temp;
    static float hum;
    PROCESS_BEGIN();
    SENSORS_ACTIVATE(sht25_sensor);
    printf("Starting loop\n");
    while(1){
        etimer_set(&et, CLOCK_SECOND);
        PROCESS_YIELD();
        raw_temp = sht25_sensor.value(SHT25_SENSOR_TEMP);

        //printf("Raw temp =  %d \n",  (raw_temp));
        temp = SHT2x_CalcTemperatureC(raw_temp);
        printf("Temp = %ld.%03dC\n", (long)temp, (unsigned)((temp-floor(temp)) * 1000));
        raw_hum = sht25_sensor.value(SHT25_SENSOR_HUMIDITY);
        //printf("Raw humidity =  %d \n",  (raw_hum));
        hum = SHT2x_CalcRH(raw_hum);
        printf("humidity = %ld.%03d%%\n", (long)hum, (unsigned)((hum-floor(hum)) * 1000));
        etimer_reset(&et);
    }
    PROCESS_END();
}
