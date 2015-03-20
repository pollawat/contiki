#include "contiki.h"
#include "dev/sht25.h"
#include "adxl345.h"
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
    static int16_t x, y, z;
    PROCESS_BEGIN();
    sht25_init();
    accm_init();
    printf("Starting loop\n");
    while(1){
        etimer_set(&et, CLOCK_SECOND);
        PROCESS_YIELD();
        raw_temp = sht25_temp();
        temp = SHT2x_CalcTemperatureC(raw_temp);
        raw_hum = sht25_humidity();
        hum = SHT2x_CalcRH(raw_hum);
        x = accm_read_axis(X_AXIS);
        y = accm_read_axis(Y_AXIS);
        z = accm_read_axis(Z_AXIS);
        printf("%ld.%03dC %ld.%03d%% %d %d %d\n", 
            (long)temp, (unsigned)((temp-floor(temp)) * 1000), 
            (long)hum, (unsigned)((hum-floor(hum)) * 1000),
            x, y, z);
        
        etimer_reset(&et);
    }
    PROCESS_END();
}
