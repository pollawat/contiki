#include "sampling.h"


void 
setup_sensors(void)
{
	SENSORS_ACTIVATE(battery_sensor);
    SENSORS_ACTIVATE(temperature_sensor);
#ifdef HAS_SHT25
    SENSORS_ACTIVATE(sht25_sensor);
#endif
}


float 
floor(float x)
{ 
    if(x>=0.0f){
        return (float) ((int)x);  
    } else{
        return (float) ((int)x-1);        
    }   
}

/*---------------------------------------------------------------------------*/
int
get_battery(void)
{
    return battery_sensor.value(0);
}
/*---------------------------------------------------------------------------*/
int 
get_internal_temp(void)
{
    return temperature_sensor.value(0);
}

float 
get_mybatt(void){ 
    return (float) ((get_battery()*2.500*2)/4096);
}

float 
get_internal_temp_converted(void){ 
    return (float) (((get_internal_temp()*2.500)/4096)-0.986)*282;
}


int16_t 
get_sensor_acc_x(void)
{
    return accm_read_axis(X_AXIS);
}

int16_t 
get_sensor_acc_y(void)
{
    return accm_read_axis(Y_AXIS);
}

int16_t 
get_sensor_acc_z(void)
{
    return accm_read_axis(Z_AXIS);
}


#ifdef HAS_SHT25
uint16_t 
get_sht_temperature(void)
{
    return sht25_sensor.value(SHT25_SENSOR_TEMP);
}

float 
get_sht_temperature_converted(void)
{
    uint16_t temp;
    float converted;
    temp = get_sht_temperature();
    //printf("raw temp = %d", temp);
    converted = SHT2x_CalcTemperatureC(temp);
    //printf("Temp = %ld.%03dC\n", (long)converted, (unsigned)((converted-floor(converted)) * 1000));
    return converted;
}

uint16_t 
get_sht_humidity(void)
{
    return sht25_sensor.value(SHT25_SENSOR_HUMIDITY);
}

float 
get_sht_humidity_converted(void)
{
    return SHT2x_CalcRH(get_sht_humidity());
}
#endif