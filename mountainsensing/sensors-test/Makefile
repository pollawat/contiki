ifndef TARGET
TARGET=z1
endif

CONTIKI_PROJECT = test-adxl345 tmp102-test batv-test 
ROJECT_SOURCEFILES = i2cmaster.c tmp102.c adxl345.c sky-sensors.c batv-sensor.c
APPS=serial-shell
 
all: $(CONTIKI_PROJECT)
 
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
