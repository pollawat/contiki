ifndef TARGET
    TARGET=z1
endif

CONTIKI_PROJECT = sht25-test2 sht25-test sht25-adxl345-test
PROJECT_SOURCEFILES = sht25.c sht25-sensor.c
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
