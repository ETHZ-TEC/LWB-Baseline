CONTIKI_PROJECT = lwb-test
CONTIKI = ../..
DESCRIPTION ?= LWB test

# for convenience only
ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
endif

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/lwb
PROJECT_SOURCEFILES += rtimer-ext.c glossy.c my2c.c
#PROJECT_CONF_PATH = project-conf.h
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)
CFLAGS += -g
LDFLAGS += -Wl,--section-start -Wl,.testbedConfigSection=0xd400

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)
	msp430-objdump -d -S lwb-test.sky > lwb-test.lst
	msp430-objdump -D -S lwb-test.sky > lwb-test_all.lst
	msp430-objcopy -O ihex lwb-test.sky lwb-test.ihex

upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
