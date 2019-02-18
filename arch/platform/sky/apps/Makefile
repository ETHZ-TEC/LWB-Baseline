CONTIKI_PROJECT = burn-nodeid
CONTIKI = ../../../..
DESCRIPTION ?= Burn nodeid

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
# MODULES += os/net/mac/lwb
# PROJECT_SOURCEFILES += burn-nodeid.c
#PROJECT_CONF_PATH = project-conf.h
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)
CFLAGS += -g

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

include $(CONTIKI)/Makefile.include
