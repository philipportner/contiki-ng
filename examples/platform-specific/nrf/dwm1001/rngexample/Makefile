CONTIKI_PROJECT = rng
all: $(CONTIKI_PROJECT)

CONTIKI = ../../../../..

PLATFORMS_ONLY += nrf
BOARDS_ONLY = nrf53832/dwm1001

#use this to enable TSCH: MAKE_MAC = MAKE_MAC_TSCH
MAKE_MAC ?= MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include