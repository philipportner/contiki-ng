CONTIKI_PROJECT = rng
all: $(CONTIKI_PROJECT)

CONTIKI = ../..

TARGET=nrf
BOARD=nrf52832/dwm1001

#use this to enable TSCH: MAKE_MAC = MAKE_MAC_TSCH
MAKE_MAC ?= MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include

flash-dev1: all
	pyocd flash -t nrf52 -u 3104360201021b1e4353121f004e4a59a5a5a5a597969908 ./build/nrf52840/bluebite/rng.elf

flash-dev2: all
	pyocd flash -t nrf52 -u 3104360201021b1f4353121f004e4a59a5a5a5a597969908 ./build/nrf52840/bluebite/rng.elf
