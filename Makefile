#
# Copyright (c) 2012 Pekka Nikander. See NOTICE for licensing information.
#

# Application files, platform-independent
SRCS := main.c

# Chip-specific files
SRCS += stm32f051_init.c
SRCS += enc28j60/enc28j60_init.c
SRCS += enc28j60/enc28j60.c
SRCS += enc28j60/enc28j60_packet.c
SRCS += enc28j60/enc28j60_stm32f0_spi.c

# HTTPD
ifeq (true,false)
C = ../Contiki
SRCS += $C/core/sys/process.c $C/core/net/tcpip.c $C/core/net/psock.c
CFLAGS += -I$C -DCONTIKI=1
endif

# uIP
UIP = ../Contiki/core/net
CFLAGS += -I. -I$(UIP) -I$(UIP)/..

SRCS += $(UIP)/uip.c $(UIP)/uip_arp.c

# Platform
PLATFORM=stm32f0

# Platform files
SRCS += $(PLATFORM)/startup_$(PLATFORM)xx.s $(PLATFORM)/system_$(PLATFORM)xx.c

# Project name for output files
PROJ_NAME=PETnode

# Location of the linker scripts
LDSCRIPT_INC=ldscripts

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

CFLAGS += -g -std=c99 -Os -Wall -Werror -Wextra -Wno-strict-aliasing
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map

CFLAGS += -I. -I$(PLATFORM) -Ienc28j60 -Iinclude

###################################################

ROOT=$(shell pwd)

###################################################

all: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -L$(LDSCRIPT_INC) -T$(PLATFORM).ld
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf

clean:
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst

