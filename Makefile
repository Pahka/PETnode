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
ifeq (true,true)
C = ../Contiki
SRCS += $C/core/sys/process.c $C/core/sys/etimer.c $C/core/sys/timer.c
SRCS += $C/core/net/tcpip.c $C/core/net/psock.c $C/core/lib/memb.c
SRCS += $C/apps/webserver/httpd.c $C/apps/webserver/http-strings.c
SRCS += $C/apps/webserver/httpd-cgi.c
SRCS += $C/apps/webserver/httpd-fs.c
SRCS += $C/apps/webserver/webserver-nogui.c
SRCS += $C/cpu/arm/common/dbg-io/dbg-snprintf.c
SRCS += $C/cpu/arm/common/dbg-io/strformat.c
SRCS += clock.c
CFLAGS += -I$C -DCONTIKI=1 -I$C/cpu/arm/common/dbg-io
CFLAGS += -Wno-strict-aliasing -Wno-missing-field-initializers
CFLAGS += -Wno-sign-compare
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

CFLAGS += -g -std=c99 -Os -Wall -Werror -Wextra
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map
CFLAGS += -nodefaultlibs

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

install: all
	st-flash erase
	st-flash write $(PROJ_NAME).bin 0x08000000

test:	install
	echo "Reset the device and press enter."
	read foo
	-sudo arp -d 10.0.0.2
	ping -c 10 10.0.0.2
	wget http://10.0.0.2
