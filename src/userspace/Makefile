#  This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version. see <http://www.gnu.org/licenses/>
#
# Makefile for the multiwii userspace tools 



CFLAGS ?= -std=c99 -D_GNU_SOURCE

ifeq ($(VERBOSE_DEBUG),true)
	# When debugging, use the following instead
	CFLAGS	+=-O -g -DVERBOSE_DEBUG
else
	CFLAGS	+=-Wall -O2
endif


#-Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
#	-Wcast-align -Wwrite-strings -Wnested-externs -Winline \
#	-W -Wundef -Wmissing-prototypes 

MWI_TARGETS=  mwi



# target
#
mwi: config.o imu.o main.c multiwii.o output.o rc.o sensors.o serial.o 
	$(CC) $(CFLAGS) -lrt -lm -o $@ $^


# Commands
#
all-mwi: $(MWI_TARGETS)


clean-mwi:
	$(RM) *.o $(MWI_TARGETS)

install-mwi:$(MWI_TARGETS)
	cp $(MWI_TARGETS) ../

all: all-mwi


clean: clean-mwi

install: install-mwi


