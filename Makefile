#  This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version. see <http://www.gnu.org/licenses/>
#
# Makefile for the multiwii project



EXTRA	:= 

SRCDIRS	:=   kernel userspace $(EXTRA)



all: 
	set -e; for dir in $(SRCDIRS); do $(MAKE) -C $$dir ; done

clean:	
	set -e; for dir in $(SRCDIRS); do $(MAKE) clean -C $$dir ; done
	