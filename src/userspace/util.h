/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * util.h , some utilities
 *
 */

#ifndef MWI_UTIL_H
#define MWI_UTIL_H
#include <stdint.h>



#if (defined(VERBOSE_DEBUG))
	#include <stdio.h>
	#define DEBUG(X) printf(X);printf(" ");printf("time = %u\n",(uint32_t)(micros()-previousTime));
#else
	#define DEBUG(X)
#endif


#define PI 3.1415926535897932384626433832795

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#include <unistd.h>
#define delay(t) usleep(1000*t)

uint64_t micros(void);

int8_t setup(void);
void loop(void);

long map(long x, long in_min, long in_max, long out_min, long out_max);


#define FAILURE_IMU_INIT -10

#endif
