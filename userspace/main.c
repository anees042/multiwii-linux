#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include "util.h"

//struct timeval startTime;
struct timespec start;
struct timespec end;

long int micros() {

	clock_gettime(CLOCK_MONOTONIC, &end);
	return (end.tv_sec * 1000000L + (end.tv_nsec/1000 )
			- start.tv_sec * 1000000ULL + (start.tv_nsec/1000 ));

}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
     // switch LEDPIN state
    }
  }
}


void signal_callback_int(int signum) {
	printf("Caught signal %d\n", signum);
	// Cleanup and close up stuff here

	// Terminate program
	exit(signum);
}

int main(void) {

	// Register sigint callback
	signal(SIGINT, signal_callback_int);

	//	gettimeofday(&startTime,0 );
	clock_gettime(CLOCK_MONOTONIC, &start);

	printf("Setup\n");
	setup();
	printf("\nok\nGo\n");

	while (1)
		loop();

	return EXIT_SUCCESS;
}

