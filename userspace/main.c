#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <stdint.h>
#include "util.h"

//struct timeval startTime;
struct timespec start;
struct timespec end;

uint64_t  micros() {

	clock_gettime(CLOCK_MONOTONIC, &end);
	return (end.tv_sec * 1000000ULL + (end.tv_nsec/1000 )
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

	int8_t state=0;
	// Register sigint callback
	signal(SIGINT, signal_callback_int);

	//	gettimeofday(&startTime,0 );
	clock_gettime(CLOCK_MONOTONIC, &start);

	printf("Setup\n");
	state = setup();
	switch (state) {
		case FAILURE_IMU_INIT:
			printf("Failed to start imu , aborting ..\n");
			return EXIT_FAILURE;
			break;
		default:
			printf("\nok\nGo\n");

				while (1) loop();
			break;
	}


	return EXIT_SUCCESS;
}

