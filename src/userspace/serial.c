/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * Serial communication
 *
 */
#include <stdio.h>
#include "multiwii.h"

extern uint8_t numberMotor ;
extern int16_t motor[8];
extern  uint16_t cycleTime;

void serial_com() {
	//printf("rcData[THROTTLE] = %i ", rcData[THROTTLE]);
						printf ("angle : %i, %i - ",angle[0],angle[1]);
			printf("MOTOR : ");
										for (uint8_t i =0;i<numberMotor;i++){
											// TODO SEND OUTPUT-> motor[i];
											printf(" %04i,",motor[i]);
										}
		printf("\n");
		printf("time = %i\n",cycleTime);
}
