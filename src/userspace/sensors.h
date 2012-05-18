/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * sensors.h
 *
 */

#ifndef MWI_SENSOR_H
#define MWI_SENSOR_H

void sensors_gyro_read(void);
void sensors_acc_read(void);
void sensors_mag_read(void);
void sensors_baro_read(void);
int8_t sensors_init(void);


#endif
