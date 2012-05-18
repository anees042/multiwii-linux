#ifndef MWI_IMU_H
#define MWI_IMU_H


int8_t imu_init(void);
void imu_compute(void);
void imu_baro_compute(void);

#endif
