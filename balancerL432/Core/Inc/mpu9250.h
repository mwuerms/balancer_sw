/**
 * Martin Egli
 * 2022-09-10
 * MPU9250 Module
 */

#ifndef _MPU9250_H_
#define _MPU9250_H_

// - includes ------------------------------------------------------------------
#include <stdint.h>

// - defines -------------------------------------------------------------------

// - public functions ----------------------------------------------------------
void mpu9250_init(void);
void mpu9250_start(void);
void mpu9250_stop(void);

uint8_t mpu9250_read_who_am_i(void);
uint16_t mpu9250_read_config(void);
uint16_t mpu9250_write_config(void);
uint16_t mpu9250_read_int_config(void);
uint16_t mpu9250_write_int_config(void);

#define MPU9250_READ_SENSOR_DATA_SIZE	(1+3*2+1*2+3*2)
uint16_t mpu9250_read_sensor_data(uint8_t *rx_data);

uint16_t mpu9250_read_sensor_values(void);
float mpu9250_get_acc_xy_angle_deg(void);
int16_t mpu9250_get_gyr_z_rate_raw(void);
float mpu9250_get_gyr_z_angle_rate_deg_pro_s(void);

int16_t mpu9250_get_acc_xyz(int16_t *acc);
int16_t mpu9250_get_temp(void);
int16_t mpu9250_get_gyro_xyz(int16_t *gyro);

#endif // _MPU9250_H_
