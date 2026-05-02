/*
 * sensor.c
 *
 *  Created on: Mar 11, 2026
 *      Author: martin
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "sensor.h"
#include "mpu9250.h"
//#include "adc.h"
#include "../../../mmlib/mmutils.h"

#include "../../../mmnrf24lib/nrf24_msg.h"
#include "can.h"

// - FreeRTOS tasks ------------------------------------------------------------
osThreadId_t sensor_task_handle = NULL;
uint32_t sensor_task_buffer[ 128 ];
StaticTask_t sensor_task_ctrl_block;
const osThreadAttr_t sensor_task_attributes = {
  .name = "sensor_task",
  .cb_mem = &sensor_task_ctrl_block,
  .cb_size = sizeof(sensor_task_ctrl_block),
  .stack_mem = &sensor_task_buffer[0],
  .stack_size = sizeof(sensor_task_buffer),
  .priority = (osPriority_t) osPriorityLow,
};

osEventFlagsId_t sensor_event_handle = NULL;
StaticEventGroup_t sensor_event_ctrl_block;
const osEventFlagsAttr_t sensor_event_attributes = {
  .name = "sensor_events",
  .cb_mem = &sensor_event_ctrl_block,
  .cb_size = sizeof(sensor_event_ctrl_block),
};

osTimerId_t sensor_timer_handle = NULL;
StaticTimer_t sensor_timer_ctrl_block;
const osTimerAttr_t sensor_timer_attributes = {
  .name = "sensor_timer",
  .cb_mem = &sensor_timer_ctrl_block,
  .cb_size = sizeof(sensor_timer_ctrl_block),
};

#define EV_READ_SENSORS BITV(0)
static void sensor_timer_cb(void *argument) {
	osEventFlagsSet(sensor_event_handle, EV_READ_SENSORS);
}

uint16_t adc_values[4];
int16_t acc[3];
int16_t temp;
int16_t gyro[3];
float angle_deg;
static volatile uint16_t sensor_cnt = 0;
static void sensor_task_cb(void *argument) {
	uint32_t events = 0;

	while(1) {
		events = osEventFlagsWait(sensor_event_handle, 0x000000FF, osFlagsWaitAny, osWaitForever);
		if(events & EV_READ_SENSORS) {
			//adc_single_conversion(adc_values, 4);
			mpu9250_read_sensor_values();
			mpu9250_get_acc_xyz(acc);
			temp = mpu9250_get_temp();
			mpu9250_get_gyro_xyz(gyro);
			angle_deg = mpu9250_get_acc_xy_angle_deg();

			nrf24_msg_send_mpu9250_values(acc[0], acc[1], acc[2], temp, gyro[0], gyro[1], gyro[2]);
			//nrf24_msg_send_angle_values(angle_deg, acc[0], acc[1], gyro[2]);
			sensor_cnt++;

			//can_hello_motor(0);
		}
	}
}

// - public functions ----------------------------------------------------------
void sensor_init(void) {
	if(sensor_timer_handle  == NULL) {
		sensor_timer_handle = osTimerNew(sensor_timer_cb, osTimerPeriodic, NULL, &sensor_timer_attributes);
	}
	if(sensor_task_handle  == NULL) {
		sensor_task_handle = osThreadNew(sensor_task_cb, NULL, &sensor_task_attributes);
	}
	if(sensor_event_handle == NULL) {
		sensor_event_handle = osEventFlagsNew(&sensor_event_attributes);
	}
	mpu9250_init();
}

void sensor_start_continous(void) {
	//adc_single_conversion(adc_values, 4);
	mpu9250_start();
	osTimerStart(sensor_timer_handle, pdMS_TO_TICKS(250));
}

