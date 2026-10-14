/* ---- USER CODE: Header ---- */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* ---- USER CODE: Includes ---- */
#include <stdio.h>
#include "bno055.h"
#include "ertc-datalogger.h"
#include <math.h>

/* ---- USER CODE: PD ---- */
#define HAL_TIMEOUT 100

#define I2C_TIMEOUT 200
#define M_PI 3.14159265358979323846

// servos limits
#define SERVO_MAX_VALUE 200
#define SERVO_MIN_VALUE 100


/* ---- USER CODE: PFP ---- */
extern void initialise_monitor_handles(void);
uint32_t saturate(uint32_t val, uint32_t min, uint32_t max);

/* ---- USER CODE: 0 ---- */
void error()
{
  // printf("An error has occurred\n")
  while (1)
    ;
}

/*base structure to communicate with the imu*/
struct bno055_t bno055;

/* structure used to read the accel xyz data output as m/s2 or mg */
struct bno055_accel_double_t d_accel_xyz;
/* structure used to read the gyro xyz data output as dps or rps */
struct bno055_gyro_double_t d_gyro_xyz;


int8_t bus_read_wrapper(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t r_len)
{
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr, 1, reg_data, r_len, I2C_TIMEOUT);
  return (int8_t)status;
}

int8_t bus_write_wrapper(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t wr_len)
{
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr, 1, reg_data, wr_len, I2C_TIMEOUT);
  return (int8_t)status;
}

void delay_wrapper(u32 delay)
{
  HAL_Delay(delay);
}

uint32_t saturate(uint32_t val, uint32_t min, uint32_t max){
	if(val<min)
		return min;
	else if(val>max)
		return max;
	else
		return val;
}

/* declare an ertc_dlog struct */
struct ertc_dlog logger;
struct datalog
{
	float tilt_angle;
	float y_accelerate;
	float pan_control;
	float z_gyro;
} logger_data;


int8_t pan = 0;
int8_t tilt = 0;
float kp = 30;
float angle = 0;

float y_accelerate;
float z_gyro;
int8_t camera_tilt;


/* ---- USER CODE: Init ---- */
  // initialise_monitor_handles();

/* ---- USER CODE: 2 ---- */

  /* Configure data logger  */
  logger.uart_handle = huart3; // for serial
  //logger.uart_handle = huart2; // for wifi

  /* Disable LCD SPI SS */
  HAL_GPIO_WritePin(GPIO_OUT_SPI_CS_LCD_GPIO_Port, GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_SET);

  bno055.bus_write = bus_write_wrapper;
  bno055.bus_read = bus_read_wrapper;
  bno055.delay_msec = delay_wrapper;
  bno055.dev_addr = 0x28;
  int32_t comres = bno055_init(&bno055);

  comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);




/* ---- USER CODE: WHILE ---- */
  while (1)
  {

/* ---- USER CODE: 3 ---- */
	  HAL_Delay(20);
	  bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
	  bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);

	  // Read the y acceleration data
	  y_accelerate = d_accel_xyz.y;

	  // calculate tilt angle
	  float g = 9.81; // m/s^2
	  tilt = asin(y_accelerate / g) * (180 / M_PI); // convert to degrees
	  camera_tilt = -tilt;

	  // limit the tilt to 45 to -45
	  if(camera_tilt > 45) camera_tilt = 45;
	  if(camera_tilt < -45) camera_tilt = -45;

	  // read the z axis gyroscope value
	  z_gyro = d_gyro_xyz.z;

	  // calculate pan control signal using proportional control kp = 20
	  pan = -kp * z_gyro;

	  // limit the pan to 45 to -45
	  if(pan > 45) pan = 45;
	  if(pan < -45) pan = -45;

	  //update data logger value
	  logger_data.y_accelerate = y_accelerate;
	  logger_data.tilt_angle = (float)tilt;
	  logger_data.z_gyro = z_gyro;
	  logger_data.pan_control = (float)pan;
	  ertc_dlog_update(&logger); // check if someone is connected and waiting for data
	  ertc_dlog_send(&logger, &logger_data, sizeof(logger_data)); // send data


	  /* update pan-tilt camera */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
					(uint32_t)saturate((150+pan*(50.0/45.0)), SERVO_MIN_VALUE, SERVO_MAX_VALUE)); // tilt
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
					(uint32_t)saturate((150+camera_tilt*(50.0/45.0)), SERVO_MIN_VALUE, SERVO_MAX_VALUE)); // pan
  }

/* ---- USER CODE: Error_Handler_Debug ---- */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }

/* ---- USER CODE: 6 ---- */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
