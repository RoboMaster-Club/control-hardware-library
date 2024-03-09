/**
 * @file       imu_task.c/h
 * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
 *             enable data ready pin to save cpu time.enalbe bmi088 data ready
 *             enable spi DMA to save the time spi transmit
 */

#ifndef imu_task_H
#define imu_task_H
#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "MahonyAHRS.h"
#include "math.h"

#define SPI_DMA_GYRO_LENGHT 8
#define SPI_DMA_ACCEL_LENGHT 9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS 0
#define IMU_SPI_SHFITS 1
#define IMU_UPDATE_SHFITS 2
#define IMU_NOTIFY_SHFITS 3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET 1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define IST8310_RX_BUF_DATA_OFFSET 16

#define TEMPERATURE_PID_KP 1600.0f
#define TEMPERATURE_PID_KI 0.2f
#define TEMPERATURE_PID_KD 0.0f

#define TEMPERATURE_PID_MAX_OUT 4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f

#define MPU6500_TEMP_PWM_MAX 5000

#define imu_task_INIT_TIME 7

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

#define RAD_TO_DEG 180.0f / 3.14159f

typedef struct Euler_Orientation
{
  float yaw;
  float pitch;
  float roll;
} Euler_Orientation_t;

typedef struct Quaternion_Orientation
{
  float yaw;
  float pitch;
  float roll;
} Quaternion_Orientation_t;

typedef struct IMU
{
  BMI088_Raw_t bmi088_raw;

  IST8310_Raw_t ist8310_raw;

  float quat[4];

  Euler_Orientation_t rad;

  Euler_Orientation_t deg;

} IMU_t;

extern IMU_t g_imu;

#endif
