#ifndef MPU9250_REGISTERS_HPP
#define MPU9250_REGISTERS_HPP

/* Search: const expresion*/

/* Default I2C address for MPU6500 (part of MPU9250)*/
#define MPU6500_DEFAULT_ADDRESS 0x68
/* Device identification register(to verify access to MPU-9250) */
#define WHO_AM_I        0x75
/* Controls power modes, reset, clock source, and sleep/cycle status */
#define PWR_MGMT_1      0x6B  
/* Sets the sample rate divider to control the output data rate */
#define SMPLRT_DIV      0x19
/* Configures FIFO behavior, digital low-pass filter for Gyro and temperature*/
#define CONFIG          0x1A
/* Configures DLPF bypass for Accelerometer */
#define ACCEL_CONFIG2   0x1D


/* Sets interrupt pin polarity */
#define INT_PIN_CFG     0x37
/* Enables/disables interrupts */
#define INT_ENABLE      0x38

/* Upper byte of acceleration measurement on X-axis */
#define ACCEL_XOUT_H    0x3B
/* Lower byte of acceleration measurement on X-axis */
#define ACCEL_XOUT_L    0x3C

#define ACCEL_YOUT_H    0x3D

#define ACCEL_YOUT_L    0x3E

#define ACCEL_ZOUT_H    0x3F

#define ACCEL_ZOUT_L    0x40

/* Upper byte of temperature measuremen */
#define TEMP_OUT_H      0x41
/* Lower byte of temperature measurement */
#define TEMP_OUT_L      0x42

/* Upper byte of rotation measurement on X-axis */
#define GYRO_XOUT_H     0x43
/* Lower byte of rotation measurement on X-axis */
#define GYRO_XOUT_L     0x44

#define GYRO_YOUT_H     0x45

#define GYRO_YOUT_L     0x46

#define GYRO_ZOUT_H     0x47

#define GYRO_ZOUT_L     0x48

/* Control enables/disables FIFO and I2C Master function */
#define USER_CTRL       0x6A
/* Configures I2C Master mode */
#define I2C_MST_CTRL    0x24
/* Enables Slave 4, controls data transfer */
#define I2C_SLV4_CTRL   0x34
/* ets I2C address and read/write for Slave 4 */
#define I2C_SLV4_ADDR   0x31

/* Default I2C address for AK8963 magnetometer */
#define AK8963_DEFAULT_ADDRESS 0x0C
/* Lower byte of magnetic field measurement on X-axis */
#define AK8963_XOUT_L 0x03

/********************************** Gyroscope Scales *********************************** */
#define GYRO_FS_250  (0x00) 
#define GYRO_FS_500  (0x08) 
#define GYRO_FS_1000 (0x10)
#define GYRO_FS_2000 (0x18) 

/************************************ Acceleration Scales [Acceleration Range] ******************************* */

#define ACCEL_FS_2G  (0x00)
#define ACCEL_FS_4G  (0x08) 
#define ACCEL_FS_8G  (0x10) 
#define ACCEL_FS_16G (0x18) /* Acceleration range ±16 g*/



#endif // MPU9250_REGISTERS_HPP
