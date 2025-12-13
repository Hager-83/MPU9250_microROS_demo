/**
 * @file : MPU9250_HAL.hpp
 * @brief: Hardware Abstraction Layer (HAL) for the MPU9250 motion sensor.
 * 
 * This header defines the MPU9250_HAL class, providing a simple interface for
 * communicating with the MPU9250 sensor (which includes an accelerometer, gyroscope,
 * temperature sensor, and AK8963 magnetometer) via I2C on a Raspberry Pi Pico w.
 * The class handles initialization, configuration, and raw data reading.
 * 
 * @author :[Sara Saad , Hager Shohieb]
 * @version:1.0
 * @date   :December 01, 2025
 *
 **/


#ifndef MPU9250_HAL_HPP
#define MPU9250_HAL_HPP

/* ************************************** Include Part **************************************** */
// edit c++ guid

/* MPU9250_Registers.hpp: Register definitions*/
#include "MPU9250_Registers.hpp"
/* cstdint: Standard integer types.*/
#include <cstdint>
/* pico/stdlib.h: Pico SDK standard library */
#include "pico/stdlib.h"
/* hardware/i2c.h: Pico SDK I2C hardware interface.*/
#include "hardware/i2c.h"
/* ******************************************************************************************** */

/**
 * @class :MPU9250_HAL
 * @brief :Hardware Abstraction Layer for MPU9250 sensor.
 * 
 * This class provides methods to initialize the I2C interface, test connection,
 * configure the sensor, and read raw sensor data (accelerometer, gyroscope,
 * temperature, and magnetometer). All read operations are blocking.
 * 
 */

class MPU9250_HAL 
{
    public:
    /**
     * @brie:Constructor for MPU9250_HAL.
     * 
     * Initializes the class with the I2C instance and device address.
     * 
     * @param i2c :Pointer to the I2C hardware instance (e.g., i2c0).
     * @param address :I2C address of the MPU9250 (default: 0x68).
     */
    MPU9250_HAL(i2c_inst_t* i2c , uint8_t address); // hint 

    /**
     * @brief :Initialize I2C interface.
     * 
     * Configures the I2C pins and baudrate for communication with the sensor.
     * Must be called before any other operations.
     * 
     * @param sda_pin :GPIO pin for SDA (data line).
     * @param scl_pin :GPIO pin for SCL (clock line).
     * @param baudrate_hz :I2C baudrate in Hz (e.g., 400000 for 400 kHz).
     * @return :true if initialization succeeded, false otherwise.
    */
    bool begin(uint sda_pin, uint scl_pin, uint32_t baudrate_hz);

    /**
     * @brief :Test connection to the MPU9250.
     * 
     * Reads the WHO_AM_I register (0x75) and verifies the device ID (expected: 0x71, 0x72, 0x73).
     * 
     * @return :true if connection is valid, false otherwise.
     */
    bool testConnection(); // edit is connected 

    /**
     * @brief I:nitialize and configure the MPU9250 sensor.
     * 
     * Performs a device reset, sets clock source, and enables basic sensor operation
     * (e.g., via PWR_MGMT_1, SMPLRT_DIV, CONFIG, etc.).
     * 
     * @return t:rue if initialization succeeded, false otherwise.
     */
    bool initMPU9250(); // edit

    /**
     * @brief :Initialize the AK8963 magnetometer.
     * 
     * Configures the magnetometer via I2C Master mode (e.g., using I2C_SLV4_ADDR).
     * 
     * @return t:rue if initialization succeeded, false otherwise.
     */
    bool initAK8963(); // but in private , or configuration file or configuration.yaml

    /**
     * @brief :Read raw accelerometer data.
     * 
     * Reads 16-bit signed values from ACCEL_XOUT_H/L, ACCEL_YOUT_H/L, ACCEL_ZOUT_H/L.
     * 
     * @param ax :Reference to store X-axis acceleration (raw).
     * @param ay :Reference to store Y-axis acceleration (raw).
     * @param az :Reference to store Z-axis acceleration (raw).
     * @return   :true if read succeeded, false otherwise.
     */
    bool readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az); // std_ optional in c++

    /**
     * @brief :Read raw gyroscope data.
     * 
     * Reads 16-bit signed values from GYRO_XOUT_H/L, GYRO_YOUT_H/L, GYRO_ZOUT_H/L.
     * 
     * @param gx :Reference to store X-axis gyro (raw).
     * @param gy :Reference to store Y-axis gyro (raw).
     * @param gz :Reference to store Z-axis gyro (raw).
     * @return   :true if read succeeded, false otherwise.
     */

    bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz);

    /**
     * @brief :Read raw temperature data.
     * 
     * Reads 16-bit signed value from TEMP_OUT_H/L.
     * 
     * @param temp :Reference to store temperature (raw).
     * @return :true if read succeeded, false otherwise.
     */
    bool readTempRaw(int16_t &temp);

    /**
     * @brief :Read all raw MPU9250 data at once (accelerometer, gyro, temperature).
     * 
     * Convenience method combining readAccelRaw, readGyroRaw, and readTempRaw.
     * 
     * @param ax :Reference to store X-axis acceleration (raw).
     * @param ay :Reference to store Y-axis acceleration (raw).
     * @param az :Reference to store Z-axis acceleration (raw).
     * @param gx :Reference to store X-axis gyro (raw).
     * @param gy :Reference to store Y-axis gyro (raw).
     * @param gz :Reference to store Z-axis gyro (raw).
     * @param temp :Reference to store temperature (raw).
     * @return :true if read succeeded, false otherwise.
     */
    bool readAllRaw(int16_t &ax, int16_t &ay, int16_t &az,
                    int16_t &gx, int16_t &gy, int16_t &gz,
                    int16_t &temp);

    /**
     * @brief :Read raw magnetometer data from AK8963.
     * 
     * Reads 16-bit signed values from the AK8963 registers (e.g., XOUT_L).
     * 
     * @param mx :Reference to store X-axis magnetic field (raw).
     * @param my :Reference to store Y-axis magnetic field (raw).
     * @param mz :Reference to store Z-axis magnetic field (raw).
     * @return :true if read succeeded, false otherwise.
     */              
    bool readMagRaw(int16_t &mx, int16_t &my, int16_t &mz);

    private:
    i2c_inst_t* i2c_ = NULL; // EDIT TO smart pointer
    uint8_t address_;
    bool i2c_configured_;

    /* ******************************** Helper Function ************************************ */
    /**
     * @brief :Write a single byte to a register.
     * 
     * Internal helper for I2C write operation.
     * 
     * @param reg :Register address.
     * @param value :Value to write.
     * @return :true if write succeeded, false otherwise.
    * */
    bool writeByte(uint8_t reg, uint8_t value);

    /**
     * @brief :Read multiple bytes from a register.
     * 
     * Internal helper for burst I2C read (e.g., 6 bytes for 3 axes).
     * 
     * @param reg :Starting register address.
     * @param buffer :Buffer to store read data.
     * @param len :Number of bytes to read.
     * @return :true if read succeeded, false otherwise.
    * */
    bool readBytes(uint8_t reg, uint8_t* buffer, size_t len); // edit c++
};

#endif // MPU9250_HAL_HPP
