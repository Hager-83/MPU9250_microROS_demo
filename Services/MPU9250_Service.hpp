/**
 * @file  :IMU_SERVICE_HPP
 * @brief :IMU Service Layer for processed sensor data from MPU9250.
 * 
 * This header defines data structures for IMU sensor readings (accelerometer, gyroscope,
 * temperature, and magnetometer) and the IMUService class. The service provides
 * scaled and processed data in physical units (e.g., g, dps, °C, µT) on top of the
 * raw HAL interface.
 * 
 * @author  :[Hager Shohieb, Sara Saad]
 * @version :1.0
 * @date    :December 01, 2025
 *
 * */

#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

/****************************************** include part ********************************************* */
#include "../HAL/MPU9250_HAL.hpp"
#include <cstdint>
/**************************************** User Data Types Part *************************************** */
/**
 * @struct :AccelData
 * @brief  :Structure holding processed accelerometer data in g (gravity units).
 */
struct AccelData 
{
    float x_g;
    float y_g;
    float z_g;
};

/**
 * @struct :GyroData
 * @brief  :Structure holding processed gyroscope data in dps (degrees per second).
 */
struct GyroData 
{
    float x_dps;
    float y_dps;
    float z_dps;
};

/**
 * @struct :TempData
 * @brief  :Structure holding processed temperature data in °C (Celsius).
 */
struct TempData 
{
    float temperature_c;
};

/**
 * @struct :MagData
 * @brief  :Structure holding processed magnetometer data in µT (microtesla).
 */
struct MagData 
{
    float x_uT;
    float y_uT;
    float z_uT;
};

/**
 * @struct :IMUData
 * @brief  :Composite structure holding all processed IMU sensor data.
 */
struct IMUData 
{
    AccelData accel;
    GyroData gyro;
    TempData temp;
    MagData mag;
};
/****************************************************************************************************** */
/**
 * @class :IMUService
 * @brief :Service layer for retrieving scaled IMU sensor data.
 * 
 * This class wraps the MPU9250_HAL to provide convenient access to processed
 * sensor readings in physical units. It handles scaling from raw LSB values
 * to meaningful units. Initialization via begin() ensures the HAL is ready.
 * 
 * @note :Scaling factors are constant members; they must be initialized in the
 *       constructor with values based on the sensor's full-scale range
 *       (e.g., accelScale_ = 1.0f / 16384.0f for ±2g).
 */
class IMUService 
{
public:
    /**
     * @brief :Constructor for IMUService.
     * 
     * Initializes the service with a reference to the HAL instance.
     * Scaling factors should be computed here based on HAL configuration.
     * 
     * @param hal :Reference to the MPU9250_HAL instance.
     */
    IMUService(MPU9250_HAL &hal);

    /**
     * @brief :Constructor for IMUService.
     * 
     * Initializes the service with a reference to the HAL instance.
     * Scaling factors should be computed here based on HAL configuration.
     * 
     * @param hal :Reference to the MPU9250_HAL instance.
    **/
    bool begin();

    /**
     * @brief :Get processed accelerometer data.
     * 
     * Reads raw accel data from HAL, applies scaling, and returns in g units.
     * 
     * @return :AccelData structure with scaled values.
    */
    AccelData getAccelerometer();

    /**
     * @brief Get processed gyroscope data.
     * 
     * Reads raw gyro data from HAL, applies scaling, and returns in dps.
     * 
     * @return GyroData structure with scaled values.
     */
    GyroData  getGyroscope();

    /**
     * @brief :Get processed temperature data.
     * 
     * Reads raw temp data from HAL and converts to °C.
     * Formula: temp_c = (raw_temp / 333.87f) + 21.0f (example; adjust per datasheet).
     * 
     * @return :TempData structure with scaled value.
     */
    TempData  getTemperature();

    /**
     * @brief :Get processed magnetometer data.
     * 
     * Reads raw mag data from AK8963 via HAL, applies scaling, and returns in µT.
     * 
     * @return :MagData structure with scaled values.
     */
    MagData   getMagnetometer();

    /**
     * @brief :Get all processed IMU data at once.
     * 
     * Convenience method calling all getters and combining into IMUData.
     * 
     * @return :IMUData structure with all scaled values.
     */
    IMUData   getAll();

private:
    MPU9250_HAL &hal_;

    const float accelScale_; // LSB -> g
    const float gyroScale_;  // LSB -> deg/s
    const float magScale_;   // LSB -> µTesla (scaling from AK8963)
    const float tempScale_;
};

#endif // IMU_SERVICE_HPP
