#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>
#include "sensor_msgs/msg/imu.h"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
//#include "pico_usb_transport.h"
#include "pico_uart_transports.h"

#include "Services/MPU9250_Service.hpp"

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
/*
rcl_publisher_t pub_accel_x;
rcl_publisher_t pub_accel_y;
rcl_publisher_t pub_accel_z;

rcl_publisher_t pub_gyro_x;
rcl_publisher_t pub_gyro_y;
rcl_publisher_t pub_gyro_z;

rcl_publisher_t pub_temp;

std_msgs__msg__Float32 msg_accel;
std_msgs__msg__Float32 msg_gyro;
std_msgs__msg__Float32 msg_temp;*/

IMUService* imu_ptr = nullptr;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    IMUData data = imu_ptr->getAll();
    /*

    // Accelerometer
    msg_accel.data = data.accel.x_g;
    rcl_publish(&pub_accel_x, &msg_accel, NULL);
    msg_accel.data = data.accel.y_g;
    rcl_publish(&pub_accel_y, &msg_accel, NULL);
    msg_accel.data = data.accel.z_g;
    rcl_publish(&pub_accel_z, &msg_accel, NULL);

    // Gyroscope
    msg_gyro.data = data.gyro.x_dps;
    rcl_publish(&pub_gyro_x, &msg_gyro, NULL);
    msg_gyro.data = data.gyro.y_dps;
    rcl_publish(&pub_gyro_y, &msg_gyro, NULL);
    msg_gyro.data = data.gyro.z_dps;
    rcl_publish(&pub_gyro_z, &msg_gyro, NULL);

    // Temperature
    msg_temp.data = data.temp.temperature_c;
    rcl_publish(&pub_temp, &msg_temp, NULL);
    */

    imu_msg.linear_acceleration.x = data.accel.x_g;
    imu_msg.linear_acceleration.y = data.accel.y_g;
    imu_msg.linear_acceleration.z = data.accel.z_g;

    imu_msg.angular_velocity.x = data.gyro.x_dps;
    imu_msg.angular_velocity.y = data.gyro.y_dps;
    imu_msg.angular_velocity.z = data.gyro.z_dps;

    rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (ret != RCL_RET_OK) 
    {
        printf("Failed to publish IMU message\n");
    }

}
    

int main()
{
    // ---------- ADD: init stdio over USB ----------
    stdio_init_all();
    sleep_ms(2000); 
    printf("MAIN STARTED\n");
    // ---------- ADD: micro-ROS USB transport ----------
    rmw_uros_set_custom_transport(
    true,
    NULL,
    pico_serial_transport_open,
    pico_serial_transport_close,
    pico_serial_transport_write,
    pico_serial_transport_read
    );

    // Initialize the IMU
    MPU9250_HAL imu_hal(i2c_default, MPU6500_DEFAULT_ADDRESS);
    IMUService imu_service(imu_hal);
    imu_ptr = &imu_service;

    // Initialize IMU HAL (low-level sensor) - retry until successful
    while (!imu_hal.begin(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, 400000))
    {
        printf("IMU HAL begin failed, retrying...\n");
        sleep_ms(500);
    }

    while (!imu_service.begin())
    {
        printf("IMU Service begin failed, retrying...\n");
        sleep_ms(500);
    }

    printf("IMU Initialized successfully\n");

    // -------------------- micro-ROS variables --------------------
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        printf("Agent unreachable, exiting...\n");
        return ret;
    }

    // -------------------- micro-ROS support & node --------------------
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // -------------------- Publisher setup --------------------
    rclc_publisher_init_default(
        &imu_pub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    );

    /*
    rclc_publisher_init_default(&pub_accel_x, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "imu_accel_x");

    rclc_publisher_init_default(&pub_accel_y, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "imu_accel_y");

    rclc_publisher_init_default(&pub_accel_z, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "imu_accel_z");

    rclc_publisher_init_default(&pub_gyro_x, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "imu_gyro_x");

    rclc_publisher_init_default(&pub_gyro_y, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "imu_gyro_y");

    rclc_publisher_init_default(&pub_gyro_z, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "imu_gyro_z");

    rclc_publisher_init_default(&pub_temp, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "imu_temp");*/


    // -------------------- Timer setup --------------------

    rcl_ret_t rc = rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true  // use internal thread flag
    );
    if (rc != RCL_RET_OK) {
    printf("Timer init failed: %d\n", rc);
    }

        

    // -------------------- Executor setup --------------------
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);


    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
