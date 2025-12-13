#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
//#include "pico_usb_transport.h"
#include "pico_uart_transports.h"

#include "Services/MPU9250_Service.hpp"


rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
IMUService* imu_ptr = nullptr;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    AccelData accel = imu_ptr->getAccelerometer();
    msg.data = accel.x_g;
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret != RCL_RET_OK) {
    printf("Failed to publish message: %d\n", ret);
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

    /*
    rmw_uros_set_custom_transport(
		true,
		(void*) &stdio_usb,
        NULL, NULL, NULL, NULL
	);*/

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
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "imu_accel_x");

    // -------------------- Timer setup --------------------
    /*rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        (rcl_timer_callback_t) timer_callback);*/
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

    //gpio_put(LED_PIN, 1);

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
