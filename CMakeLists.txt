cmake_minimum_required(VERSION 3.12)

# Import Pico SDK 
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(IMU_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# Add the micro-ROS agent lib directory
link_directories(libmicroros)

# Add your executable with micro-ROS example source + IMU HAL and Service sources
add_executable(IMU_test
    IMU_test.cpp       # main micro-ROS app
    pico_uart_transport.c           # micro-ROS serial transport

    # MPU9250 HAL and Service files :
    HAL/MPU9250_HAL.cpp          # changed: added HAL cpp
    Services/MPU9250_Service.cpp # changed: added Service cpp
)

target_link_libraries(IMU_test
    pico_stdlib
    microros
    hardware_i2c                   # changed: add hardware i2c for MPU9250 communication
    hardware_gpio                  # changed: add gpio library (needed for sensors and LEDs)
)

target_include_directories(IMU_test PUBLIC
    libmicroros/include
    HAL                       # changed: add imu hal headers path
    Services                   # changed: add imu service headers path
)

# Compiler flags for size optimization (keep as is)
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico 
pico_enable_stdio_usb(IMU_test 1)   # changed: enable USB stdio for serial output
pico_enable_stdio_uart(IMU_test 0)  # changed: disable UART stdio to avoid conflict

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2 output for flashing
pico_add_extra_outputs(IMU_test)
