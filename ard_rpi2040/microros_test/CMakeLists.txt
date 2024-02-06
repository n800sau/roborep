cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_nema_motors C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(micro_ros_raspberrypi_pico_sdk/libmicroros)
add_executable(pico_nema_motors
    pico_nema_motors.c
    micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c
)
target_link_libraries(pico_nema_motors
    pico_stdlib
    microros
)

target_include_directories(pico_nema_motors PUBLIC
    micro_ros_raspberrypi_pico_sdk/libmicroros/include
    micro_ros_raspberrypi_pico_sdk
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections -Wno-unused-result")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections -Wno-unused-result")

# Configure Pico
pico_enable_stdio_usb(pico_nema_motors 1)
pico_enable_stdio_uart(pico_nema_motors 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
