cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH "D:/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(flood_monitoring C CXX ASM)

pico_sdk_init()

include_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME} flood_monitoring.c lib/ssd1306.c)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)


pico_set_program_name(${PROJECT_NAME} "flood_monitoring")
pico_set_program_version(${PROJECT_NAME} "0.1")

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        hardware_i2c
        hardware_adc
        hardware_pio
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        hardware_pwm
        hardware_clocks
        )

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(${PROJECT_NAME})

