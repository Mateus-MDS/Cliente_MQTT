# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Cliente_MQTT C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Cliente_MQTT Cliente_MQTT.c inc/ssd1306.c)

pico_set_program_name(Cliente_MQTT "Cliente_MQTT")
pico_set_program_version(Cliente_MQTT "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Cliente_MQTT 0)
pico_enable_stdio_usb(Cliente_MQTT 1)

pico_generate_pio_header(Cliente_MQTT ${CMAKE_CURRENT_LIST_DIR}/extra/animacoes_led.pio)

# Add the standard library to the build
target_link_libraries(Cliente_MQTT
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_i2c
    hardware_pio
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    pico_mbedtls
    pico_lwip_mbedtls
    pico_unique_id
    )

# Add the standard include files to the build
target_include_directories(Cliente_MQTT PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/extra
        ${CMAKE_CURRENT_SOURCE_DIR}/mqtt
)

# Add any user requested libraries
target_link_libraries(Cliente_MQTT 
        )

pico_add_extra_outputs(Cliente_MQTT)


