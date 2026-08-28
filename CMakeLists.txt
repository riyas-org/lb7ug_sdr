cmake_minimum_required(VERSION 3.13)

if (NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH "$ENV{HOME}/pico-sdk")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)


project(usb_audio_sdr C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(${PROJECT_NAME}
    main.c
    usb_descriptors.c
    uac2_app.c
    cdc_app.c
    wm8731.c
    si5351.c
    cat_app.c
)
# Add this line so the compiler can find tusb_config.h
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
# Generate PIO headers
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)
# Add the pio header generation
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)


# Pull in common dependencies
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    pico_multicore
    tinyusb_device
    tinyusb_board
    hardware_i2c
    hardware_dma
    hardware_pio
    hardware_pwm
    hardware_clocks
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# Compiler definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_AUDIO_I2S_DATA_PIN=${WM8731_SDO_PIN}
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=${WM8731_BCLK_PIN}
)

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-function
)
