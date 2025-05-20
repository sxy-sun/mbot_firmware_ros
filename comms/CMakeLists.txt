# Add the comms library
add_library(comms STATIC
    src/dual_cdc.c
    src/pico_uart_transport.c
    src/usb_descriptors.c
)

# Link against required libraries
target_link_libraries(comms PUBLIC
    pico_stdlib
    pico_stdio_usb
    pico_multicore
    tinyusb_device
    tinyusb_board
)

# Include directories
target_include_directories(comms PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
    # ${PICO_SDK_PATH}/lib/tinyusb/src
    ${CMAKE_SOURCE_DIR}/libmicroros/include
)