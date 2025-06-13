cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES "${CMAKE_CURRENT_SOURCE_DIR}/drivers")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nau7802-test-delete-me)

# Add your source files here
target_sources(app PRIVATE
    src/main.c            # Main application file
    src/shell/shell_commands.c  # Add shell command source file here
    src/calibration/calibration.c
    src/ble/ble_service.c
    src/ble/myble_lbs.c
    src/nvs/nvs.c
    src/sensor/appcall_nau7802.c
)

# Include directories for header files
target_include_directories(app PRIVATE
    src
    drivers     
)
