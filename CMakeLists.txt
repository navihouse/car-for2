cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME C10B_Test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Modules/motor.c
    Core/Src/Modules/encoder.c
    Core/Src/Modules/speed_control.c
    Core/Src/Modules/button.c
    Core/Src/Modules/oled.c
    Core/Src/Modules/line_sensor.c
    Core/Src/Modules/line_calibration.c
    Core/Src/Modules/line_calibration_store.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Mission
    Core/Src/Modules
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Modules/imu_heading.c
    Core/Src/Modules/imu_mpu6050.c
)
set(MISSION_MODE "FULL" CACHE STRING "Route to build: FULL, AC, CB, BD, DA")
set_property(CACHE MISSION_MODE PROPERTY STRINGS FULL AC CB BD DA)
set(_mission_modes FULL AC CB BD DA)
list(FIND _mission_modes "${MISSION_MODE}" _mission_mode_id)
if(_mission_mode_id EQUAL -1)
    message(FATAL_ERROR "MISSION_MODE must be FULL, AC, CB, BD or DA")
endif()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE MISSION_BUILD_MODE=${_mission_mode_id})
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Mission/mission.c
    Core/Src/Mission/mission_control.c
    Core/Src/Mission/mission_port_stm32.c
    Core/Src/Mission/motion_control.c
    Core/Src/Mission/motion_distance.c
    Core/Src/Mission/motion_edge.c
    Core/Src/Mission/motion_straight.c
    Core/Src/Mission/motion_turn.c
    Core/Src/Mission/motion_line.c
    Core/Src/Mission/motion_drive.c
    Core/Src/Mission/segment_ac.c
    Core/Src/Mission/segment_cb.c
    Core/Src/Mission/segment_bd.c
    Core/Src/Mission/segment_da.c
)

# Intel HEX + raw binary next to the .elf, so STM32_Programmer_CLI can flash it
# (scripts/flash_mission.ps1). Uses the toolchain objcopy already on PATH.
set(_mission_objcopy "${CMAKE_OBJCOPY}")
if(NOT _mission_objcopy)
    find_program(_mission_objcopy arm-none-eabi-objcopy)
endif()
if(_mission_objcopy)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND "${_mission_objcopy}" -O ihex
                "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>"
                "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex"
        COMMAND "${_mission_objcopy}" -O binary
                "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>"
                "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin"
        COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
        VERBATIM)
else()
    message(WARNING "arm-none-eabi-objcopy not found on PATH: no .hex/.bin generated")
endif()
