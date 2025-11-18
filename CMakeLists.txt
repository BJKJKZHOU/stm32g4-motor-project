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
set(CMAKE_PROJECT_NAME stm32g4-motor-project-ex)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

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
    Core/Src/vofa_com_threadx.c
    Core/Src/test_threadx.c
    Core/Src/motor_control_threadx.c

    Communication/Src/Vofa.c
    Communication/Src/Vofa_STM32G474.c

    MotorControl/Src/FOC_math.c
    MotorControl/Src/motor_params.c
    MotorControl/Src/command.c
    MotorControl/Src/PID_controller.c
    MotorControl/Src/normalization.c
    MotorControl/Src/Positioning.c
    MotorControl/Src/FOC_Loop.c
    MotorControl/Src/Current.c

    Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_sin_cos_f32.c
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Communication/Inc
    MotorControl/Inc
    Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# Add post-build commands to generate BIN and HEX files
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE:${CMAKE_PROJECT_NAME}>.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE:${CMAKE_PROJECT_NAME}>.hex
)
add_custom_target(flash
    COMMAND STM32_Programmer_CLI -c port=SWD -d $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -hardRst
    DEPENDS ${CMAKE_PROJECT_NAME}
)

# Create J-Link flash script using add_custom_command to properly handle generator expressions
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "halt" > ${CMAKE_BINARY_DIR}/flash.jlink
    COMMAND ${CMAKE_COMMAND} -E echo "loadfile $<TARGET_FILE:${CMAKE_PROJECT_NAME}>" >> ${CMAKE_BINARY_DIR}/flash.jlink
    COMMAND ${CMAKE_COMMAND} -E echo "r" >> ${CMAKE_BINARY_DIR}/flash.jlink
    COMMAND ${CMAKE_COMMAND} -E echo "g" >> ${CMAKE_BINARY_DIR}/flash.jlink
    COMMAND ${CMAKE_COMMAND} -E echo "exit" >> ${CMAKE_BINARY_DIR}/flash.jlink
    COMMENT "Creating J-Link flash script"
    VERBATIM
)

# J-Link flash target for Windows PowerShell
add_custom_target(flash_jlink
    COMMAND powershell -Command "& 'JLink' -device STM32G474RE -if SWD -speed 4000 -CommanderScript ${CMAKE_BINARY_DIR}/flash.jlink"
    DEPENDS ${CMAKE_PROJECT_NAME}
)