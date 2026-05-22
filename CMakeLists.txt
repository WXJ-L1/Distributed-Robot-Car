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
set(CMAKE_PROJECT_NAME First-car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/Src/UART.c
        User/Inc/UART.h
        User/Src/MAIN.c
        User/Src/servo.c
        User/Inc/servo.h
        User/Src/hcsr04.c
        User/Inc/hcsr04.h
        User/Src/linewalking.c
        User/Inc/linewalking.h
        User/Src/oled.c
        User/Inc/oled.h
        User/Src/font.c
        User/Inc/font.h
        User/Src/camera.c
        User/Inc/camera.h
        User/Inc/Motor.h
        User/Src/Motor.c
        User/Src/MAIN.c
        User/Src/turning.c
        User/Inc/mpu6050_turn.h
        User/Src/mpu6050_turn.c
        User/Src/MAIN.c
        User/Inc/buzzer.h
        User/Src/buzzer.c
        User/Inc/turning.h
        User/Src/map.c
        User/Inc/map.h
        User/Src/broadcast.c
        User/Inc/broadcast.h
        User/Src/groundtrue.c
        User/Inc/groundtrue.h
        User/Src/car_queue.c
        User/Inc/car_queue.h
        User/Src/MQTT.c
        User/Inc/MQTT.h

)

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
        ${CMAKE_SOURCE_DIR}/User/Inc

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
