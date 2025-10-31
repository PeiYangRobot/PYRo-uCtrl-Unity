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
set(CMAKE_PROJECT_NAME PYRo)

include(cmake/gcc-arm-none-eabi.cmake)
# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        PYRo/Core/Memory/pyro_core_mem.cpp
        PYRo/Core/Memory/pyro_core_dma_heap.c
        PYRo/Core/Lock/pyro_rw_lock.cpp
        PYRo/Core/ETL/map.cpp
        PYRo/Core/Lock/pyro_rw_lock.cpp

        PYRo/Peripheral/CAN/pyro_can_drv.cpp
        PYRo/Peripheral/UART/pyro_uart_drv.cpp

        PYRo/Component/RC/pyro_rc_base_drv.cpp
        PYRo/Component/RC/pyro_dr16_rc_drv.cpp
        PYRo/Component/RC/pyro_rc_hub.cpp

        PYRo/Component/Motor/pyro_dji_motor_drv.cpp
        PYRo/Component/Motor/pyro_dm_motor_drv.cpp
        PYRo/Component/Motor/pyro_motor_base.cpp
        PYRo/Component/Chassis/pyro_wheel_drv.cpp
        PYRo/Component/Chassis/pyro_steering_wheel_drv.cpp
        PYRo/Component/Chassis/pyro_chassis_drv.cpp
        PYRo/Component/Gimbal/pyro_gimbal_drv.cpp
        PYRo/Component/Gimbal/pyro_yaw_drv.cpp
        PYRo/Component/Pid/pyro_pid_ctrl.cpp
        PYRo/Component/Controller/pyro_velocity_controller.cpp
        PYRo/Component/Controller/pyro_position_controller.cpp

        PYRo/Component/Shoot/pyro_fric_drv.cpp
        PYRo/Component/Shoot/pyro_trigger_drv.cpp
        PYRo/Component/Shoot/pyro_shoot_base.cpp
        
        PYRo/Application/Demo/pyro_rc_demo.cpp
        PYRo/Application/Demo/demo_task.cpp
        PYRo/Application/Demo/pyro_motor_demo.cpp
        PYRo/Application/Demo/pyro_wheel_demo.cpp
        PYRo/Application/Demo/pyro_control_demo.cpp
        PYRo/Application/Demo/pyro_controller_demo.cpp
        PYRo/Application/Demo/pyro_shoot_demo.cpp

        PYRo/Debug/Debug_task.cpp
        PYRo/Debug/VOFA/pyro_vofa.cpp
        
        PYRo/Debug/JCOM/pyro_jcom.cpp
        PYRo/Application/mission/pyro_init_thread.cpp
        PYRo/Application/Mission/pyro_mission_planer.cpp

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
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    # Add user defined include paths
    PYRo/Core/Def
    PYRo/Core/Memory
    PYRo/Core/Config
    PYRo/Core/ETL
    PYRo/Core/Lock
    PYRo/Peripheral/UART
    PYRo/Peripheral/CAN
    PYRo/Component/RC
    PYRo/Component/Motor
    PYRo/Component/Pid
    PYRo/Component/Chassis
    PYRo/Component/Gimbal
    PYRo/Component/Controller
    PYRo/Component/Shoot
    PYRo/Application/Demo
    PYRo/Debug/VOFA
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
