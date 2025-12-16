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

set(CPU_FLAGS
    "-mcpu=cortex-m7"
    "-mthumb"
    "-mfpu=fpv5-d16"
    "-mfloat-abi=hard"
)

#add_compile_options(
#    ${CPU_FLAGS}
#    "-Ofast"
#    "-Wdouble-promotion"
#    "-funsafe-math-optimizations"
#    "-Wall"
#)

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
        PYRo/Peripheral/DWT/pyro_dwt_drv.cpp

        PYRo/Algorithm/OLS/pyro_algo_ols.cpp
        PYRo/Algorithm/PID/pyro_algo_pid.cpp

        PYRo/Component/RC/pyro_rc_base_drv.cpp
        PYRo/Component/RC/pyro_vt03_rc_drv.cpp
        PYRo/Component/RC/pyro_dr16_rc_drv.cpp
        PYRo/Component/RC/pyro_rc_hub.cpp

        PYRo/Component/Motor/pyro_dji_motor_drv.cpp
        PYRo/Component/Motor/pyro_dm_motor_drv.cpp
        PYRo/Component/Motor/pyro_motor_base.cpp

        PYRo/Component/CRC/PYRo_crc.cpp

        PYRo/Component/IMU/AHRS.c
        PYRo/Component/IMU/BMI088driver.c
        PYRo/Component/IMU/BMI088Middleware.c
        PYRo/Component/IMU/IMU_Base.c
        PYRo/Component/IMU/IMU_Ext.c
        PYRo/Component/IMU/MATH_LIB.c
        PYRo/Component/IMU/PID.c

        PYRo/Component/Shoot/pyro_fric_drv.cpp
        PYRo/Component/Shoot/pyro_trigger_drv.cpp
        PYRo/Component/Shoot/pyro_shoot_base.cpp
        PYRo/Component/Shoot/pyro_shoot_17mm_control.cpp
        
        PYRo/Application/Demo/pyro_rc_demo.cpp
        PYRo/Application/Demo/demo_task.cpp
        PYRo/Application/Demo/pyro_motor_demo.cpp
        PYRo/Application/Demo/pyro_control_demo.cpp
        PYRo/Application/Demo/pyro_controller_demo.cpp
        PYRo/Application/Demo/pyro_shoot_demo.cpp

        PYRo/Debug/Debug_task.cpp
        PYRo/Debug/VOFA/pyro_vofa.cpp
        PYRo/Debug/JCOM/pyro_jcom.cpp

        PYRo/Application/Mission/pyro_mission_planer.cpp
        PYRo/Application/Mission/pyro_init_thread.cpp
        PYRo/Component/Referee/CRC8_CRC16.c
        PYRo/Component/Referee/fifo.c
        PYRo/Component/Referee/referee.c
        PYRo/Component/Referee/referee_drv.cpp
        PYRo/Component/Referee/referee_usart_task.c

        PYRo/Component/Powercontrol/pyro_power_control_drv.cpp

        PYRo/Component/Powermeter/pyro_powermeter.cpp
        PYRo/Moudle/Chassis/pyro_chassis_base.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_mec.cpp
        PYRo/Moudle/Chassis/Mecanum/pyro_mec_chassis.cpp
        PYRo/Core/Lock/pyro_mutex.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_rudder.cpp
        PYRo/Moudle/Chassis/Rudder/pyro_rud_chassis.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_hybrid.cpp
        PYRo/Moudle/Chassis/Hybrid/pyro_hybrid_chassis.cpp

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
    PYRo/Core/FSM

    PYRo/Peripheral/UART
    PYRo/Peripheral/CAN
    PYRo/Peripheral/DWT

    PYRo/Algorithm/OLS
    PYRo/Algorithm/PID
    PYRo/Algorithm/Kinematics

    PYRo/Component/RC
    PYRo/Component/Motor

    PYRo/Component/CRC
    PYRo/Component/Shoot
    PYRo/Component/IMU
    PYRo/Component/Referee
    PYRo/Component/Powermeter
    PYRo/Component/Powercontrol

    PYRo/Application/Demo

    PYRo/Debug/VOFA

    PYRo/Moudle/Chassis
    PYRo/Moudle/Chassis/Mecanum
    PYRo/Moudle/Chassis/Rudder
    PYRo/Moudle/Chassis/Hybrid

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
