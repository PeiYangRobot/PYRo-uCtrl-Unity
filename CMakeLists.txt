cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# 1. 编译器检测配置 (必须放在 project 命令之前)
# ==============================================================================
set(CMAKE_SYSTEM_NAME Generic)
# 解决 _exit/_sbrk 缺失导致的 CMake 配置失败
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

include(cmake/gcc-arm-none-eabi.cmake)

# ==============================================================================
# 2. 项目定义
# ==============================================================================
set(CMAKE_PROJECT_NAME PYRo)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# ==============================================================================
# 3. 硬件架构与编译优化 (STM32H723)
# ==============================================================================
set(CPU_FLAGS
        "-mcpu=cortex-m7"
        "-mthumb"
        "-mfpu=fpv5-d16"
        "-mfloat-abi=hard"
)

add_compile_options(
        ${CPU_FLAGS}
        "-Ofast"
        "-Wdouble-promotion"
        "-funsafe-math-optimizations"
        "-Wall"
        "-fdata-sections"
        "-ffunction-sections"
)

# 【关键修改】这里移除了 --specs=nano.specs 和 --specs=nosys.specs
# 因为 add_subdirectory(cmake/stm32cubemx) 内部已经添加了它们。
# 重复添加会导致 "attempt to rename spec 'link'" 错误。
add_link_options(
        ${CPU_FLAGS}
        "-Wl,--gc-sections"
        "-Wl,-Map=${CMAKE_PROJECT_NAME}.map"
        "-Wl,--print-memory-usage"
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

# ==============================================================================
# 4. CMSIS-DSP & dsppp 库引入
# ==============================================================================
set(DSP_REPO "${CMAKE_CURRENT_SOURCE_DIR}/Ported_third_lib/CMSIS-DSP")

add_compile_definitions(
        ARM_MATH_CM7
        ARM_MATH_LOOPUNROLL
)

set(CMSISDSP_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)

# 添加 CMSIS-DSP 子工程
add_subdirectory(${DSP_REPO} build_cmsis_dsp)

# 【关键保留】手动修复 CMSIS-Core 路径，解决 arm_math_types.h 找不到 cmsis_compiler.h 的问题
target_include_directories(CMSISDSP PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include"
)

# 配置 dsppp 接口
add_library(dsppp_lib INTERFACE)
target_include_directories(dsppp_lib INTERFACE
        ${DSP_REPO}/dsppp/Include
        ${DSP_REPO}/Include
)

# ==============================================================================
# 5. 源文件配置
# ==============================================================================
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

# Add STM32CubeMX generated sources (这会自动引入 specs 文件)
add_subdirectory(cmake/stm32cubemx)

# ==============================================================================
# 6. 包含路径与链接
# ==============================================================================
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

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        CMSISDSP
        dsppp_lib
        m
)