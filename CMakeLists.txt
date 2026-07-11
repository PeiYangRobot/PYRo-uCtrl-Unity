cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# 1. 项目定义
# ==============================================================================
# 子模块直接继承上级配置，只需声明库目标
add_library(PYRo OBJECT)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()



# ==============================================================================
# 5. 源文件配置 (按分类排序)
# ==============================================================================
target_sources(PYRo PRIVATE
        Core/ETL/map.cpp
        Core/Lock/pyro_mutex.cpp
        Core/Lock/pyro_rw_lock.cpp
        Core/Task/pyro_task.cpp
        Core/Memory/pyro_core_dma_heap.c
        Core/Memory/pyro_core_mem.cpp
        Core/Task/pyro_task.cpp

        # --- Peripheral ---
        Peripheral/CAN/pyro_can_drv.cpp
        Peripheral/CAN/pyro_bsp_can.cpp
        Peripheral/DWT/pyro_dwt_drv.cpp
        Peripheral/UART/pyro_uart_drv.cpp
        Peripheral/UART/pyro_bsp_uart.cpp

        # --- Algorithm ---
        Algorithm/Kinematics/pyro_kin_hybrid.cpp
        Algorithm/Kinematics/pyro_kin_mec.cpp
        Algorithm/Kinematics/pyro_kin_rudder.cpp
        Algorithm/Kinematics/pyro_kin_wl.cpp
        Algorithm/OLS/pyro_algo_ols.cpp
        Algorithm/PID/pyro_algo_pid.cpp
        Algorithm/EKF/QuaternionEKF.cpp
        Algorithm/KF/kalman_filter.cpp
        Algorithm/Common/pyro_algo_common.cpp
        Algorithm/CRC/PYRo_crc.cpp
        Algorithm/Powercontrol/pyro_power_control.cpp
        Algorithm/Navigation/pyro_nav_hub.cpp

        # --- Component ---
        Component/Motor/pyro_dji_motor_drv.cpp
        Component/Motor/pyro_dm_motor_drv.cpp
        Component/Motor/pyro_motor_base.cpp
        Component/Powermeter/pyro_powermeter.cpp
        Component/RC/pyro_dr16_rc_drv.cpp
        Component/RC/pyro_rc_base_drv.cpp
        Component/RC/pyro_vt03_rc_drv.cpp
        Component/Referee/fifo.c
        Component/Referee/pyro_referee.cpp
        Component/Image/pyro_image_drv.cpp
        Component/INS/pyro_ins.cpp
        Component/Supercap/pyro_supercap_drv.cpp
        Component/SR/pyro_sonic_base_drv.cpp
        Component/SR/pyro_sr04_drv.cpp
        Component/SR/pyro_sr05_drv.cpp
        Component/SR/pyro_us100_drv.cpp

        # --- Debug ---
        Debug/Debug_task.cpp
        Debug/JCOM/pyro_jcom.cpp
        Debug/VOFA/pyro_vofa.cpp

        # --- Device ---
        Device/BMI088/BMI088_driver.cpp
        Device/BMI088/BMI088_middleware.cpp
        Component/Referee/UI/pyro_ui_drv.cpp


)


# ==============================================================================
# 6. 包含路径与链接 (按路径排序)
# ==============================================================================
target_include_directories(PYRo PUBLIC
        # --- Core ---
        Core/Config
        Core/Def
        Core/ETL
        Core/FSM
        Core/Lock
        Core/Memory
        Core/Task

        # --- Peripheral ---
        Peripheral/CAN
        Peripheral/DWT
        Peripheral/UART

        # --- Device ---
        Device/BMI088

        # --- Algorithm ---
        Algorithm/Common
        Algorithm/Kinematics
        Algorithm/OLS
        Algorithm/PID
        Algorithm/EKF
        Algorithm/KF
        Algorithm/LESO
        Algorithm/CRC
        Algorithm/Powercontrol
        Algorithm/Navigation

        # --- Component ---
        Component/IMU
        Component/INS
        Component/Motor
        Component/Powermeter
        Component/RC
        Component/Referee
        Component/Referee/UI
        Component/Shoot
        Component/Supercap
        Component/SR
        Component/Image

        # --- Module ---
        Module

        # --- Debug ---
        Debug/VOFA
        Debug/JCOM
)


target_link_libraries(PYRo PUBLIC stm32cubemx dsppp_lib)
