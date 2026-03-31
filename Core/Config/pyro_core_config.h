/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:56:04
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-28 16:02:41
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#ifndef __PYRO_CORE_CONFIG_H__
#define __PYRO_CORE_CONFIG_H__

// #define DEMO_MODE 1
// #define DEBUG_MODE 0

#if DEMO_MODE

#define RC_DEMO_EN         0
#define MOTOR_DEMO_EN      0
#define CONTROLLER_DEMO_EN 0
#define CONTROL_DEMO_EN    0
#define IMU_DEMO_EN        0
#define REFEREE_DEMO_EN    0

#endif

#if DEBUG_MODE

#define VOFA_DEBUG_EN 0
#define JCOM_DEBUG_EN 0

#endif

// #define IMU_CALIBRATION_EN 0
//
// #define TEST_ROBOT_ID 0
// #define HERO_ID   1
// #define SUB_HERO_ID 10
// #define ENGINEER_ID 2
// #define SUB_ENGINEER_ID 20
// #define INFANTRY1_ID 3
// #define INFANTRY2_ID 4
// #define SUB_INFANTRY_ID 30
// #define SENTRY_ID 5
// #define SUB_SENTRY_ID 6
// #define UAV_ID 7
// #define DARTS_ID 8
// #define RADAR_ID 9


#if ROBOT_ID == SENTRY_ID

#define GIMBAL_ID  1
#define CHASSIS_ID 2

#define BOARD_ID   GIMBAL_ID
#endif
#if (ROBOT_ID == HERO_ID) || (ROBOT_ID == SUB_HERO_ID)
#define GIMBAL_ID  1
#define CHASSIS_ID 2

#define BOARD_ID   GIMBAL_ID
#endif

#define PYRO_UART1 pyro::bsp_uart::get_uart1()
#define PYRO_UART5 pyro::bsp_uart::get_uart5()
#define PYRO_UART7 pyro::bsp_uart::get_uart7()
#define PYRO_UART10 pyro::bsp_uart::get_uart10()

#define DR16_UART PYRO_UART5
#define VT03_UART PYRO_UART1
#define REFEREE_UART PYRO_UART10
#define SUPERCAP_UART PYRO_UART7

#define VOFA_DEBUG_PORT PYRO_UART10
#define JCOM_DEBUG_PORT PYRO_UART7


#endif // PYRO_PYRO_CORE_CONFIG_H