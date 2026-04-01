/**
* @file pyro_bsp_uart.h
 * @brief Board Support Package for UART instances.
 * 串口板级支持包头文件。
 *
 * Provides singleton interfaces for specific UART hardware instances, ensuring
 * safe initialization and isolation from the core driver logic.
 * 提供特定串口硬件实例的单例接口，确保安全的初始化以及与核心驱动逻辑的隔离。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-03-25
 */

#ifndef __PYRO_BSP_UART_H__
#define __PYRO_BSP_UART_H__

#include "pyro_uart_drv.h"

namespace pyro
{
/**
 * @brief BSP manager class for UART.
 * 串口的 BSP 管理类。
 */
class bsp_uart
{
public:
    /* Instance Getters ------------------------------------------------------*/

    /**
     * @brief Gets the singleton reference for UART1.
     * 获取 UART1 的单例引用。
     */
    static uart_drv_t& get_uart1();

    /**
     * @brief Gets the singleton reference for UART5.
     * 获取 UART5 的单例引用。
     */
    static uart_drv_t& get_uart5();

    /**
     * @brief Gets the singleton reference for UART7.
     * 获取 UART7 的单例引用。
     */
    static uart_drv_t& get_uart7();

    /**
     * @brief Gets the singleton reference for UART10.
     * 获取 UART10 的单例引用。
     */
    static uart_drv_t& get_uart10();

    /* Initialization --------------------------------------------------------*/

    /**
     * @brief Initializes all hardware UART instances.
     * 初始化所有硬件串口实例。
     */
    static void init_all();
};

} // namespace pyro

#endif // __PYRO_BSP_UART_H__