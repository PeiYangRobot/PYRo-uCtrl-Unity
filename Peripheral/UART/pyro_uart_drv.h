/**
 * @file pyro_uart_drv.h
 * @brief Header file for the PYRO UART driver.
 * PYRO 串口驱动头文件。
 *
 * This file encapsulates the STM32 HAL UART functionality, including DMA
 * double-buffering for reception and FreeRTOS integration.
 * 本文件封装了 STM32 HAL 串口功能，包含用于接收的 DMA 双缓冲以及
 * FreeRTOS 集成。
 *
 * @author Lucky
 * @version 1.2.0
 * @date 2026-03-25
 */

#ifndef __PYRO_UART_DRV_H__
#define __PYRO_UART_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

#include "pyro_core_def.h"

#include "FreeRTOS.h"

#include <functional>
#include <map>
#include <vector>

namespace pyro
{

// Forward declaration of the BSP manager class.
// 前向声明 BSP 管理类。
class bsp_uart;

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief UART driver class.
 * 串口驱动类。
 */
class uart_drv_t
{
    // Declare bsp_uart as a friend to allow calling the private constructor.
    // 声明 bsp_uart 为友元，允许其调用私有构造函数。
    friend class bsp_uart;

    /* Private Types ---------------------------------------------------------*/
    /**
     * @brief RX event callback signature.
     * 接收事件回调函数签名。
     *
     * @param p Pointer to the received data buffer.
     * 接收数据缓冲区的指针。
     * @param size Size of the received data.
     * 接收数据的大小。
     * @param xHigherPriorityTaskWoken Flag to request a context switch.
     * 请求上下文切换的标志。
     * @return true if the data is consumed and buffer should switch.
     * 如果数据已被处理且需要切换缓冲区，则返回 true。
     */
    using rx_event_func = std::function<bool(
        uint8_t *p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken)>;

    // 【新增】定义 TX 发送完成的事件回调函数类型
    using tx_cplt_func = std::function<void(BaseType_t& xHigherPriorityTaskWoken)>;

    /**
     * @brief Structure to store registered RX callbacks.
     * 用于存储已注册接收回调的结构体。
     */
    typedef struct rx_event_callback_t
    {
        uint32_t owner;
        rx_event_func func;
    } rx_event_callback_t;

    /**
     * @brief Internal state flags for tracking driver status.
     * 用于跟踪驱动状态的内部标志位。
     */
    typedef struct state_t
    {
        volatile uint8_t init_flag      : 1;
        volatile uint8_t tx_busy        : 1;
        volatile uint8_t tx_timeout     : 1;
        volatile uint8_t rx_dma_enable  : 1;
        volatile uint8_t rx_busy        : 1;
        volatile uint8_t rx_error       : 1;
        // 【新增】引脚与电平翻转状态标志
        volatile uint8_t pin_swapped    : 1;
        volatile uint8_t level_inverted : 1;
    } state_t;

public:
    /* Public Methods - De-initialization ------------------------------------*/
    /**
     * @brief Destructor.
     * 析构函数。
     */
    ~uart_drv_t();

    /* Public Methods - Peripheral Management --------------------------------*/
    /**
     * @brief Resets and re-initializes the UART peripheral.
     * 复位并重新初始化串口外设。
     */
    status_t reset(uint32_t BaudRate, uint32_t WordLength, uint32_t StopBits,
                   uint32_t Parity);

    /**
     * @brief 设置是否交换 TX 和 RX 引脚。
     * @param enable true 交换，false 恢复默认
     * @return 状态码
     */
    status_t set_pin_swap(bool enable);

    /**
     * @brief 设置是否反转 TX 和 RX 引脚的有效电平。
     * @param tx_invert true 反转 TX 电平
     * @param rx_invert true 反转 RX 电平
     * @return 状态码
     */
    status_t set_level_invert(bool tx_invert, bool rx_invert);

    /* Public Methods - Transmission -----------------------------------------*/
    /**
     * @brief Blocking write using HAL polling.
     * 使用 HAL 轮询方式的阻塞写操作。
     */
    status_t write(const uint8_t *p, uint16_t size, uint32_t waittime);

    /**
     * @brief Non-blocking write using HAL DMA.
     * 使用 HAL DMA 方式的非阻塞写操作。
     */
    status_t write(const uint8_t *p, uint16_t size);

    /* Public Methods - Reception Control ------------------------------------*/
    /**
     * @brief Starts DMA reception in ReceiveToIdle mode.
     * 启动 ReceiveToIdle 模式的 DMA 接收。
     */
    status_t enable_rx_dma();

    /**
     * @brief Aborts the ongoing DMA reception.
     * 终止正在进行的 DMA 接收。
     */
    status_t disable_rx_dma();

    /* Public Methods - Custom Callback Management ---------------------------*/
    /**
     * @brief Adds a custom C++ RX event callback.
     * 添加自定义的 C++ 接收事件回调。
     */
    void add_rx_event_callback(const rx_event_func &func, uint32_t owner);

    /**
     * @brief Removes a custom C++ RX event callback by its owner ID.
     * 根据所有者 ID 移除自定义的 C++ 接收事件回调。
     */
    status_t remove_rx_event_callback(uint32_t owner);

    // 【新增】注册 TX 发送完成回调
    void set_tx_cplt_callback(const tx_cplt_func &func) {
        _tx_cplt_callback = func;
    }

    /* Public Methods - HAL Callback Registration ----------------------------*/
    status_t register_event_callback(pUART_RxEventCallbackTypeDef pCallback) const;
    status_t unregister_event_callback() const;
    status_t register_callback(HAL_UART_CallbackIDTypeDef CB_ID,
                               pUART_CallbackTypeDef pCallback) const;
    status_t unregister_callback(HAL_UART_CallbackIDTypeDef CB_ID) const;

    /* Public Methods - Static Access ----------------------------------------*/
    /**
     * @brief Provides access to the static map for ISR routing.
     * 提供对静态映射表的访问，用于中断路由。
     */
    static std::map<UART_HandleTypeDef *, uart_drv_t *> &uart_map();

    /* Public Members - State/Data -------------------------------------------*/

    // List of registered RX callbacks.
    // 已注册的接收回调列表。
    std::vector<rx_event_callback_t> rx_event_callbacks;
    // 【新增】保存 TX 发送完成的回调函数
    tx_cplt_func _tx_cplt_callback = nullptr;

    // Double buffers for DMA reception.
    // 用于 DMA 接收的双缓冲区。
    uint8_t *rx_buf[2];

    // Index of the currently active RX buffer.
    // 当前激活的接收缓冲区索引。
    uint8_t rx_buf_switch{};

    // Internal state tracking.
    // 内部状态跟踪。
    state_t state{};

private:
    /* Private Constructor ---------------------------------------------------*/
    /**
     * @brief Private constructor, accessible only by the BSP class.
     * 私有构造函数，仅允许 BSP 类访问。
     *
     * @param huart Pointer to the HAL UART handle.
     * HAL 串口句柄指针。
     * @param buf_length Size of each RX buffer.
     * 单个接收缓冲区的大小。
     */
    explicit uart_drv_t(UART_HandleTypeDef *huart, uint16_t buf_length);

    /* Private Members -------------------------------------------------------*/

    // HAL handle for the peripheral.
    // 外设的 HAL 句柄。
    UART_HandleTypeDef *_huart;

    // Size of each RX buffer.
    // 单个接收缓冲区的大小。
    uint16_t _rx_buf_size{};

};

} // namespace pyro

#endif // __PYRO_UART_DRV_H__