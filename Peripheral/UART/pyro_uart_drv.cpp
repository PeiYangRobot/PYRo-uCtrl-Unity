/**
 * @file pyro_uart_drv.cpp
 * @brief Implementation file for the PYRO UART driver.
 * PYRO 串口驱动实现文件。
 *
 * @author Lucky
 * @version 1.2.0
 * @date 2026-03-25
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_uart_drv.h"
#include "dma.h"
#include "stm32h7xx_hal_dma.h"

#include <cstring>
#include <map>
#include <stdexcept>

#include "pyro_core_dma_heap.h"

namespace pyro
{
/* Constructor and Destructor ------------------------------------------------*/
uart_drv_t::uart_drv_t(UART_HandleTypeDef *huart, const uint16_t buf_length)
    : rx_buf{nullptr, nullptr}, _huart(huart)
{
    // Auto-register this instance to the static map for ISR routing.
    // 自动将此实例注册到静态映射表中，用于中断路由。
    uart_map()[huart] = this;

    rx_buf[0]         = static_cast<uint8_t *>(pvPortDmaMalloc(buf_length));
    rx_buf[1]         = static_cast<uint8_t *>(pvPortDmaMalloc(buf_length));

    if (rx_buf[0] && rx_buf[1])
    {
        state.init_flag = true;
        memset(rx_buf[0], 0, buf_length);
        memset(rx_buf[1], 0, buf_length);
        _rx_buf_size = buf_length;
    }
    rx_buf_switch = 0;
}

uart_drv_t::~uart_drv_t()
{
    // Free the DMA-allocated buffers.
    // 释放 DMA 分配的缓冲区。
    if (rx_buf[0])
    {
        vPortFree(rx_buf[0]);
        rx_buf[0] = nullptr;
    }
    if (rx_buf[1])
    {
        vPortFree(rx_buf[1]);
        rx_buf[1] = nullptr;
    }

    // Remove the instance from the static map.
    // 从静态映射表中移除该实例。
    uart_map().erase(_huart);
}

/* Static Map Management -----------------------------------------------------*/
std::map<UART_HandleTypeDef *, uart_drv_t *> &uart_drv_t::uart_map()
{
    static std::map<UART_HandleTypeDef *, uart_drv_t *> instance;
    return instance;
}

/* Transmission Methods ------------------------------------------------------*/
status_t uart_drv_t::write(const uint8_t *p, const uint16_t size,
                           const uint32_t waittime)
{
    const uint8_t ret = HAL_UART_Transmit(_huart, p, size, waittime);
    if (ret == HAL_OK)
        return PYRO_OK;
    if (ret == HAL_BUSY)
    {
        state.tx_busy = 0x01U;
        return PYRO_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        state.tx_timeout = 0x01U;
        return PYRO_TIMEOUT;
    }
    return PYRO_ERROR;
}

status_t uart_drv_t::write(const uint8_t *p, const uint16_t size)
{
    const uint8_t ret = HAL_UART_Transmit_DMA(_huart, p, size);
    if (ret == HAL_OK)
        return PYRO_OK;
    if (ret == HAL_BUSY)
    {
        state.tx_busy = 0x01U;
        return PYRO_BUSY;
    }
    return PYRO_ERROR;
}

/* Reception Control Methods -------------------------------------------------*/
__attribute__((section(".itcm_text"))) status_t uart_drv_t::enable_rx_dma()
{
    if (!state.init_flag)
        return PYRO_ERROR;

    // Start DMA reception with idle line detection.
    // 启动带有空闲线路检测的 DMA 接收。
    const uint8_t ret = HAL_UARTEx_ReceiveToIdle_DMA(
        _huart, rx_buf[rx_buf_switch], _rx_buf_size);

    if (ret != HAL_OK)
    {
        state.rx_dma_enable = 0;
        if (ret == HAL_BUSY)
            state.rx_busy = 0x01U;
        else
            state.rx_error = 0x01U;
        return PYRO_ERROR;
    }

    // Disable the DMA Half Transfer interrupt.
    // 关闭 DMA 半传输中断。
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
    state.rx_dma_enable = 1;
    state.rx_error      = 0;
    state.rx_busy       = 0;
    return PYRO_OK;
}

status_t uart_drv_t::disable_rx_dma()
{
    if (state.rx_dma_enable)
    {
        if (HAL_OK != HAL_UART_AbortReceive(_huart))
            return PYRO_ERROR;
        state.rx_dma_enable = 0;
        return PYRO_OK;
    }
    return PYRO_OK;
}

/* Peripheral Management -----------------------------------------------------*/
status_t uart_drv_t::reset(const uint32_t BaudRate, const uint32_t WordLength,
                           const uint32_t StopBits, const uint32_t Parity)
{
    if (disable_rx_dma() != PYRO_OK)
    {
        return PYRO_ERROR;
    }

    _huart->Init.BaudRate   = BaudRate;
    _huart->Init.WordLength = WordLength;
    _huart->Init.StopBits   = StopBits;
    _huart->Init.Parity     = Parity;

    if (HAL_OK != HAL_UART_DeInit(_huart))
        return PYRO_ERROR;
    if (HAL_OK != HAL_UART_Init(_huart))
        return PYRO_ERROR;

    // Clear all pending error flags.
    // 清除所有挂起的错误标志。
    __HAL_UART_CLEAR_FLAG(_huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                      UART_CLEAR_NEF | UART_CLEAR_OREF |
                                      UART_CLEAR_RTOF | UART_CLEAR_CMF |
                                      UART_CLEAR_WUF);
    return PYRO_OK;
}

/* Pin and Level Configuration -----------------------------------------------*/
status_t uart_drv_t::set_pin_swap(const bool enable)
{
    // 修改硬件配置前，必须先停止当前的接收动作
    if (disable_rx_dma() != PYRO_OK)
    {
        return PYRO_ERROR;
    }

    // 配置高级特性：标记我们要初始化引脚交换功能
    _huart->AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_SWAP_INIT;

    // 设置使能或禁用状态
    _huart->AdvancedInit.Swap = enable ? UART_ADVFEATURE_SWAP_ENABLE : UART_ADVFEATURE_SWAP_DISABLE;

    // 重新初始化串口使配置生效
    if (HAL_OK != HAL_UART_DeInit(_huart))
        return PYRO_ERROR;
    if (HAL_OK != HAL_UART_Init(_huart))
        return PYRO_ERROR;

    // 更新标志位
    state.pin_swapped = enable ? 1 : 0;

    return PYRO_OK;
}

status_t uart_drv_t::set_level_invert(const bool tx_invert, const bool rx_invert)
{
    // 修改硬件配置前，必须先停止当前的接收动作
    if (disable_rx_dma() != PYRO_OK)
    {
        return PYRO_ERROR;
    }

    // 配置高级特性：标记我们要初始化 TX 和 RX 的电平反转功能
    _huart->AdvancedInit.AdvFeatureInit |= (UART_ADVFEATURE_TXINVERT_INIT | UART_ADVFEATURE_RXINVERT_INIT);

    // 设置 TX 与 RX 是否反转电平
    _huart->AdvancedInit.TxPinLevelInvert = tx_invert ? UART_ADVFEATURE_TXINV_ENABLE : UART_ADVFEATURE_TXINV_DISABLE;
    _huart->AdvancedInit.RxPinLevelInvert = rx_invert ? UART_ADVFEATURE_RXINV_ENABLE : UART_ADVFEATURE_RXINV_DISABLE;

    // 重新初始化串口使配置生效
    if (HAL_OK != HAL_UART_DeInit(_huart))
        return PYRO_ERROR;
    if (HAL_OK != HAL_UART_Init(_huart))
        return PYRO_ERROR;

    // 只要有任意一个引脚发生了反转，标志位置 1
    state.level_inverted = (tx_invert || rx_invert) ? 1 : 0;

    return PYRO_OK;
}

/* Custom RX Event Callback Management ---------------------------------------*/
void uart_drv_t::add_rx_event_callback(const rx_event_func &func,
                                       const uint32_t owner)
{
    rx_event_callback_t callback;
    callback.owner = owner;
    callback.func  = func;
    rx_event_callbacks.push_back(callback);
}

status_t uart_drv_t::remove_rx_event_callback(const uint32_t owner)
{
    for (auto it = rx_event_callbacks.begin(); it != rx_event_callbacks.end();
         ++it)
    {
        if (it->owner == owner)
        {
            rx_event_callbacks.erase(it);
            return PYRO_OK;
        }
    }
    return PYRO_NOT_FOUND;
}

/* HAL Callback Registration -------------------------------------------------*/
status_t uart_drv_t::register_event_callback(
    const pUART_RxEventCallbackTypeDef pCallback) const
{
    if (HAL_OK != HAL_UART_RegisterRxEventCallback(_huart, pCallback))
        return PYRO_ERROR;
    return PYRO_OK;
}

status_t uart_drv_t::unregister_event_callback() const
{
    if (HAL_OK != HAL_UART_UnRegisterRxEventCallback(_huart))
        return PYRO_ERROR;
    return PYRO_OK;
}

status_t
uart_drv_t::register_callback(const HAL_UART_CallbackIDTypeDef CB_ID,
                              const pUART_CallbackTypeDef pCallback) const
{
    if (HAL_OK != HAL_UART_RegisterCallback(_huart, CB_ID, pCallback))
        return PYRO_ERROR;
    return PYRO_OK;
}

status_t
uart_drv_t::unregister_callback(const HAL_UART_CallbackIDTypeDef CB_ID) const
{
    if (HAL_OK != HAL_UART_UnRegisterCallback(_huart, CB_ID))
        return PYRO_ERROR;
    return PYRO_OK;
}

} // namespace pyro

/* External HAL/ISR Callbacks ------------------------------------------------*/

extern "C" __attribute__((section(".itcm_text"))) void
HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Find the C++ driver instance by its hardware handle.
    // 通过硬件句柄查找 C++ 驱动实例。
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    static BaseType_t xHigherPriorityTaskWoken;

    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        const auto drv = it->second;
        for (auto &cb : drv->rx_event_callbacks)
        {
            // Execute the callback and check if data was consumed.
            // 执行回调并检查数据是否已被处理。
            if (cb.func(drv->rx_buf[drv->rx_buf_switch], Size,
                        xHigherPriorityTaskWoken))
            {
                // Switch buffer upon successful consumption.
                // 成功处理后切换缓冲区。
                drv->rx_buf_switch ^= 0x01U;
                break;
            }
        }

        // Restart DMA reception for the next incoming packet.
        // 为下一个传入的数据包重新启动 DMA 接收。
        drv->enable_rx_dma();
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

extern "C" __attribute__((section(".itcm_text"))) void
HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        const auto drv = it->second;

        // Clear error flags and restart reception to recover the peripheral.
        // 清除错误标志并重新启动接收，以恢复外设。
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                         UART_CLEAR_NEF | UART_CLEAR_OREF |
                                         UART_CLEAR_RTOF);
        drv->enable_rx_dma();
    }
}

// 【新增】DMA 硬件发送完成中断回调
extern "C" __attribute__((section(".itcm_text"))) void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 通过硬件句柄查找对应的 C++ 驱动实例
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        const auto drv = it->second;
        drv->state.tx_busy = 0; // 清除忙碌状态

        // 触发已注册的 TX 完成回调（如果有）
        if (drv->_tx_cplt_callback)
        {
            drv->_tx_cplt_callback(xHigherPriorityTaskWoken);
        }
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}