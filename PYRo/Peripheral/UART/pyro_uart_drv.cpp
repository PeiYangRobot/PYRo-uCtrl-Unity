/**
 * @file PYRo_uart_drv.cpp
 * @brief Implementation file for the PYRO C++ UART Driver class.
 *
 * This file implements the `pyro::uart_drv_t` methods, including DMA buffer
 * allocation, transmission logic, and the critical static map mechanism
 * to link HAL ISR callbacks to the correct C++ driver instance.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"
#include "stm32h7xx_hal_dma.h"

#include <cstring>
#include <map>

#include "pyro_core_dma_heap.h"
#include "pyro_uart_drv.h"
#include "usart.h"

#include <stdexcept>

namespace pyro
{
/* Constructor and Destructor ------------------------------------------------*/
/**
 * @brief Constructor for the UART driver.
 *
 * Allocates two DMA-capable buffers, registers the instance in the static map,
 * and initializes state flags.
 */
uart_drv_t::uart_drv_t(UART_HandleTypeDef *huart, const uint16_t buf_length)
    : rx_buf{nullptr, nullptr}, _huart(huart)
{
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

/**
 * @brief Destructor.
 *
 * Frees DMA-allocated buffers and removes the instance from the static map.
 */
uart_drv_t::~uart_drv_t()
{
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
    uart_map().erase(_huart);
}

uart_drv_t *uart_drv_t::get_instance(const which_uart uart)
{
    switch (uart)
    {
        case uart1:
            static uart_drv_t uart_drv1(&huart1, 42);
            return &uart_drv1;
        case uart5:
            static uart_drv_t uart_drv5(&huart5, 36);
            return &uart_drv5;
        case uart7:
            static uart_drv_t uart_drv7(&huart7, 48);
            return &uart_drv7;
        case uart10:
            static uart_drv_t uart_drv10(&huart10, 64);
            return &uart_drv10;
        default:
            return nullptr;
    }
}
/* Static Map Management -----------------------------------------------------*/
/**
 * @brief Provides access to the static map linking HAL handles to driver
 * instances.
 */
std::map<UART_HandleTypeDef *, uart_drv_t *> &uart_drv_t::uart_map()
{
    static std::map<UART_HandleTypeDef *, uart_drv_t *> instance;
    return instance;
}

/* Transmission Methods ------------------------------------------------------*/
/**
 * @brief Blocking write using HAL polling. Updates state flags on
 * error/timeout.
 */
status_t uart_drv_t::write(const uint8_t *p, const uint16_t size,
                           const uint32_t waittime)
{
    const uint8_t ret = HAL_UART_Transmit(_huart, p, size, waittime);
    if (ret == HAL_OK)
    {
        return PYRO_OK;
    }
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

/**
 * @brief Non-blocking write using HAL DMA. Updates state flags on busy status.
 */
status_t uart_drv_t::write(const uint8_t *p, const uint16_t size)
{
    const uint8_t ret = HAL_UART_Transmit_DMA(_huart, p, size);
    if (ret == HAL_OK)
    {
        return PYRO_OK;
    }
    if (ret == HAL_BUSY)
    {
        state.tx_busy = 0x01U;
        return PYRO_BUSY;
    }
    return PYRO_ERROR;
}

/* Reception Control Methods -------------------------------------------------*/
/**
 * @brief Starts DMA reception, typically using ReceiveToIdle mode.
 *
 * It uses the currently selected RX buffer and disables the Half Transfer
 * interrupt, relying on the full RX Event (IDLE) interrupt.
 */
status_t uart_drv_t::enable_rx_dma()
{

    if (!state.init_flag)
    {
        return PYRO_ERROR;
    }
    uint8_t ret;
    ret = HAL_UARTEx_ReceiveToIdle_DMA(_huart, rx_buf[rx_buf_switch],
                                       _rx_buf_size);
    if (ret != HAL_OK)
    {
        state.rx_dma_enable = 0;
        if (ret == HAL_BUSY)
        {
            state.rx_busy = 0x01U;
        }
        else
        {
            state.rx_error = 0x01U;
        }
        return PYRO_ERROR;
    }
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
    state.rx_dma_enable = 1;
    state.rx_error      = 0;
    state.rx_busy       = 0;
    return PYRO_OK;
}

/**
 * @brief Aborts the ongoing DMA reception.
 */
status_t uart_drv_t::disable_rx_dma()
{
    if (HAL_OK != HAL_UART_AbortReceive(_huart))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/* Peripheral Management -----------------------------------------------------*/
/**
 * @brief Performs a full peripheral reset (DeInit -> Init).
 *
 * Clears all pending error flags and restarts DMA reception.
 */
status_t uart_drv_t::reset(uint32_t BaudRate, uint32_t WordLength,
                           uint32_t StopBits, uint32_t Parity)
{
    _huart->Init.BaudRate   = BaudRate;
    _huart->Init.WordLength = WordLength;
    _huart->Init.StopBits   = StopBits;
    _huart->Init.Parity     = Parity;

    if (HAL_OK != HAL_UART_DeInit(_huart))
    {
        return PYRO_ERROR;
    }
    if (HAL_OK != HAL_UART_Init(_huart))
    {
        return PYRO_ERROR;
    }
    __HAL_UART_CLEAR_FLAG(_huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                      UART_CLEAR_NEF | UART_CLEAR_OREF |
                                      UART_CLEAR_RTOF | UART_CLEAR_CMF |
                                      UART_CLEAR_WUF);
    if (PYRO_OK != enable_rx_dma())
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/* Custom RX Event Callback Management ---------------------------------------*/
/**
 * @brief Registers a custom C++ RX event callback with an owner ID.
 */
void uart_drv_t::add_rx_event_callback(const rx_event_func &func,
                                       uint32_t owner)
{
    rx_event_callback_t callback;
    callback.owner = owner;
    callback.func  = func;
    rx_event_callbacks.push_back(callback);
}

/**
 * @brief Removes a custom C++ RX event callback based on the owner ID.
 */
status_t uart_drv_t::remove_rx_event_callback(uint32_t owner)
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
/**
 * @brief Registers the HAL Rx Event Callback.
 */
status_t uart_drv_t::register_event_callback(
    const pUART_RxEventCallbackTypeDef pCallback)
{
    if (HAL_OK != HAL_UART_RegisterRxEventCallback(_huart, pCallback))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/**
 * @brief Unregisters the HAL Rx Event Callback.
 */
status_t uart_drv_t::unregister_event_callback()
{
    if (HAL_OK != HAL_UART_UnRegisterRxEventCallback(_huart))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/**
 * @brief Registers a standard HAL callback by ID (e.g., TX complete).
 */
status_t uart_drv_t::register_callback(const HAL_UART_CallbackIDTypeDef CB_ID,
                                       const pUART_CallbackTypeDef pCallback)
{
    if (HAL_OK != HAL_UART_RegisterCallback(_huart, CB_ID, pCallback))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/**
 * @brief Unregisters a standard HAL callback by ID.
 */
status_t uart_drv_t::unregister_callback(const HAL_UART_CallbackIDTypeDef CB_ID)
{
    if (HAL_OK != HAL_UART_UnRegisterCallback(_huart, CB_ID))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

} // namespace pyro

/* External HAL/ISR Callbacks ------------------------------------------------*/
/**
 * @brief HAL Extended RX Event Callback (triggered by DMA IDLE detection).
 *
 * This ISR-context function looks up the C++ driver instance and executes
 * registered C++ callbacks. If a callback consumes the data, the RX buffer
 * is switched and DMA reception is restarted. A FreeRTOS yield is performed
 * if a higher-priority task was woken.
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size)
{
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    static BaseType_t xHigherPriorityTaskWoken;
    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        const auto drv = it->second;
        for (auto &cb : drv->rx_event_callbacks)
        {
            if (cb.func(drv->rx_buf[drv->rx_buf_switch], Size,
                        xHigherPriorityTaskWoken))
            {
                drv->rx_buf_switch ^= 0x01U;
                break;
            }
        }
        drv->enable_rx_dma();
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief HAL UART Error Callback.
 *
 * This ISR-context function clears all pending error flags (Parity, Framing,
 * Overrun, etc.) and restarts DMA reception to recover the peripheral.
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        const auto drv = it->second;
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                         UART_CLEAR_NEF | UART_CLEAR_OREF |
                                         UART_CLEAR_RTOF);
        drv->enable_rx_dma();
    }
}