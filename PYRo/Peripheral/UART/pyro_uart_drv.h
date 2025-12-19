/**
 * @file pyro_uart_drv.h
 * @brief Header file for the PYRO C++ UART Driver class.
 *
 * This file defines the `pyro::uart_drv_t` class, which encapsulates the
 * STM32 HAL UART functionality, including DMA double-buffering for reception
 * and integration with FreeRTOS for asynchronous event handling.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_UART_DRV_H__
#define __PYRO_UART_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

#include "pyro_core_def.h"

#include "FreeRTOS.h"

#include "functional"
#include "map"
#include "vector"

namespace pyro
{

class uart_drv_t;
/* Class Definition ----------------------------------------------------------*/
/**
 * @brief C++ class to encapsulate the STM32 HAL UART driver functionality.
 *
 * It manages double-buffering for DMA reception, integrates with FreeRTOS
 * for yielding from ISRs, and uses a static map to dispatch HAL callbacks
 * to the correct C++ instance.
 *
 * This class uses a Singleton pattern via `get_instance()` for access.
 */
class uart_drv_t
{

    /* Private Types ---------------------------------------------------------*/
    /**
     * @brief Type alias for the RX event callback signature (for ISR context).
     * @return true if the data was consumed and the RX buffer should switch.
     */
    using rx_event_func = std::function<bool(
        uint8_t *p, uint16_t size, BaseType_t xHigherPriorityTaskWoken)>;

    /**
     * @brief Structure to store registered RX callbacks with an owner ID.
     */
    typedef struct rx_event_callback_t
    {
        uint32_t owner;
        rx_event_func func;
    } rx_event_callback_t;

    /**
     * @brief Internal state flags (bit-field) for tracking driver status.
     */
    typedef struct state_t
    {
        volatile uint8_t init_flag     : 1;
        volatile uint8_t tx_busy       : 1;
        volatile uint8_t tx_timeout    : 1;
        volatile uint8_t rx_dma_enable : 1;
        volatile uint8_t rx_busy       : 1;
        volatile uint8_t rx_error      : 1;
    } state_t;

  public:
    /**
     * @brief Enum to identify specific UART instances for the Singleton.
     */
    enum which_uart
    {
        uart1,
        uart5,
        uart7,
        uart10,
    };

    /* Public Methods - Initialization and De-initialization
     * -------------------*/
    /**
     * @brief Private constructor. Called by `get_instance()`.
     */
    explicit uart_drv_t(UART_HandleTypeDef *huart, uint16_t buf_length);
    /**
     * @brief Destructor.
     */
    ~uart_drv_t();
    /**
     * @brief Resets and re-initializes the UART peripheral.
     */
    status_t reset(uint32_t BaudRate, uint32_t WordLength, uint32_t StopBits,
                   uint32_t Parity);
    /**
     * @brief Accessor for the UART driver Singleton instance.
     */
    static uart_drv_t *get_instance(which_uart uart);

    /* Public Methods - Transmission
     * -------------------------------------------*/
    /**
     * @brief Blocking (polling) write.
     */
    status_t write(const uint8_t *p, uint16_t size,
                   uint32_t waittime); // Polling
    /**
     * @brief Non-blocking (DMA) write.
     */
    status_t write(const uint8_t *p, uint16_t size); // DMA

    /* Public Methods - Reception Control
     * --------------------------------------*/
    /**
     * @brief Starts DMA reception (ReceiveToIdle).
     */
    status_t enable_rx_dma();
    /**
     * @brief Aborts DMA reception.
     */
    status_t disable_rx_dma() const;

    /* Public Methods - Custom Callback Management
     * -----------------------------*/
    /**
     * @brief Adds a C++ RX event callback.
     */
    void add_rx_event_callback(const rx_event_func &func, uint32_t owner);
    /**
     * @brief Removes a C++ RX event callback by its owner ID.
     */
    status_t remove_rx_event_callback(uint32_t);

    /* Public Methods - HAL Callback Registration
     * ------------------------------*/
    /**
     * @brief Registers the HAL RxEventCallback.
     */
    status_t register_event_callback(pUART_RxEventCallbackTypeDef pCallback) const;
    /**
     * @brief Unregisters the HAL RxEventCallback.
     */
    status_t unregister_event_callback() const;
    /**
     * @brief Registers other HAL callbacks (e.g., TxCplt).
     */
    status_t register_callback(HAL_UART_CallbackIDTypeDef CB_ID,
                               pUART_CallbackTypeDef pCallback) const;
    /**
     * @brief Unregisters other HAL callbacks.
     */
    status_t unregister_callback(HAL_UART_CallbackIDTypeDef CB_ID) const;

    /* Public Methods - Static Access
     * ----------------------------------------*/
    /**
     * @brief Provides access to the static map linking HAL handles to
     * instances.
     */
    static std::map<UART_HandleTypeDef *, uart_drv_t *> &uart_map();

    /* Public Members - State/Data
     * ------------------------------------*/
    std::vector<rx_event_callback_t> rx_event_callbacks;
    uint8_t *rx_buf[2];      // Double buffers for DMA reception
    uint8_t rx_buf_switch{}; // Index of the currently active buffer
    state_t state{};


  private:
    /* Private Members
     * ---------------------------------------------------------*/
    UART_HandleTypeDef *_huart; // HAL handle for the peripheral
    uint16_t _rx_buf_size{};    // Size of each RX buffer
};

} // namespace pyro


#endif // __PYRO_UART_DRV_H__