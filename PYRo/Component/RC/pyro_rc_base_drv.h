/**
 * @file pyro_rc_base_drv.h
 * @brief Header file for the PYRO Remote Control (RC) Driver base class.
 *
 * This file defines the pure virtual base class `pyro::rc_drv_t` which
 * establishes the interface for specific remote control protocols (e.g.,
 * SBUS, PPM) that communicate over UART. It manages resources like a
 * FreeRTOS task and a message buffer.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_RC_BASE_DRV_H__
#define __PYRO_RC_BASE_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_uart_drv.h"  // Dependency on the UART driver
#include "message_buffer.h" // FreeRTOS Message Buffer definitions
#include "semphr.h"         // FreeRTOS Semaphore definitions
#include "task.h"           // FreeRTOS Task definitions
#include "pyro_rw_lock.h"

namespace pyro
{

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief Abstract base class for Remote Control (RC) drivers.
 *
 * Provides a common interface for initializing, enabling, and managing the
 * communication and processing thread for various RC protocols.
 */
class rc_drv_t
{
  public:
    using cmd_func                = std::function<void(rc_drv_t *)>;
    inline static uint8_t sequence = 0x80;

    /* Public Methods - Construction and Lifecycle
     * -----------------------------*/
    explicit rc_drv_t(uart_drv_t *uart);
    virtual ~rc_drv_t();

    /* Public Methods - Pure Virtual Interface
     * ---------------------------------*/
    virtual status_t init()                                       = 0;
    virtual void enable()                                         = 0;
    virtual void disable()                                        = 0;
    virtual void thread()                                         = 0;
    virtual void config_rc_cmd(const cmd_func &func)              = 0;
    virtual void *get_p_ctrl()                                    = 0;
    virtual void *get_p_last_ctrl()                               = 0;
    rw_lock &get_lock() const;

    /**
     * @brief Callback function executed by the underlying UART driver in ISR
     * context.
     *
     * This function is protocol-specific and must be implemented by derived
     * classes.
     * @param buf Pointer to the received data buffer.
     * @param len Length of the received data.
     * @param xHigherPriorityTaskWoken Flag for FreeRTOS context switching.
     * @return true if the buffer was processed and should be switched by the
     * UART driver.
     */
    virtual bool rc_callback(uint8_t *buf, uint16_t len,
                             BaseType_t xHigherPriorityTaskWoken) = 0;


  protected:
    /* Protected Members - Resources and State
     * ---------------------------------*/
    /**
     * @brief Static sequence counter used for protocol state tracking.
     */

    std::vector<cmd_func> _cmd_funcs;
    rw_lock *_lock{};
    MessageBufferHandle_t _rc_msg_buffer{};
    ///< Handle for the FreeRTOS message buffer.
    TaskHandle_t _rc_task_handle{};
    ///< Handle for the FreeRTOS processing task.
    uart_drv_t *_rc_uart; ///< Pointer to the underlying UART driver instance.
    uint8_t _priority{};  ///< Priority of the associated FreeRTOS task.
};
} // namespace pyro

#endif
