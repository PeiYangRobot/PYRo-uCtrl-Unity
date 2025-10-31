/**
* @file pyro_rc_base_drv.cpp
 * @brief Implementation file for the PYRO Remote Control (RC) Driver base class.
 *
 * This file implements the constructor and destructor for the abstract base
 * class `pyro::rc_drv_t`, handling the initialization of dependencies and the
 * safe cleanup of FreeRTOS resources (task and message buffer).
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_rc_base_drv.h"
#include <cstring>
#include "message_buffer.h"
#include "task.h"


// Sequence variable defined globally in the original file, kept for structure

namespace pyro
{

/* Constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for the RC driver base class.
 *
 * Initializes the pointer to the required UART driver instance.
 *
 * @param uart Pointer to the initialized UART driver.
 */
rc_drv_t::rc_drv_t(uart_drv_t *uart)
{
    _rc_uart = uart;
    sequence = 0x80;
}

rw_lock& rc_drv_t::get_lock() const
{
    return *_lock;
}


/* Destructor ----------------------------------------------------------------*/
/**
 * @brief Virtual destructor for the RC driver base class.
 *
 * Safely deletes the FreeRTOS message buffer and stops/deletes the
 * associated FreeRTOS task if their handles are valid.
 */
rc_drv_t::~rc_drv_t()
{
    // Clean up the message buffer resource
    if (_rc_msg_buffer)
    {
        vMessageBufferDelete(_rc_msg_buffer);
        _rc_msg_buffer = nullptr;
    }
    // Clean up the FreeRTOS task resource
    if (_rc_task_handle)
    {
        vTaskDelete(_rc_task_handle);
        _rc_task_handle = nullptr;
    }
}
} // namespace pyro
