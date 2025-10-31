/**
 * @file pyro_dr16_rc_drv.cpp
 * @brief Implementation file for the PYRO DR16 Remote Control Driver.
 *
 * This file contains the protocol-specific implementation of the DR16 driver,
 * including FreeRTOS task creation, data unpacking, error checking, and
 * managing data transfer between the ISR and the processing thread.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_dr16_rc_drv.h"

#include "message_buffer.h" // Needed for xMessageBuffer* calls
#include "pyro_rw_lock.h"
#include "task.h"           // Needed for xTaskCreate calls
#include <cstring>

// External FreeRTOS task entry point
extern "C" void dr16_task(void *argument);

namespace pyro
{

/* Constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for the DR16 driver.
 *
 * Calls the base class constructor and sets the internal task priority.
 */
dr16_drv_t::dr16_drv_t(uart_drv_t *dr16_uart) : rc_drv_t(dr16_uart)
{
    _priority = 1;
}

/* Initialization ------------------------------------------------------------*/
/**
 * @brief Initializes FreeRTOS resources (message buffer and processing task).
 * @return PYRO_OK on success, PYRO_ERROR otherwise.
 */
status_t dr16_drv_t::init()
{
    // Create the message buffer (108 bytes capacity)
    _rc_msg_buffer   = xMessageBufferCreate(108);

    // Create the processing task
    BaseType_t x_ret = xTaskCreate(dr16_task, "dr16_task", 1024, this,
                                   configMAX_PRIORITIES - 1, &_rc_task_handle);

    if (x_ret != pdPASS)
    {
        return PYRO_ERROR;
    }
    if (_rc_msg_buffer == nullptr)
    {
        return PYRO_ERROR;
    }
    _lock = new rw_lock;
    return PYRO_OK;
}

/* Enable/Disable ------------------------------------------------------------*/
/**
 * @brief Enables the DR16 receiver.
 *
 * Adds the ISR callback to the UART driver and sets the protocol's sequence
 * bit.
 */
void dr16_drv_t::enable()
{
    // Set the priority bit in the base class static sequence variable
    sequence |= (1 << _priority);
    // Register the local rc_callback method as the UART RX event handler
    _rc_uart->add_rx_event_callback(
        [this](uint8_t *buf, uint16_t len,
               BaseType_t xHigherPriorityTaskWoken) -> bool
        { return rc_callback(buf, len, xHigherPriorityTaskWoken); },
        reinterpret_cast<uint32_t>(this));
}

/**
 * @brief Disables the DR16 receiver.
 *
 * Removes the ISR callback from the UART driver and clears the sequence bit.
 */
void dr16_drv_t::disable()
{
    // Clear the priority bit in the base class static sequence variable
    sequence &= ~(1 << _priority);
    // Remove the registered callback using the instance address as the owner ID
    _rc_uart->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
}

/* Data Processing - Error Check ---------------------------------------------*/
/**
 * @brief Performs basic range checking on received channel data.
 * @return PYRO_OK if all main channels are within min/max bounds.
 */
status_t dr16_drv_t::error_check(const dr16_buf_t *dr16_buf)
{
    if (dr16_buf->ch0 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch0 > DR16_CH_VALUE_MAX ||
        dr16_buf->ch1 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch1 > DR16_CH_VALUE_MAX ||
        dr16_buf->ch2 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch2 > DR16_CH_VALUE_MAX ||
        dr16_buf->ch3 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch3 > DR16_CH_VALUE_MAX ||
        dr16_buf->wheel < DR16_CH_VALUE_MIN ||
        dr16_buf->wheel > DR16_CH_VALUE_MAX)
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/* Data Processing - Unpack --------------------------------------------------*/
/**
 * @brief Unpacks the raw DR16 buffer into the consumer-friendly control
 * structure.
 *
 * If the error check passes, it scales RC channels (center-aligned), copies
 * mouse data, and maps the key_code to the keyboard bitfield structure.
 */
void dr16_drv_t::unpack(const dr16_buf_t *dr16_buf)
{
    if (PYRO_OK == error_check(dr16_buf))
    {
        _dr16_last_ctrl = _dr16_ctrl; // Save last state
        // Scale and center RC channels
        _dr16_ctrl.rc.ch[0] =
            static_cast<int16_t>(dr16_buf->ch0 - DR16_CH_VALUE_OFFSET);
        _dr16_ctrl.rc.ch[1] =
            static_cast<int16_t>(dr16_buf->ch1 - DR16_CH_VALUE_OFFSET);
        _dr16_ctrl.rc.ch[2] =
            static_cast<int16_t>(dr16_buf->ch2 - DR16_CH_VALUE_OFFSET);
        _dr16_ctrl.rc.ch[3] =
            static_cast<int16_t>(dr16_buf->ch3 - DR16_CH_VALUE_OFFSET);
        _dr16_ctrl.rc.wheel =
            static_cast<int16_t>(dr16_buf->wheel - DR16_CH_VALUE_OFFSET);

        // Copy switch and mouse data
        _dr16_ctrl.rc.s[DR16_SW_RIGHT]      = dr16_buf->s1;
        _dr16_ctrl.rc.s[DR16_SW_LEFT]       = dr16_buf->s2;
        _dr16_ctrl.mouse.x                 = dr16_buf->mouse_x;
        _dr16_ctrl.mouse.y                 = dr16_buf->mouse_y;
        _dr16_ctrl.mouse.z                 = dr16_buf->mouse_z;
        _dr16_ctrl.mouse.press_l           = dr16_buf->press_l & 0x01;
        _dr16_ctrl.mouse.press_r           = dr16_buf->press_r & 0x01;

        // Copy key code into the key bitfield structure
        memcpy(&_dr16_ctrl.key, &dr16_buf->key_code,
               sizeof(dr16_buf->key_code));

        // Execute the registered consumer callback with the decoded data
        write_scope_lock rc_write_lock(get_lock());
        // Critical section - safely update shared control data
        // (No other thread can access vt03_ctrl during this time)
        for (auto &rc_to_cmd : _cmd_funcs)
        {
            if (rc_to_cmd)
            {
                rc_to_cmd(this);
            }
        }
    }
}

/* Interrupt Service Routine (ISR) Callback ----------------------------------*/
/**
 * @brief Called by the UART driver upon an RX event (ISR context).
 *
 * If the length is correct (18 bytes for DR16), it checks the priority flag
 * and sends the raw buffer to the message buffer for deferred processing.
 * @return true if data was buffered and the UART buffer should switch.
 */
bool dr16_drv_t::rc_callback(uint8_t *buf, uint16_t len,
                             BaseType_t xHigherPriorityTaskWoken)
{
    if (len == 18)
    {
        // Check if the protocol is higher priority than any other active RC
        // protocol
        if (__builtin_ctz(sequence) >= _priority)
        {
            xMessageBufferSendFromISR(_rc_msg_buffer, buf, len,
                                      &xHigherPriorityTaskWoken);
            return true;
        }
    }
    return false;
}

/* FreeRTOS Task Thread ------------------------------------------------------*/
/**
 * @brief The main processing thread (FreeRTOS task).
 *
 * Waits for a message buffer signal, then continuously processes all available
 * messages until the buffer is empty or a timeout occurs, resetting the
 * sequence bit on timeout (loss of signal).
 */
void dr16_drv_t::thread()
{
    static dr16_buf_t dr16_buf;
    static size_t xReceivedBytes;

    // Wait indefinitely for the first packet after a potential loss
    if (xMessageBufferReceive(_rc_msg_buffer, &dr16_buf, sizeof(dr16_buf_t),
                              portMAX_DELAY) == sizeof(dr16_buf_t))
    {
        // Signal that a packet was received (used for priority management)
        sequence |= (1 << _priority);
    }

    // Process packets as long as the sequence bit is set
    while (sequence >> _priority & 0x01)
    {
        // Receive subsequent packets with a timeout (100 ticks)
        xReceivedBytes = xMessageBufferReceive(_rc_msg_buffer, &dr16_buf,
                                               sizeof(dr16_buf_t), 100);
        if (xReceivedBytes == sizeof(dr16_buf_t))
        {
            unpack(&dr16_buf); // Process the packet
        }
        else if (xReceivedBytes == 0)
        {
            // If timeout occurs (0 bytes received), assume link loss/stale data
            sequence &= ~(1 << _priority);
        }
    }
}
void *dr16_drv_t::get_p_ctrl()
{
    return &_dr16_ctrl;
}
void *dr16_drv_t::get_p_last_ctrl()
{
    return &_dr16_last_ctrl;
}


/* Configuration -------------------------------------------------------------*/
/**
 * @brief Sets the callback function that receives the decoded control data.
 */
void dr16_drv_t::config_rc_cmd(const cmd_func &func)
{
    _cmd_funcs.push_back(func);
}

} // namespace pyro

/* External FreeRTOS Task Entry ----------------------------------------------*/
/**
 * @brief C-linkage entry point for the FreeRTOS task.
 *
 * Casts the argument pointer to the `dr16_drv_t` instance and runs the main
 * processing loop (`thread()`). Deletes the task upon exit.
 */
extern "C" void dr16_task(void *argument)
{
    auto *drv = static_cast<pyro::dr16_drv_t *>(argument);
    if (drv)
    {
        while (true)
        {
            drv->thread();
        }
    }
    vTaskDelete(nullptr);
}