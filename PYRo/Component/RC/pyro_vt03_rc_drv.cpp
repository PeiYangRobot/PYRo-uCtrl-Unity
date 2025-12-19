/**
 * @file pyro_vt03_rc_drv.cpp
 * @brief Implementation file for the PYRO VT03 Remote Control Driver.
 *
 * This file contains the protocol-specific implementation of the VT03 driver,
 * including FreeRTOS task creation, data unpacking, error checking, and
 * managing data transfer between the ISR and the processing thread.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_vt03_rc_drv.h"
#include "pyro_crc.h"
#include "pyro_rw_lock.h"
#include "task.h" // Needed for xTaskCreate calls
#include <cstring>

// External FreeRTOS task entry point
extern "C" void vt03_task(void *argument);

namespace pyro
{
static constexpr uint16_t VT03_CH_VALUE_MIN    = 364;
static constexpr uint16_t VT03_CH_VALUE_MAX    = 1684;
static constexpr uint16_t VT03_CH_VALUE_OFFSET = 1024;
/* Constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for the VT03 driver.
 *
 * Calls the base class constructor and sets the internal task priority.
 */
vt03_drv_t::vt03_drv_t(uart_drv_t *vt03_uart) : rc_drv_t(vt03_uart)
{
    _priority = 0;
    vt03_drv_t::init();
}

/* Initialization ------------------------------------------------------------*/
/**
 * @brief Initializes FreeRTOS resources (message buffer and processing task).
 * @return PYRO_OK on success, PYRO_ERROR otherwise.
 */
status_t vt03_drv_t::init()
{
    // Create the message buffer (108 bytes capacity)
    _rc_msg_buffer   = xMessageBufferCreate(108);

    // Create the processing task
    const BaseType_t x_ret = xTaskCreate(vt03_task, "vt03_task", 256, this,
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
    _rc_data = &_vt03_ctrl;
    return PYRO_OK;
}

/* Enable/Disable ------------------------------------------------------------*/
/**
 * @brief Enables the VT03 receiver.
 *
 * Adds the ISR callback to the UART driver and sets the protocol's sequence
 * bit.
 */
void vt03_drv_t::enable()
{
    // Register the local rc_callback method as the UART RX event handler
    _rc_uart->add_rx_event_callback(
        [this](uint8_t *buf, const uint16_t len,
               const BaseType_t xHigherPriorityTaskWoken) -> bool
        { return rc_callback(buf, len, xHigherPriorityTaskWoken); },
        reinterpret_cast<uint32_t>(this));
}

/**
 * @brief Disables the VT03 receiver.
 *
 * Removes the ISR callback from the UART driver and clears the sequence bit.
 */
void vt03_drv_t::disable()
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
status_t vt03_drv_t::error_check(const vt03_buf_t *vt03_buf)
{
    if (vt03_buf->ch0 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch0 > VT03_CH_VALUE_MAX ||
        vt03_buf->ch1 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch1 > VT03_CH_VALUE_MAX ||
        vt03_buf->ch2 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch2 > VT03_CH_VALUE_MAX ||
        vt03_buf->ch3 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch3 > VT03_CH_VALUE_MAX ||
        vt03_buf->wheel < VT03_CH_VALUE_MIN ||
        vt03_buf->wheel > VT03_CH_VALUE_MAX)
    {
        return PYRO_ERROR;
    }
    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const*>(vt03_buf), sizeof(vt03_buf_t)))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/**
 * @brief Checks for gear (switch) state changes.
 * @param vt03_gear The gear state object (to be updated).
 * @param raw_state
 */
void vt03_drv_t::check_ctrl(vt03_gear_t &vt03_gear, const uint8_t raw_state)
{
    vt03_gear_t gear = {};
    const auto state = static_cast<gear_state_t>(raw_state);
    if (vt03_gear.state == state)
    {
        gear.ctrl = gear_ctrl_t::GEAR_NO_CHANGE;
    }
    if (gear_state_t::GEAR_LEFT == vt03_gear.state && gear_state_t::GEAR_MID == state)
    {
        gear.ctrl = gear_ctrl_t::GEAR_LEFT_TO_MID;
    }
    else if (gear_state_t::GEAR_MID == vt03_gear.state && gear_state_t::GEAR_LEFT == state)
    {
        gear.ctrl = gear_ctrl_t::GEAR_MID_TO_LEFT;
    }
    else if (gear_state_t::GEAR_MID == vt03_gear.state && gear_state_t::GEAR_RIGHT == state)
    {
        gear.ctrl = gear_ctrl_t::GEAR_MID_TO_RIGHT;
    }
    else if (gear_state_t::GEAR_RIGHT == vt03_gear.state && gear_state_t::GEAR_MID == state)
    {
        gear.ctrl = gear_ctrl_t::GEAR_RIGHT_TO_MID;
    }
    gear.state = state;
    vt03_gear  = gear;
}

/**
 * @brief Checks for key state changes (PRESSED, HOLD, RELEASED).
 * @param key The key state object (to be updated).
 * @param raw_state
 */
void vt03_drv_t::check_ctrl(key_t &key, const uint8_t raw_state)
{
    key_t temp_key = {};
    const auto state     = static_cast<key_ctrl_t>(raw_state);
    if (key_ctrl_t::KEY_RELEASED == state)
    {
        temp_key.ctrl = key_ctrl_t::KEY_RELEASED;
        temp_key.time = 0;
    }
    else if (key_ctrl_t::KEY_PRESSED == state)
    {
        if (key_ctrl_t::KEY_RELEASED == key.ctrl)
        {
            temp_key.time = key.time + 14;
            if (temp_key.time > 40)
            {
                temp_key.ctrl = key_ctrl_t::KEY_PRESSED;
            }
        }
        else if (key_ctrl_t::KEY_PRESSED == key.ctrl)
        {
            temp_key.time = key.time + 14;
            if (temp_key.time > 160)
            {
                temp_key.ctrl = key_ctrl_t::KEY_HOLD;
                temp_key.time = 0;
            }
        }
    }
    key = temp_key;
}

/* Data Processing - Unpack --------------------------------------------------*/
/**
 * @brief Unpacks the raw VT03 buffer into the consumer-friendly control
 * structure.
 *
 * If the error check passes, it scales RC channels (center-aligned), copies
 * mouse data, and maps the key_code to the keyboard bitfield structure.
 */
void vt03_drv_t::unpack(const vt03_buf_t *vt03_buf)
{
    if (PYRO_OK == error_check(vt03_buf))
    {
        write_scope_lock rc_write_lock(get_lock());

        // Scale and center RC channels
        _vt03_ctrl.rc.ch_rx =
            static_cast<float>(vt03_buf->ch0 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.ch_ry =
            static_cast<float>(vt03_buf->ch1 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.ch_lx =
            static_cast<float>(vt03_buf->ch2 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.ch_ly =
            static_cast<float>(vt03_buf->ch3 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.wheel =
            static_cast<float>(vt03_buf->wheel - VT03_CH_VALUE_OFFSET) / 660.0f;

        // Copy switch and mouse data
        check_ctrl(_vt03_ctrl.rc.gear, vt03_buf->gear);
        check_ctrl(_vt03_ctrl.rc.fn_l, vt03_buf->fn_l);
        check_ctrl(_vt03_ctrl.rc.fn_r, vt03_buf->fn_r);
        check_ctrl(_vt03_ctrl.rc.pause, vt03_buf->pause);
        check_ctrl(_vt03_ctrl.rc.trigger, vt03_buf->trigger);

        _vt03_ctrl.mouse.x = static_cast<float>(vt03_buf->mouse_x) / 32768.0f;
        _vt03_ctrl.mouse.y = static_cast<float>(vt03_buf->mouse_y) / 32768.0f;
        _vt03_ctrl.mouse.z = static_cast<float>(vt03_buf->mouse_z) / 32768.0f;
        check_ctrl(_vt03_ctrl.mouse.press_l, vt03_buf->press_l);
        check_ctrl(_vt03_ctrl.mouse.press_r, vt03_buf->press_r);
        check_ctrl(_vt03_ctrl.mouse.press_m, vt03_buf->press_m);

        for (int i = 0; i < 16; ++i)
        {
            check_ctrl(*(reinterpret_cast<key_t *>(&_vt03_ctrl.key) + i),
                       (vt03_buf->key_code >> i) & 0x01);
        }
    }
}

/* Interrupt Service Routine (ISR) Callback ----------------------------------*/
/**
 * @brief Called by the UART driver upon an RX event (ISR context).
 *
 * If the length matches the expected size, it checks the priority flag
 * and sends the raw buffer to the message buffer for deferred processing.
 * @return true if data was buffered and the UART buffer should switch.
 */
bool vt03_drv_t::rc_callback(uint8_t *buf, const uint16_t len,
                             BaseType_t xHigherPriorityTaskWoken)
{
    if (len == sizeof(vt03_buf_t))
    {
        if (buf[0] == 0xA9 && buf[1] == 0x53)
        {
            if (__builtin_ctz(sequence) >= _priority)
            {
                xMessageBufferSendFromISR(_rc_msg_buffer, buf, len,
                                          &xHigherPriorityTaskWoken);
                return true;
            }
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
void vt03_drv_t::thread()
{
    static vt03_buf_t vt03_buf;
    static size_t xReceivedBytes;

    // Wait indefinitely for the first packet after a potential loss
    if (xMessageBufferReceive(_rc_msg_buffer, &vt03_buf, sizeof(vt03_buf_t),
                              portMAX_DELAY) == sizeof(vt03_buf_t))
    {
        // Signal that a packet was received (used for priority management)
        sequence |= (1 << _priority);
    }

    // Process packets as long as the sequence bit is set
    while (sequence >> _priority & 0x01)
    {
        // Receive subsequent packets with a timeout (100 ticks)
        xReceivedBytes = xMessageBufferReceive(_rc_msg_buffer, &vt03_buf,
                                               sizeof(vt03_buf_t), 120);
        if (xReceivedBytes == sizeof(vt03_buf_t))
        {
            unpack(&vt03_buf); // Process the packet
        }
        else if (xReceivedBytes == 0)
        {
            // If timeout occurs (0 bytes received), assume link loss/stale data
            sequence &= ~(1 << _priority);
        }
    }
}


/* Configuration -------------------------------------------------------------*/
/**
 * @brief Sets the callback function that receives the decoded control data.
 */

} // namespace pyro

/* External FreeRTOS Task Entry ----------------------------------------------*/
/**
 * @brief C-linkage entry point for the FreeRTOS task.
 *
 * Casts the argument pointer to the `vt03_drv_t` instance and runs the main
 * processing loop (`thread()`). Deletes the task upon exit.
 */
extern "C" void vt03_task(void *argument)
{
    auto *drv = static_cast<pyro::vt03_drv_t *>(argument);
    if (drv)
    {
        while (true)
        {
            drv->thread();
        }
    }
    vTaskDelete(nullptr);
}