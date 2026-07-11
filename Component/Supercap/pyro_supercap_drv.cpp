/**
 * @file pyro_supercap_drv.cpp
 * @brief Implementation of Supercapacitor Driver using Composition.
 *
 * @author Lucky (AI Assistant)
 * @date 2026-02-03
 */

#include "pyro_supercap_drv.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"
#include "pyro_crc.h"
#include <cstring>

namespace pyro
{

/* ========================================================================== */
/* Inner Task Implementation                                                  */
/* ========================================================================== */

status_t supercap_drv_t::supercap_task_t::init()
{
    if (_owner)
    {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void supercap_drv_t::supercap_task_t::run_loop()
{
    if (_owner)
    {
        _owner->run_loop_impl();
    }
}

/* ========================================================================== */
/* Driver Implementation                                                      */
/* ========================================================================== */
#ifdef SUPERCAP_UART
/* instance ------------------------------------------------------------------*/
supercap_drv_t *supercap_drv_t::get_instance()
{
    static supercap_drv_t instance(&PYRO_UART7);
    return &instance;
}
#endif
/* Constructor & Destructor --------------------------------------------------*/
supercap_drv_t::supercap_drv_t(uart_drv_t *uart_handle)
    : _uart_drv(uart_handle), _task(nullptr), _tx_buffer(nullptr),
      _rx_msg_buf(nullptr), _is_online(false)
{
    memset(&_latest_feedback, 0, sizeof(_latest_feedback));

    // 1. Allocate DMA-capable TX buffer
    constexpr size_t buf_size = sizeof(tx_packet_t);
    _tx_buffer = static_cast<tx_packet_t *>(pvPortDmaMalloc(buf_size));

    if (_tx_buffer)
    {
        memset(_tx_buffer, 0, buf_size);
    }

    // 2. Instantiate the internal task (it is NOT started yet)
    _task = new supercap_task_t(this);
}

supercap_drv_t::~supercap_drv_t()
{
    // 1. Stop and delete task
    if (_task)
    {
        _task->stop(); // Ensure FreeRTOS task is deleted
        delete _task;
        _task = nullptr;
    }

    // 2. Unregister UART callback
    if (_uart_drv)
    {
        _uart_drv->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
    }

    // 3. Free resources
    if (_tx_buffer)
    {
        vPortFree(_tx_buffer);
        _tx_buffer = nullptr;
    }

    if (_rx_msg_buf)
    {
        vMessageBufferDelete(_rx_msg_buf);
        _rx_msg_buf = nullptr;
    }
}

/* Public Control Methods ----------------------------------------------------*/
void supercap_drv_t::start_rx() const
{
    // Only start if resources are valid
    if (_task && _uart_drv && _tx_buffer)
    {
        _task->start();
    }
}

/* Logic Implementation (Private) --------------------------------------------*/

// Called by supercap_task_t::init() in the new task context
void supercap_drv_t::init_impl()
{
    // 1. Create Message Buffer (Capacity for ~4 frames + overhead)
    if (_rx_msg_buf == nullptr)
    {
        _rx_msg_buf = xMessageBufferCreate(80);
    }

    if (_rx_msg_buf == nullptr)
        return;

    // 2. Register RX ISR Callback
    // We use a lambda to bridge to the member function
    _uart_drv->add_rx_event_callback(
        [this](const uint8_t *p, const uint16_t size,
               BaseType_t &task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
}

// Called by supercap_task_t::run_loop()
void supercap_drv_t::run_loop_impl()
{
    while (true)
    {
        static rx_packet_t pkt;
        static size_t xReceivedBytes;
        if (xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt),
                                  portMAX_DELAY) == sizeof(rx_packet_t))
        {
            // Signal that this high-priority protocol is now online
            _is_online = true;
        }

        while (_is_online)
        {
            // Block until data arrives (100ms Timeout)
            xReceivedBytes =
                xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt), 120);

            if (xReceivedBytes == sizeof(rx_packet_t))
            {
                if (error_check(&pkt) == PYRO_OK)
                {
                    unpack(&pkt);
                }
            }
            else if (xReceivedBytes == 0)
            {
                // Timeout -> Offline
                _is_online = false;
            }
        }
    }
}

/* ISR Callback --------------------------------------------------------------*/
__attribute__((section(".itcm_text")))
bool supercap_drv_t::rx_callback(const uint8_t *p_data, const uint16_t size,
                                 BaseType_t &xHigherPriorityTaskWoken) const
{
    // Minimal check in ISR: Frame Start and Minimum Length
    if (size == sizeof(rx_packet_t) + 1 && p_data[0] == FRAME_SOF &&
        p_data[sizeof(rx_packet_t)] == '\n')
    {
        // Send raw data to the task via Message Buffer
        xMessageBufferSendFromISR(_rx_msg_buf, p_data, sizeof(rx_packet_t),
                                  &xHigherPriorityTaskWoken);
        return true; // Data consumed, driver should swap buffer
    }
    return false;
}

/* Protocol Helpers ----------------------------------------------------------*/
status_t supercap_drv_t::error_check(const rx_packet_t *buf)
{
    // // 1. CRC8 Check (Header)
    // if (!verify_crc8_check_sum(reinterpret_cast<uint8_t const *>(buf),
    //                            sizeof(frame_header_t)))
    // {
    //     return PYRO_ERROR;
    // }

    // 2. CRC16 Check (Whole Packet)
    // Verify standard frame size (13 bytes). This ignores the extra '\n' if
    // present.
    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const *>(buf),
                                sizeof(rx_packet_t)))
    {
        return PYRO_ERROR;
    }

    return PYRO_OK;
}

void supercap_drv_t::unpack(const rx_packet_t *buf)
{
    memcpy(&_latest_feedback, &buf->data, sizeof(cap_feedback_t));
}

/* Transmission --------------------------------------------------------------*/
status_t supercap_drv_t::send_cmd(const chassis_cmd_t &cmd) const
{
    if (!_tx_buffer || !_uart_drv)
        return PYRO_ERROR;

    // 1. Fill Header
    _tx_buffer->header.sof = FRAME_SOF;
    append_crc8_check_sum(reinterpret_cast<uint8_t *>(&_tx_buffer->header),
                          sizeof(frame_header_t));

    // 2. Copy Payload
    memcpy(&_tx_buffer->data, &cmd, sizeof(chassis_cmd_t));

    // 3. Fill Tailer (CRC16)
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_buffer),
                           sizeof(tx_packet_t) - 1);

    _tx_buffer->end_byte = '\n';

    // 4. DMA Write
    return _uart_drv->write(reinterpret_cast<uint8_t *>(_tx_buffer),
                            sizeof(tx_packet_t));
}

/* Getters -------------------------------------------------------------------*/
const supercap_drv_t::cap_feedback_t &supercap_drv_t::get_feedback() const
{
    return _latest_feedback;
}

bool supercap_drv_t::check_online() const
{
    return _is_online;
}

} // namespace pyro