#include "pyro_stp23l_drv.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"

namespace pyro
{

#ifdef STP23L_DOWN_UART
stp23l_drv& stp23l_drv::down_get_instance()
{
    static stp23l_drv instance(&STP23L_DOWN_UART);
    return instance;
}
#endif

#ifdef STP23L_FRONT_UART
stp23l_drv& stp23l_drv::front_get_instance()
{
    static stp23l_drv instance(&STP23L_FRONT_UART);
    return instance;
}
#endif

stp23l_drv::stp23l_drv(uart_drv_t *uart)
    : sonic_base_drv(uart, "stp23l_task")
{
}

stp23l_drv::~stp23l_drv() = default;

void stp23l_drv::init()
{
    if (_uart_drv)
    {
        _uart_drv->add_rx_event_callback(
            [this](uint8_t *p, uint16_t size, BaseType_t &woken) -> bool
            { return this->rx_callback_impl(p, size); },
            reinterpret_cast<uint32_t>(this));
    }
}

status_t stp23l_drv::trigger_impl()
{
    // STP-23L starts streaming automatically after power-up.  No trigger
    // command is needed for continuous measurement.
    return PYRO_OK;
}

bool stp23l_drv::rx_callback_impl(const uint8_t *p_data, uint16_t size)
{
    if (!p_data)
    {
        return false;
    }

    // ReceiveToIdle DMA may return a partial frame or several frames at once.
    // Keep a small protocol parser so neither case loses synchronization.
    for (uint16_t i = 0; i < size; ++i)
    {
        consume_byte(p_data[i]);
    }

    // The callback has consumed the DMA buffer; let the UART driver switch to
    // the other buffer before restarting reception.
    return true;
}

void stp23l_drv::reset_parser()
{
    _frame_size = 0;
    _expected_frame_size = 0;
}

void stp23l_drv::consume_byte(const uint8_t byte)
{
    if (_frame_size == 0)
    {
        if (byte == 0xAAU)
        {
            _frame[_frame_size++] = byte;
        }
        return;
    }

    // The four-byte preamble is the only valid packet start.  If a malformed
    // stream contains another 0xAA, use it as the beginning of a new frame.
    if (_frame_size < 4U)
    {
        if (byte == 0xAAU)
        {
            _frame[_frame_size++] = byte;
        }
        else
        {
            reset_parser();
        }
        return;
    }

    if (_frame_size >= MAX_FRAME_LENGTH)
    {
        reset_parser();
        if (byte == 0xAAU)
        {
            _frame[_frame_size++] = byte;
        }
        return;
    }

    _frame[_frame_size++] = byte;

    if (_frame_size == FRAME_HEADER_LENGTH)
    {
        const uint16_t data_length =
            static_cast<uint16_t>(_frame[8]) |
            (static_cast<uint16_t>(_frame[9]) << 8U);

        if (data_length > MAX_DATA_LENGTH)
        {
            reset_parser();
            return;
        }

        _expected_frame_size = FRAME_HEADER_LENGTH + data_length + 1U;
    }

    if (_expected_frame_size != 0U && _frame_size == _expected_frame_size)
    {
        process_frame();
        reset_parser();
    }
}

void stp23l_drv::process_frame()
{
    if (_frame[5] != PACK_GET_DISTANCE)
    {
        return;
    }

    const uint16_t data_length =
        static_cast<uint16_t>(_frame[8]) |
        (static_cast<uint16_t>(_frame[9]) << 8U);
    if (data_length < POINT_SIZE)
    {
        return;
    }

    uint8_t checksum = 0;
    for (uint16_t i = 4; i < FRAME_HEADER_LENGTH + data_length; ++i)
    {
        checksum = static_cast<uint8_t>(checksum + _frame[i]);
    }
    if (checksum != _frame[FRAME_HEADER_LENGTH + data_length])
    {
        return;
    }

    // A point is 15 bytes: distance (int16), noise (uint16), peak (uint32),
    // confidence (uint8), integration count (uint32), and temperature marker
    // (int16).  Use the nearest positive distance in the frame as the scalar
    // distance exposed by sonic_base_drv.
    const uint16_t point_data_length = data_length - 4U; // omit timestamp
    const uint16_t point_count = point_data_length / POINT_SIZE;
    uint16_t nearest = 0xFFFFU;
    for (uint16_t point = 0; point < point_count; ++point)
    {
        const uint16_t offset = FRAME_HEADER_LENGTH + point * POINT_SIZE;
        const int16_t distance = static_cast<int16_t>(
            static_cast<uint16_t>(_frame[offset]) |
            (static_cast<uint16_t>(_frame[offset + 1U]) << 8U));
        if (distance > 0 && static_cast<uint16_t>(distance) < nearest)
        {
            nearest = static_cast<uint16_t>(distance);
        }
    }

    if (nearest != 0xFFFFU)
    {
        update_distance(nearest);
    }
}

} // namespace pyro
