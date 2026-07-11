#include "pyro_us100_drv.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"

namespace pyro
{
#ifdef US100_UART
us100_drv& us100_drv::get_instance()
{

    static us100_drv instance(&US100_UART);
    return instance;
}
#endif

us100_drv::us100_drv(uart_drv_t *uart) : sonic_base_drv(uart, "us100_task")
{
    _tx_buf = static_cast<uint8_t *>(pvPortDmaMalloc(1));
    if (_tx_buf) *_tx_buf = 0x55; // US-100 测距启动电平
}

us100_drv::~us100_drv()
{
    if (_tx_buf) vPortFree(_tx_buf);
}

void us100_drv::init()
{
    if (_uart_drv)
    {
        _uart_drv->add_rx_event_callback(
            [this](const uint8_t *p, uint16_t sz, BaseType_t &woken) -> bool
            { return this->rx_callback_impl(p, sz); },
            reinterpret_cast<uint32_t>(this));
    }
}

status_t us100_drv::trigger_impl()
{
    if (!_uart_drv || !_tx_buf) return PYRO_ERROR;
    return _uart_drv->write(_tx_buf, 1);
}

bool us100_drv::rx_callback_impl(const uint8_t *p_data, uint16_t size)
{
    if (size == 2) // US-100 返回 2 字节距离数据
    {
        uint32_t dist = (p_data[0] << 8) | p_data[1]; // 单位 mm
        update_distance(dist);
        return true;
    }
    return false;
}

} // namespace pyro