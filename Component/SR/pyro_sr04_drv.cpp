#include "pyro_sr04_drv.h"
#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"

namespace pyro
{
#ifdef SR04_UART
sr04_drv& sr04_drv::get_instance()
{
    static sr04_drv instance(&SR04_UART);
    return instance;
}
#endif

sr04_drv::sr04_drv(uart_drv_t *uart) : sonic_base_drv(uart, "sr04_task")
{
    _tx_buf = static_cast<uint8_t *>(pvPortDmaMalloc(1));
    if (_tx_buf) *_tx_buf = 0xA0; // HC-SR04 单芯片版串口触发命令
}

sr04_drv::~sr04_drv()
{
    if (_tx_buf) vPortFree(_tx_buf);
}

void sr04_drv::init()
{
    if (_uart_drv)
    {
        _uart_drv->add_rx_event_callback(
            [this](const uint8_t *p, uint16_t sz, BaseType_t &woken) -> bool
            { return this->rx_callback_impl(p, sz); },
            reinterpret_cast<uint32_t>(this));
    }
}

status_t sr04_drv::trigger_impl()
{
    if (!_uart_drv || !_tx_buf) return PYRO_ERROR;
    return _uart_drv->write(_tx_buf, 1);
}

bool sr04_drv::rx_callback_impl(const uint8_t *p_data, uint16_t size)
{
    if (size == 3) // HC-SR04 单芯片版返回 3 字节数据
    {
        // 距离=((BYTE_H<<16) + (BYTE_M<<8) + BYTE_L)/1000
        uint32_t raw = (static_cast<uint32_t>(p_data[0]) << 16) | 
                       (static_cast<uint32_t>(p_data[1]) << 8) | 
                       p_data[2];

        update_distance(raw / 1000); // 转换为 mm
        return true;
    }
    return false;
}

} // namespace pyro