#include "pyro_sr05_drv.h"
#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"

namespace pyro
{
#ifdef SR05_UART
sr05_drv& sr05_drv::get_instance()
{
    static sr05_drv instance(&SR05_UART);
    return instance;
}
#endif

sr05_drv::sr05_drv(uart_drv_t *uart) : sonic_base_drv(uart, "sr05_task")
{
    // 该模块只要接入电源便会以 18ms 为周期自动发送数据 [cite: 2]。
    // 串口模式下模块不接收数据，只发送 [cite: 65]。
    // 因此无需像 SR04/US100 那样分配 TX DMA 缓冲用于触发。
}

sr05_drv::~sr05_drv()
{
    // 无 _tx_buf，无需手动释放内存
}

void sr05_drv::init()
{
    if (_uart_drv)
    {
        _uart_drv->add_rx_event_callback(
            [this](const uint8_t *p, uint16_t sz, BaseType_t &woken) -> bool
            { return this->rx_callback_impl(p, sz); },
            reinterpret_cast<uint32_t>(this));
    }
}

status_t sr05_drv::trigger_impl()
{
    // 模块为自动周期测量，且串口不接收外部指令数据 [cite: 65, 67]。
    // 基础类中的 sensor_task 会定时调用此函数，此处直接返回成功即可。
    return PYRO_OK;
}

bool sr05_drv::rx_callback_impl(const uint8_t *p_data, uint16_t size)
{
    // 模块每次输出 4 个字节，格式为：0XFF + H_DATA + L_DATA + SUM [cite: 4]
    if (size == 4) 
    {
        // 1. 判断包头：0XFF 为一组开始数据 [cite: 5]
        if (p_data[0] != 0xFF)
        {
            return false;
        }

        // 2. 和校验：其 0XFF+H_DATA+L_DATA=SUM（仅低 8 位） [cite: 8]
        uint8_t sum = p_data[0] + p_data[1] + p_data[2];
        if (sum != p_data[3])
        {
            return false; // 校验错，丢弃该帧数据
        }

        // 3. 数据合成：H_DATA * 256 + L_DATA，即以毫米为单位的距离值 [cite: 9, 10]
        uint32_t dist = (p_data[1] << 8) | p_data[2];
        update_distance(dist);
        
        return true;
    }
    return false;
}

} // namespace pyro