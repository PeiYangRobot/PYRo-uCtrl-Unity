#include "pyro_ms53l0m_drv.h"
#include "task.h"
#include <cstdlib>
#include <cstring>

namespace pyro
{

ms53l0m_drv_t *ms53l0m_drv_t::get_instance()
{
    static ms53l0m_drv_t instance(uart_drv_t::which_uart::uart5);
    return &instance;
}

ms53l0m_drv_t::ms53l0m_drv_t(uart_drv_t::which_uart uart_instance)
    : _distance_mm(0), _state(range_state_t::UNKNOWN)
{
    _uart = uart_drv_t::get_instance(uart_instance);
}

ms53l0m_drv_t::~ms53l0m_drv_t()
{
    if (_uart)
    {
        _uart->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
    }
}

status_t ms53l0m_drv_t::init()
{
    if (!_uart)
    {
        return PYRO_ERROR;
    }

    // 重新定义轻量级的路由 Lambda，将底层传入的非 const 指针转发给类的成员函数
    auto rx_cb = [this](const uint8_t *p, const uint16_t size,
                        BaseType_t &woken) -> bool
    { return this->rx_callback(p, size, woken); };

    // 注册回调，开启 DMA 接收
    _uart->add_rx_event_callback(rx_cb, reinterpret_cast<uint32_t>(this));
    return PYRO_OK;
}

bool ms53l0m_drv_t::rx_callback(const uint8_t *p, const uint16_t size,
                                BaseType_t &xHigherPriorityTaskWoken)
{
    // // 1. 包头校验：快速过滤长度不足或非 "State" 开头的脏数据
    if (size < 5)
    {
        return false;
    }

    // 2. 数据安全拷贝，最大截取 63 字节并保证末尾有 '\0'
    char buf[64]      = {0};
    uint16_t copy_len = (size < (sizeof(buf) - 1)) ? size : (sizeof(buf) - 1);
    std::memcpy(buf, p, copy_len);

    long parsed_state     = -1;
    long parsed_dist      = -1;

    // 3. 状态解析 (使用 strtol 替代 atoi)
    const char *state_str = std::strstr(buf, "State:");
    if (state_str)
    {
        const char *num_start = state_str + 6; // 跳过 "State:"
        char *end_ptr         = nullptr;

        // base 10 表示十进制，end_ptr 会指向解析完成后的下一个字符
        long val              = std::strtol(num_start, &end_ptr, 10);

        // 如果 end_ptr 不等于 num_start，说明成功解析出了至少一个数字
        if (end_ptr != num_start)
        {
            parsed_state = val;
        }
    }

    // 4. 距离解析 (使用 strtol 替代 atoi)
    const char *dist_str = std::strstr(buf, "d:");
    if (dist_str)
    {
        const char *num_start = dist_str + 2; // 跳过 "d:"
        char *end_ptr         = nullptr;

        // strtol 会自动跳过开头的空格，并把 end_ptr 指向数字后面的字符
        // (比如空格或 'm')
        long val              = std::strtol(num_start, &end_ptr, 10);

        if (end_ptr != num_start)
        {
            parsed_dist = val;
        }
    }

    // 5. 数据有效性校验与临界区保护更新
    if (parsed_state != -1 && parsed_dist != -1)
    {
        // const uint32_t isr_status = taskENTER_CRITICAL_FROM_ISR();
        this->_state              = static_cast<range_state_t>(parsed_state);
        this->_distance_mm        = static_cast<uint32_t>(parsed_dist);
        // taskEXIT_CRITICAL_FROM_ISR(isr_status);
    }

    // 返回 true 通知底层驱动数据处理完毕，切换 DMA 缓冲接收下一帧
    return true;
}

void ms53l0m_drv_t::get_data(uint32_t &distance_mm, range_state_t &state) const
{
    taskENTER_CRITICAL();
    distance_mm = this->_distance_mm;
    state       = this->_state;
    taskEXIT_CRITICAL();
}

uint32_t ms53l0m_drv_t::get_distance() const
{
    uint32_t dist;
    taskENTER_CRITICAL();
    dist = this->_distance_mm;
    taskEXIT_CRITICAL();
    return dist;
}

} // namespace pyro