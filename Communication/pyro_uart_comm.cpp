/**
 * @file pyro_uart_comm.cpp
 * @brief UART 通信应用层二次封装实现
 */

#include "pyro_uart_comm.h"

namespace pyro
{

uart_comm_t::uart_comm_t(const uart_drv_t &uart_drv, uint32_t owner_id,
                         size_t msg_buf_size)
    : _drv(uart_drv), _owner_id(owner_id)
{
    // 1. 创建 FreeRTOS 消息缓冲区
    _msg_buffer = xMessageBufferCreate(msg_buf_size);
    configASSERT(_msg_buffer != nullptr);


    // 3. 将本类的 internal_rx_callback 通过 Lambda 表达式注册到底层 ISR
    _drv.add_rx_event_callback(
        [this](uint8_t *p, uint16_t size, BaseType_t &xHigherTaskWoken) -> bool
        { return this->internal_rx_callback(p, size, xHigherTaskWoken); },
        _owner_id);

    // 4. 开启底层 DMA 双缓冲接收
    _drv.enable_rx_dma();
}

uart_comm_t::~uart_comm_t()
{
    _drv.remove_rx_event_callback(_owner_id);

    // 销毁消息缓冲区
    if (_msg_buffer != nullptr)
    {
        vMessageBufferDelete(_msg_buffer);
        _msg_buffer = nullptr;
    }
}

// 注册重载 1：无帧头校验，仅校验长度
void uart_comm_t::register_msg_type(uint16_t length)
{
    msg_config_t cfg;
    cfg.length     = length;
    cfg.header_len = 0; // 显式置 0，代表不校验帧头

    _msg_configs.push_back(cfg);
}

// 注册重载 2：有帧头校验
void uart_comm_t::register_msg_type(uint16_t length, const uint8_t *header,
                                    uint8_t header_len)
{
    msg_config_t cfg;
    cfg.length = length;
    cfg.header_len =
        (header_len > 8) ? 8 : header_len; // 限制最大帧头长度防溢出

    if (header != nullptr && cfg.header_len > 0)
    {
        std::memcpy(cfg.header, header, cfg.header_len);
    }
    else
    {
        cfg.header_len = 0; // 安全降级为无帧头
    }

    _msg_configs.push_back(cfg);
}

// 【核心逻辑】ISR 内部滑动窗口防粘包/错位解析算法
bool uart_comm_t::internal_rx_callback(uint8_t *p, uint16_t size,
                                       BaseType_t &xHigherPriorityTaskWoken)
{
    // 若未注册任何消息白名单，或传入数据为空，直接返回 false
    // (丢弃数据并要求底层勿翻转)
    if (_msg_configs.empty() || size == 0)
    {
        return false;
    }

    uint16_t idx         = 0;
    bool found_valid_msg = false;

    // 滑动窗口机制：在接收到的字节流中搜索所有合法的数据包
    while (idx < size)
    {
        bool packet_matched = false;

        // 遍历所有已注册的消息规则
        for (const auto &cfg : _msg_configs)
        {
            // 1. 长度校验：窗口剩余数据量必须大于等于该类型包长
            if ((idx + cfg.length) <= size)
            {
                bool header_ok = true;

                // 2. 帧头校验 (仅当配置了帧头时进行)
                if (cfg.header_len > 0)
                {
                    if (std::memcmp(&p[idx], cfg.header, cfg.header_len) != 0)
                    {
                        header_ok = false;
                    }
                }

                // 3. 校验完全通过，提取此包并送入 FreeRTOS 消息队列
                if (header_ok)
                {
                    xMessageBufferSendFromISR(_msg_buffer, &p[idx], cfg.length,
                                              &xHigherPriorityTaskWoken);

                    idx += cfg.length; // 窗口指针直接跳过这一整个完美包
                    packet_matched  = true;
                    found_valid_msg = true;
                    break; // 跳出规则遍历循环，继续向后解析下一个可能粘连的包
                }
            }
        }

        // 如果在当前 idx 起始位置没有任何匹配的规则，说明可能是错位数据或干扰
        // 窗口向后滑动 1 个字节，继续寻找下一个有效的特征帧头
        if (!packet_matched)
        {
            idx++;
        }
    }

    // 告知底层驱动：true
    // 代表从这段乱码/粘包中成功抠出了至少一帧有用数据（底层将执行 DMA 翻转）
    // false 代表全是垃圾数据，直接抛弃
    return found_valid_msg;
}

} // namespace pyro