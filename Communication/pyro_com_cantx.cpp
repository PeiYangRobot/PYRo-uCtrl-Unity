#include "pyro_com_cantx.h"
#include <cstring>

namespace pyro
{

can_tx_drv_t::can_tx_drv_t() = default;
can_tx_drv_t::~can_tx_drv_t() = default;

can_tx_drv_t *can_tx_drv_t::instance()
{
    // C++11 保证了静态局部变量初始化的线程安全
    static can_tx_drv_t instance;
    return &instance;
}

can_tx_drv_t::frame_node_t *can_tx_drv_t::get_frame(const uint32_t id)
{
    // 直接访问成员变量 _frames
    for (auto &frame : _frames)
    {
        if (frame.id == id) return &frame;
    }

    frame_node_t new_frame{};
    new_frame.id = id;
    new_frame.current_bit_usage = 0;
    memset(new_frame.buffer, 0, 8);
    this->_frames.push_back(new_frame);

    return &_frames.back();
}

void can_tx_drv_t::clear(const uint32_t id)
{
    frame_node_t *frame = instance()->get_frame(id);
    if (frame)
    {
        frame->current_bit_usage = 0;
        memset(frame->buffer, 0, 8);
    }
}

void can_tx_drv_t::add_data_raw(const uint32_t id, const uint8_t bit_len, const void *data)
{
    if (!data || bit_len == 0) return;

    frame_node_t *frame = instance()->get_frame(id);

    if (frame->current_bit_usage + bit_len > 64) return;

    uint64_t data_val = 0;
    uint8_t copy_size = (bit_len + 7) / 8;
    if (copy_size > 8) copy_size = 8;
    memcpy(&data_val, data, copy_size);

    if (bit_len < 64) data_val &= ((1ULL << bit_len) - 1);
    data_val <<= frame->current_bit_usage;

    uint64_t buffer_val = 0;
    memcpy(&buffer_val, frame->buffer, 8);
    buffer_val |= data_val;
    memcpy(frame->buffer, &buffer_val, 8);

    frame->current_bit_usage += bit_len;
}

// 【修改点2】 send 函数实现更新
void can_tx_drv_t::send(const uint32_t id, can_drv_t *driver)
{
    // 检查传入的 driver 是否有效，而不是检查 this->_can_driver
    if (!driver) return;

    for (auto &frame : instance()->_frames)
    {
        if (frame.id == id)
        {
            // 使用传入的 driver 发送
            driver->send_msg(frame.id, frame.buffer);
            return;
        }
    }
}

} // namespace pyro