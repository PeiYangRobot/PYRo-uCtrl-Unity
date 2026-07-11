/**
 * @file pyro_ui_drv.cpp
 * @brief RoboMaster Referee System UI Driver Implementation
 */

#include "pyro_ui_drv.h"
#include <cstring>
#include <cmath>

#ifdef REFEREE_UART

namespace pyro
{

ui_drv_t::ui_drv_t(referee_drv_t *referee) : _referee(referee)
{
    _buffer.reserve(20); // 预分配空间
}

bool ui_drv_t::clear_layer(const uint8_t layer) const
{
    const uint8_t data[2] = {1, layer}; // 1: 删除图层
    return _referee->send_ui_interaction(
        static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DELETE), data);
}

bool ui_drv_t::clear_all() const
{
    constexpr uint8_t data[2] = {2, 0}; // 2: 删除所有
    return _referee->send_ui_interaction(
        static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DELETE), data);
}

ui_figure_data_t ui_drv_t::create_base_figure(const char name[3], ui_operate op,
                                              ui_figure type, uint8_t layer,
                                              ui_color color, uint16_t width,
                                              uint16_t start_x,
                                              uint16_t start_y)
{
    ui_figure_data_t fig{};
    std::memcpy(fig.figure_name, name, 3);
    fig.operate_type = static_cast<uint32_t>(op);
    fig.figure_type  = static_cast<uint32_t>(type);
    fig.layer        = layer;
    fig.color        = static_cast<uint32_t>(color);
    fig.width        = width;
    fig.start_x      = start_x;
    fig.start_y      = start_y;
    return fig;
}

void ui_drv_t::check_and_try_send()
{
    // 如果缓存满了 7 个，则直接打包发送这 7 个图形并从队列中移除
    if (_buffer.size() >= 7)
    {
        _referee->send_ui_interaction(
            static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_7),
            &_buffer[0]);
        _buffer.erase(_buffer.begin(), _buffer.begin() + 7);
    }
}

ui_drv_t &ui_drv_t::draw_line(const char name[3], ui_operate op, uint8_t layer,
                              ui_color color, uint16_t width, uint16_t start_x,
                              uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    auto fig      = create_base_figure(name, op, ui_figure::LINE, layer, color,
                                       width, start_x, start_y);
    fig.details_d = end_x;
    fig.details_e = end_y;
    _buffer.push_back(fig);
    check_and_try_send();
    return *this;
}

ui_drv_t &ui_drv_t::draw_rect(const char name[3], ui_operate op, uint8_t layer,
                              ui_color color, uint16_t width, uint16_t start_x,
                              uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    auto fig      = create_base_figure(name, op, ui_figure::RECT, layer, color,
                                       width, start_x, start_y);
    fig.details_d = end_x;
    fig.details_e = end_y;
    _buffer.push_back(fig);
    check_and_try_send();
    return *this;
}

ui_drv_t &ui_drv_t::draw_circle(const char name[3], ui_operate op,
                                uint8_t layer, ui_color color, uint16_t width,
                                uint16_t center_x, uint16_t center_y,
                                uint16_t radius)
{
    auto fig = create_base_figure(name, op, ui_figure::CIRCLE, layer, color,
                                  width, center_x, center_y);
    fig.details_c = radius;
    _buffer.push_back(fig);
    check_and_try_send();
    return *this;
}

ui_drv_t &ui_drv_t::draw_ellipse(const char name[3], ui_operate op,
                                 uint8_t layer, ui_color color, uint16_t width,
                                 uint16_t center_x, uint16_t center_y,
                                 uint16_t rx, uint16_t ry)
{
    auto fig = create_base_figure(name, op, ui_figure::ELLIPSE, layer, color,
                                  width, center_x, center_y);
    fig.details_c = rx;
    fig.details_d = ry;
    _buffer.push_back(fig);
    check_and_try_send();
    return *this;
}

ui_drv_t &ui_drv_t::draw_arc(const char name[3], ui_operate op, uint8_t layer,
                             ui_color color, uint16_t width, uint16_t center_x,
                             uint16_t center_y, uint16_t start_angle,
                             uint16_t end_angle, uint16_t rx, uint16_t ry)
{
    auto fig = create_base_figure(name, op, ui_figure::ARC, layer, color, width,
                                  center_x, center_y);
    fig.details_a = start_angle;
    fig.details_b = end_angle;
    fig.details_c = rx;
    fig.details_d = ry;
    _buffer.push_back(fig);
    check_and_try_send();
    return *this;
}

ui_drv_t &ui_drv_t::draw_float(const char name[3], ui_operate op, uint8_t layer,
                               ui_color color, uint16_t font_size,
                               uint16_t width, uint16_t start_x,
                               uint16_t start_y, float value)
{
    auto fig      = create_base_figure(name, op, ui_figure::FLOAT, layer, color,
                                       width, start_x, start_y);
    fig.details_a = font_size;

    int32_t i_val = static_cast<int32_t>(std::round(value * 1000.0f));
    auto u_val    = *reinterpret_cast<uint32_t *>(&i_val);

    fig.details_c = (u_val & 0x3FF);
    fig.details_d = ((u_val >> 10) & 0x7FF);
    fig.details_e = ((u_val >> 21) & 0x7FF);

    _buffer.push_back(fig);
    check_and_try_send();
    return *this;
}

ui_drv_t &ui_drv_t::draw_int(const char name[3], ui_operate op, uint8_t layer,
                             ui_color color, uint16_t font_size, uint16_t width,
                             uint16_t start_x, uint16_t start_y, int32_t value)
{
    auto fig = create_base_figure(name, op, ui_figure::INT, layer, color, width,
                                  start_x, start_y);
    fig.details_a = font_size;

    auto u_val    = static_cast<uint32_t>(value);
    fig.details_c = (u_val & 0x3FF);
    fig.details_d = ((u_val >> 10) & 0x7FF);
    fig.details_e = ((u_val >> 21) & 0x7FF);

    _buffer.push_back(fig);
    check_and_try_send();
    return *this;
}

bool ui_drv_t::draw_string(const char name[3], ui_operate op, uint8_t layer,
                           ui_color color, uint16_t font_size, uint16_t width,
                           uint16_t start_x, uint16_t start_y,
                           const std::string &text) const
{
    ui_string_data_t str_pkt{};
    str_pkt.figure = create_base_figure(name, op, ui_figure::STRING, layer,
                                        color, width, start_x, start_y);
    str_pkt.figure.details_a = font_size;

    size_t len = text.length();
    if (len > 30)
        len = 30;
    str_pkt.figure.details_b = len;

    std::memset(str_pkt.text, 0, 30);
    std::memcpy(str_pkt.text, text.c_str(), len);

    return _referee->send_ui_interaction(
        static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_CHAR), &str_pkt);
}

bool ui_drv_t::flush()
{
    if (_buffer.empty() || !_referee)
        return true;

    bool success = true;
    size_t i     = 0;

    // 清理最后的残余图形包 (数量必定小于 7)
    while (i < _buffer.size())
    {
        size_t remain = _buffer.size() - i;

        if (remain >= 7) // 正常情况下由于 check_and_try_send，此处不会触发 7 包
        {
            success &= _referee->send_ui_interaction(
                static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_7),
                &_buffer[i]);
            i += 7;
        }
        else if (remain >= 5)
        {
            success &= _referee->send_ui_interaction(
                static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_5),
                &_buffer[i]);
            i += 5;
        }
        else if (remain >= 2)
        {
            success &= _referee->send_ui_interaction(
                static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_2),
                &_buffer[i]);
            i += 2;
        }
        else if (remain == 1)
        {
            success &= _referee->send_ui_interaction(
                static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_1),
                &_buffer[i]);
            i += 1;
        }
    }

    _buffer.clear();
    return success;
}

} // namespace pyro

#endif