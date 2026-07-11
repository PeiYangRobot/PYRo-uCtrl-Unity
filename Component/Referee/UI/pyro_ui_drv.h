/**
 * @file pyro_ui_drv.h
 * @brief RoboMaster Referee System UI Driver (Modern C++ Style)
 */

#ifndef PYRO_UI_H
#define PYRO_UI_H

#include "pyro_referee.h"
#include <vector>
#include <string>
#include <cstdint>

#ifdef REFEREE_UART

namespace pyro
{

enum class ui_operate : uint32_t
{
    NULL_OP = 0,
    ADD     = 1,
    MODIFY  = 2,
    DELETE  = 3
};
enum class ui_figure : uint32_t
{
    LINE    = 0,
    RECT    = 1,
    CIRCLE  = 2,
    ELLIPSE = 3,
    ARC     = 4,
    FLOAT   = 5,
    INT     = 6,
    STRING  = 7
};
enum class ui_color : uint32_t
{
    ALLY    = 0,
    YELLOW  = 1,
    GREEN   = 2,
    ORANGE  = 3,
    MAGENTA = 4,
    PINK    = 5,
    CYAN    = 6,
    BLACK   = 7,
    WHITE   = 8
};

#pragma pack(push, 1)
struct ui_figure_data_t
{
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type  : 3;
    uint32_t layer        : 4;
    uint32_t color        : 4;
    uint32_t details_a    : 9;
    uint32_t details_b    : 9;
    uint32_t width        : 10;
    uint32_t start_x      : 11;
    uint32_t start_y      : 11;
    uint32_t details_c    : 10;
    uint32_t details_d    : 11;
    uint32_t details_e    : 11;
};

struct ui_string_data_t
{
    ui_figure_data_t figure;
    uint8_t text[30];
};
#pragma pack(pop)

class ui_drv_t
{
  public:
    explicit ui_drv_t(referee_drv_t *referee);

    bool clear_layer(uint8_t layer) const;
    bool clear_all() const;

    ui_drv_t &draw_line(const char name[3], ui_operate op, uint8_t layer,
                        ui_color color, uint16_t width, uint16_t start_x,
                        uint16_t start_y, uint16_t end_x, uint16_t end_y);

    ui_drv_t &draw_rect(const char name[3], ui_operate op, uint8_t layer,
                        ui_color color, uint16_t width, uint16_t start_x,
                        uint16_t start_y, uint16_t end_x, uint16_t end_y);

    ui_drv_t &draw_circle(const char name[3], ui_operate op, uint8_t layer,
                          ui_color color, uint16_t width, uint16_t center_x,
                          uint16_t center_y, uint16_t radius);

    ui_drv_t &draw_ellipse(const char name[3], ui_operate op, uint8_t layer,
                           ui_color color, uint16_t width, uint16_t center_x,
                           uint16_t center_y, uint16_t rx, uint16_t ry);

    ui_drv_t &draw_arc(const char name[3], ui_operate op, uint8_t layer,
                       ui_color color, uint16_t width, uint16_t center_x,
                       uint16_t center_y, uint16_t start_angle,
                       uint16_t end_angle, uint16_t rx, uint16_t ry);

    ui_drv_t &draw_float(const char name[3], ui_operate op, uint8_t layer,
                         ui_color color, uint16_t font_size, uint16_t width,
                         uint16_t start_x, uint16_t start_y, float value);

    ui_drv_t &draw_int(const char name[3], ui_operate op, uint8_t layer,
                       ui_color color, uint16_t font_size, uint16_t width,
                       uint16_t start_x, uint16_t start_y, int32_t value);

    bool draw_string(const char name[3], ui_operate op, uint8_t layer,
                     ui_color color, uint16_t font_size, uint16_t width,
                     uint16_t start_x, uint16_t start_y,
                     const std::string &text) const;

    bool flush();

  private:
    static ui_figure_data_t create_base_figure(const char name[3],
                                               ui_operate op, ui_figure type,
                                               uint8_t layer, ui_color color,
                                               uint16_t width, uint16_t start_x,
                                               uint16_t start_y);
    void check_and_try_send();

    referee_drv_t *_referee;
    std::vector<ui_figure_data_t> _buffer;
};

} // namespace pyro

#endif
#endif // PYRO_UI_H