#ifndef __PYRO_VIRTUAL_RC_H__
#define __PYRO_VIRTUAL_RC_H__

#include "pyro_rc_core.h"

namespace pyro
{

struct virtual_rc_t
{
    // 线性连续量 (直接读取)
    struct
    {
        float lx, ly, rx, ry;
        float wheel;
    } axes{};
    struct
    {
        float x, y, z;
    } mouse_axes{};

    // 离散事件控件 (App 层通过指针订阅)
    struct
    {
        tiny_switch_t left;
        tiny_switch_t right;
        tiny_switch_t gear;
    } switches;
    struct
    {
        tiny_button_t trigger, fn_l, fn_r, pause;
        tiny_button_t press_m;
        tiny_button_t press_l, press_r;
    } buttons;

    struct
    {
        tiny_button_t w, s, a, d;
        tiny_button_t shift, ctrl;
        tiny_button_t q, e, r, f, g;
        tiny_button_t z, x, c, v, b;
    } keys;

    void init_all()
    {
        // init(active_high, enable_multi_click, enable_long_press)

        // 核心武器按键，极致0延迟，关闭所有高级判定
        buttons.trigger.init(true, true, true);
        buttons.press_l.init(true, true, true);
        buttons.press_r.init(true, true, true);

        // 其他可能带有复合逻辑的辅助按键
        buttons.fn_l.init(true, true, true);
        buttons.fn_r.init(true, true, true);

        // 键盘按键默认全开高级判定 (不用担心，订阅 PRESS_DOWN 依然是0延迟)
        keys.w.init(true, true, true);
        keys.s.init(true, true, true);
        keys.a.init(true, true, true);
        keys.d.init(true, true, true);
        keys.q.init(true, true, true);
        keys.e.init(true, true, true);
        keys.r.init(true, true, true);
        keys.f.init(true, true, true);
        keys.g.init(true, true, true);
        keys.z.init(true, true, true);
        keys.x.init(true, true, true);
        keys.c.init(true, true, true);
        keys.v.init(true, true, true);
        keys.b.init(true, true, true);

        // 修饰键通常不双击，只长按
        keys.shift.init(true, false, true);
        keys.ctrl.init(true, false, true);
    }
};

} // namespace pyro
#endif