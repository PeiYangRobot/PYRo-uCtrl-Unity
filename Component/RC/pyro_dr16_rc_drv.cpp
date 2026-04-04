#include "pyro_dr16_rc_drv.h"
#include "pyro_bsp_uart.h"


namespace pyro
{
static constexpr uint16_t DR16_CH_VALUE_OFFSET = 1024;
#ifdef DR16_UART
dr16_drv_t &dr16_drv_t::instance()
{
    // 传入对单例对象的解引用（引用传递）
    static dr16_drv_t _inst(DR16_UART);
    return _inst;
}
#endif

dr16_drv_t::dr16_drv_t(uart_drv_t &dr16_uart)
    : rc_drv_t(dr16_uart, "dr16_task", 1, sizeof(dr16_buf_t))
{
}

bool dr16_drv_t::check_packet(const uint8_t *buf)
{
    return true;
}

status_t dr16_drv_t::error_check(const dr16_buf_t *dr16_buf)
{
    if (dr16_buf->ch0 < 364 || dr16_buf->ch0 > 1684)
        return PYRO_ERROR;
    if (dr16_buf->ch1 < 364 || dr16_buf->ch1 > 1684)
        return PYRO_ERROR;
    if (dr16_buf->ch2 < 364 || dr16_buf->ch2 > 1684)
        return PYRO_ERROR;
    if (dr16_buf->ch3 < 364 || dr16_buf->ch3 > 1684)
        return PYRO_ERROR;
    return PYRO_OK;
}

void dr16_drv_t::unpack(const uint8_t *buf)
{
    auto *dr16_buf = reinterpret_cast<const dr16_buf_t *>(buf);

    if (PYRO_OK == error_check(dr16_buf))
    {
        write_scope_lock rc_write_lock(get_lock());

        shared_v_rc.axes.rx =
            static_cast<float>(dr16_buf->ch0 - DR16_CH_VALUE_OFFSET) / 660.0f;
        shared_v_rc.axes.ry =
            static_cast<float>(dr16_buf->ch1 - DR16_CH_VALUE_OFFSET) / 660.0f;
        shared_v_rc.axes.lx =
            static_cast<float>(dr16_buf->ch2 - DR16_CH_VALUE_OFFSET) / 660.0f;
        shared_v_rc.axes.ly =
            static_cast<float>(dr16_buf->ch3 - DR16_CH_VALUE_OFFSET) / 660.0f;
        shared_v_rc.axes.wheel =
            static_cast<float>(dr16_buf->wheel - DR16_CH_VALUE_OFFSET) / 660.0f;

        auto map_sw = [](const uint8_t raw)
        {
            if (raw == 1)
                return sw_pos_t::UP;
            if (raw == 3)
                return sw_pos_t::MID;
            if (raw == 2)
                return sw_pos_t::DOWN;
            return sw_pos_t::UNKNOWN;
        };
        shared_v_rc.switches.right.update(map_sw(dr16_buf->s1));
        shared_v_rc.switches.left.update(map_sw(dr16_buf->s2));

        shared_v_rc.mouse_axes.x =
            static_cast<float>(dr16_buf->mouse_x) / 32768.0f;
        shared_v_rc.mouse_axes.y =
            static_cast<float>(dr16_buf->mouse_y) / 32768.0f;
        shared_v_rc.mouse_axes.z =
            static_cast<float>(dr16_buf->mouse_z) / 32768.0f;

        shared_v_rc.buttons.press_l.update(dr16_buf->press_l);
        shared_v_rc.buttons.press_r.update(dr16_buf->press_r);

        const uint16_t kc = dr16_buf->key_code;
        shared_v_rc.keys.w.update(kc & (1 << 0));
        shared_v_rc.keys.s.update(kc & (1 << 1));
        shared_v_rc.keys.a.update(kc & (1 << 2));
        shared_v_rc.keys.d.update(kc & (1 << 3));
        shared_v_rc.keys.shift.update(kc & (1 << 4));
        shared_v_rc.keys.ctrl.update(kc & (1 << 5));
        shared_v_rc.keys.q.update(kc & (1 << 6));
        shared_v_rc.keys.e.update(kc & (1 << 7));
        shared_v_rc.keys.r.update(kc & (1 << 8));
        shared_v_rc.keys.f.update(kc & (1 << 9));
        shared_v_rc.keys.g.update(kc & (1 << 10));
        shared_v_rc.keys.z.update(kc & (1 << 11));
        shared_v_rc.keys.x.update(kc & (1 << 12));
        shared_v_rc.keys.c.update(kc & (1 << 13));
        shared_v_rc.keys.v.update(kc & (1 << 14));
        shared_v_rc.keys.b.update(kc & (1 << 15));
    }
}
} // namespace pyro

