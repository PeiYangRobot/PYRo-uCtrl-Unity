#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_direct_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_quad_booster.h"
#include "pyro_com_canrx.h"
#include "struct.h"

using namespace pyro;

extern StateBytes state_bytes;
extern float read_time;

static pyro::quad_booster_t *quad_booster_ptr             = nullptr;
static pyro::quad_booster_cmd_t *quad_booster_cmd_ptr     = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *dr16_ctrl_ptr = nullptr;
static pyro::vt03_drv_t::vt03_ctrl_t const *vt03_ctrl_ptr = nullptr;
static bool flush_flag                                    = false;

void booster_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl)
{
    pyro::read_scope_lock lock(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());

    if (pyro::dr16_drv_t::sw_state_t::SW_DOWN == rc_ctrl->rc.s_r.state)
    {
        quad_booster_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
        quad_booster_cmd_ptr->fric_on     = false;
        quad_booster_cmd_ptr->fire_enable = false;
        return;
    }
    quad_booster_cmd_ptr->mode         = pyro::cmd_base_t::mode_t::ACTIVE;
    quad_booster_cmd_ptr->target_speed = 11.5f; // 可调节
    // 摩擦轮控制
    static float sl_using_time         = 0;
    if (pyro::dr16_drv_t::sw_ctrl_t::SW_UP_TO_MID == rc_ctrl->rc.s_l.ctrl &&
        rc_ctrl->rc.s_l.change_time != sl_using_time)
    {
        sl_using_time                 = rc_ctrl->rc.s_l.change_time;
        quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
    }

    if (pyro::dr16_drv_t::sw_state_t::SW_MID == rc_ctrl->rc.s_r.state)
    {
        if (pyro::dr16_drv_t::sw_ctrl_t::SW_DOWN_TO_MID ==
                rc_ctrl->rc.s_l.ctrl &&
            rc_ctrl->rc.s_l.change_time != sl_using_time)
        {
            sl_using_time                     = rc_ctrl->rc.s_l.change_time;
            quad_booster_cmd_ptr->fire_enable = true;
        }
        else
        {
            quad_booster_cmd_ptr->fire_enable = false;
        }
    }
    else
    {
        static bool autoaim_fire_flag = false;
        if (pyro::dr16_drv_t::sw_state_t::SW_UP == rc_ctrl->rc.s_r.state)
        {
            if (state_bytes.input_data.fire == 0)
            {
                autoaim_fire_flag = true;
            }
            if (autoaim_fire_flag)
            {
                if (state_bytes.input_data.fire == 1)
                {
                    quad_booster_cmd_ptr->fire_enable = true;
                }
            }
        }
    }
    // 开火控制 (单发）
}

void booster_vt032cmd(vt03_drv_t::vt03_ctrl_t const *rc_ctrl)
{
    pyro::read_scope_lock lock(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->get_lock());

    static float r_using_time = 0;
    static float f_using_time = 0;
    if (rc_ctrl->key.ctrl.state)
    {
        if (rc_ctrl->key.c.state)
        {
            flush_flag = true;
        }
    }
    else
    {
        flush_flag = false;
    }
    if (rc_ctrl->key.g.state)
    {
        if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.r.ctrl &&
            rc_ctrl->key.r.change_time != r_using_time)
        {
            r_using_time = rc_ctrl->key.r.change_time;
            quad_booster_ptr->get_ctx().shoot_data.fric1_mps -= 0.1f;
        }
        if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.f.ctrl &&
            rc_ctrl->key.f.change_time != f_using_time)
        {
            f_using_time = rc_ctrl->key.f.change_time;
            quad_booster_ptr->get_ctx().shoot_data.fric2_mps -= 0.1f;
        }
    }

    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.r.ctrl &&
        rc_ctrl->key.r.change_time != r_using_time)
    {
        r_using_time = rc_ctrl->key.r.change_time;
        quad_booster_ptr->get_ctx().shoot_data.fric1_mps += 0.1f;
    }
    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.f.ctrl &&
        rc_ctrl->key.f.change_time != f_using_time)
    {
        f_using_time = rc_ctrl->key.f.change_time;
        quad_booster_ptr->get_ctx().shoot_data.fric2_mps += 0.1f;
    }

    static float v_using_time = 0;
    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.v.ctrl &&
        rc_ctrl->key.v.change_time != v_using_time)
    {
        v_using_time = rc_ctrl->key.v.change_time;
        quad_booster_cmd_ptr->speed_contorl_en =
            !quad_booster_cmd_ptr->speed_contorl_en;
    }

    if (vt03_drv_t::gear_state_t::GEAR_LEFT == rc_ctrl->rc.gear.state)
    {
        quad_booster_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
        quad_booster_cmd_ptr->fric_on     = false;
        quad_booster_cmd_ptr->fire_enable = false;
        return;
    }
    quad_booster_cmd_ptr->mode         = pyro::cmd_base_t::mode_t::ACTIVE;
    quad_booster_cmd_ptr->target_speed = 11.5f; // 可调节
    // 摩擦轮控制
    static float fn_l_using_time       = 0;
    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->rc.fn_l.ctrl &&
        rc_ctrl->rc.fn_l.change_time != fn_l_using_time)
    {
        fn_l_using_time               = rc_ctrl->rc.fn_l.change_time;
        quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
    }
    static float q_using_time = 0;
    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.q.ctrl &&
        rc_ctrl->key.q.change_time != q_using_time)
    {
        q_using_time                  = rc_ctrl->key.q.change_time;
        quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
    }
    // 开火控制 (单发）
    static float trigger_using_time    = 0;
    static float mouse_left_using_time = 0;
    static bool autoaim_fire_flag      = false;
    if (vt03_drv_t::gear_state_t::GEAR_RIGHT == rc_ctrl->rc.gear.state)
    {
        if (state_bytes.input_data.fire == 0)
        {
            autoaim_fire_flag = true;
        }
        if (autoaim_fire_flag)
        {
            if (state_bytes.input_data.fire == 1)
            {
                quad_booster_cmd_ptr->fire_enable = true;
            }
        }
    }
    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->rc.trigger.ctrl &&
        rc_ctrl->rc.trigger.change_time != trigger_using_time)
    {
        trigger_using_time                = rc_ctrl->rc.trigger.change_time;
        quad_booster_cmd_ptr->fire_enable = true;
    }
    else if (vt03_drv_t::key_ctrl_t::KEY_PRESSED ==
                 rc_ctrl->mouse.press_l.ctrl &&
             rc_ctrl->mouse.press_l.change_time != mouse_left_using_time)
    {
        mouse_left_using_time             = rc_ctrl->mouse.press_l.change_time;
        quad_booster_cmd_ptr->fire_enable = true;
    }
    else
    {
        quad_booster_cmd_ptr->fire_enable = false;
    }

    static float b_using_time = 0;
    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.b.ctrl &&
        rc_ctrl->key.b.change_time != b_using_time)
    {
        b_using_time                     = rc_ctrl->key.b.change_time;
        quad_booster_cmd_ptr->reset_trig = true;
    }
    else
    {
        quad_booster_cmd_ptr->reset_trig = false;
    }
}

void booster2ui()
{
    pyro::can_tx_drv_t::clear(0x110);
    static auto fric1_speed = static_cast<int8_t>(
        quad_booster_ptr->get_ctx().shoot_data.fric1_mps * 10);
    static auto fric2_speed = static_cast<int8_t>(
        quad_booster_ptr->get_ctx().shoot_data.fric2_mps * 10);
    static bool fric_on               = false;
    static bool speed_control_enabled = false;
    static bool fric1_online;
    static bool fric2_online;
    if (quad_booster_cmd_ptr->fric_on)
    {
        fric_on     = true;
        fric1_speed = static_cast<int8_t>(
            quad_booster_ptr->get_ctx().shoot_data.fric1_mps * 10);
        fric2_speed = static_cast<int8_t>(
            quad_booster_ptr->get_ctx().shoot_data.fric2_mps * 10);
        speed_control_enabled = quad_booster_cmd_ptr->speed_contorl_en;
        if (quad_booster_ptr->get_ctx().data.fric_online[1] &&
            quad_booster_ptr->get_ctx().data.fric_online[3])
        {
            fric1_online = true;
        }
        else
        {
            fric1_online = false;
        }
        if (quad_booster_ptr->get_ctx().data.fric_online[0] &&
            quad_booster_ptr->get_ctx().data.fric_online[2])
        {
            fric2_online = true;
        }
        else
        {
            fric2_online = false;
        }
    }
    else
    {
        fric_on     = false;
        fric1_speed = 0;
        fric2_speed = 0;
    }
    pyro::can_tx_drv_t::add_data(0x110, 8, fric1_speed);
    pyro::can_tx_drv_t::add_data(0x110, 8, fric2_speed);
    pyro::can_tx_drv_t::add_data(0x110, 1, fric_on);
    pyro::can_tx_drv_t::add_data(0x110, 1, flush_flag);
    pyro::can_tx_drv_t::add_data(0x110, 1, speed_control_enabled);
    pyro::can_tx_drv_t::add_data(0x110, 1, fric1_online);
    pyro::can_tx_drv_t::add_data(0x110, 1, fric2_online);

    pyro::can_tx_drv_t::send(0x110,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can3));
}
extern "C"
{
    void hero_booster_thread(void *argument)
    {
        while (true)
        {
            if (rc_hub_t::get_instance(rc_hub_t::VT03)->check_online())
            {
                booster_vt032cmd(vt03_ctrl_ptr);
            }
            else if (rc_hub_t::get_instance(rc_hub_t::DR16)->check_online())
            {
                booster_dr162cmd(dr16_ctrl_ptr);
            }
            booster2ui();
            quad_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_booster_init(void *argument)
    {
        quad_booster_ptr     = pyro::quad_booster_t::instance();
        quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();
        dr16_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        vt03_ctrl_ptr = static_cast<pyro::vt03_drv_t::vt03_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->read());
        quad_booster_ptr->start();
        xTaskCreate(hero_booster_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}
