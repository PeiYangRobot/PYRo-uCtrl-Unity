#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_screw_gimbal.h"
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
static pyro::quad_deps_t *quad_deps_ptr                   = nullptr;
static void deps_init();
extern "C"
{
    void booster_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());
        if (pyro::dr16_drv_t::sw_state_t::SW_DOWN == rc_ctrl->rc.s_r.state)
        {
            quad_booster_cmd_ptr->mode    = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on = false;
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
        if (vt03_drv_t::gear_state_t::GEAR_LEFT == rc_ctrl->rc.gear.state)
        {
            quad_booster_cmd_ptr->mode    = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on = false;
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
            mouse_left_using_time = rc_ctrl->mouse.press_l.change_time;
            quad_booster_cmd_ptr->fire_enable = true;
        }
        else
        {
            quad_booster_cmd_ptr->fire_enable = false;
        }
    }


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
            quad_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_booster_init(void *argument)
    {
        quad_booster_ptr     = pyro::quad_booster_t::instance();
        quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();
        deps_init();
        quad_booster_ptr->configure(*quad_deps_ptr);
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

void deps_init()
{
    quad_deps_ptr = new pyro::quad_deps_t();
    quad_deps_ptr->motor_deps.fric_wheels[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::can_hub_t::can2); // Fric 1
    quad_deps_ptr->motor_deps.fric_wheels[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::can_hub_t::can2); // Fric 2
    quad_deps_ptr->motor_deps.fric_wheels[2] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::can_hub_t::can2); // Fric 3
    quad_deps_ptr->motor_deps.fric_wheels[3] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                        pyro::can_hub_t::can2); // Fric 4
    quad_deps_ptr->motor_deps.trigger_wheel =
        new pyro::dm_motor_drv_t(0x51, 0x61, pyro::can_hub_t::can1);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_torque_range(-7.0f, 7.0f);
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    quad_deps_ptr->pid_deps.fric_pid[0] =
        new pid_t(6.40f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    quad_deps_ptr->pid_deps.fric_pid[1] =
        new pid_t(11.28f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    quad_deps_ptr->pid_deps.fric_pid[2] =
        new pid_t(6.4f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    quad_deps_ptr->pid_deps.fric_pid[3] =
        new pid_t(11.28f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);

    quad_deps_ptr->pid_deps.trigger_pos_pid =
        new pid_t(20.2f, 0.03f, 0.005f, 1.0f, 20.0f, 200, 100, 4);
    quad_deps_ptr->pid_deps.trigger_spd_pid =
        new pid_t(0.8f, 0.02f, 0.005f, 0.2f, 7.0f, 200, 100, 4);
}