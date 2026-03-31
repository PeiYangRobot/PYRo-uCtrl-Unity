#include "pyro_module_base.h"
#include "pyro_mec_chassis.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_direct_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_uart_comm.h"
#include "struct.h"

using namespace pyro;
static pyro::direct_gimbal_t *direct_gimbal_ptr           = nullptr;
static pyro::direct_gimbal_cmd_t *direct_gimbal_cmd_ptr   = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *dr16_ctrl_ptr = nullptr;
static pyro::vt03_drv_t::vt03_ctrl_t const *vt03_ctrl_ptr = nullptr;
static void gimbal_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl);
static void gimbal_vt032cmd(vt03_drv_t::vt03_ctrl_t const *rc_ctrl);
static void chassis_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl);
static void chassis_vt032cmd(vt03_drv_t::vt03_ctrl_t const *rc_ctrl);

extern StateBytes state_bytes;

extern "C"
{
    void hero_gimbal_thread(void *argument)
    {
        while (true)
        {
            if (rc_hub_t::get_instance(rc_hub_t::VT03)->check_online())
            {
                chassis_vt032cmd(vt03_ctrl_ptr);
                gimbal_vt032cmd(vt03_ctrl_ptr);
            }
            else if (rc_hub_t::get_instance(rc_hub_t::DR16)->check_online())
            {
                chassis_dr162cmd(dr16_ctrl_ptr);
                gimbal_dr162cmd(dr16_ctrl_ptr);
            }
            direct_gimbal_ptr->set_command(*direct_gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_gimbal_init(void *argument)
    {
        direct_gimbal_cmd_ptr = new pyro::direct_gimbal_cmd_t();
        direct_gimbal_ptr     = pyro::direct_gimbal_t::instance();
        dr16_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        vt03_ctrl_ptr = static_cast<pyro::vt03_drv_t::vt03_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->read());
        direct_gimbal_ptr->start();
        xTaskCreate(hero_gimbal_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}


void gimbal_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl)
{
    pyro::read_scope_lock lock(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());

    if (pyro::dr16_drv_t::sw_state_t::SW_DOWN == rc_ctrl->rc.s_r.state)
    {
        direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        direct_gimbal_cmd_ptr->pitch_delta_angle = 0;
        direct_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }
    direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
    if (pyro::dr16_drv_t::sw_state_t::SW_UP == rc_ctrl->rc.s_r.state)
    {
        direct_gimbal_cmd_ptr->auto_aim = true;
        direct_gimbal_cmd_ptr->target_pitch =
            state_bytes.input_data.shoot_pitch;
        direct_gimbal_cmd_ptr->target_yaw = state_bytes.input_data.shoot_yaw;
    }
    else
    {
        direct_gimbal_cmd_ptr->auto_aim          = false;
        direct_gimbal_cmd_ptr->pitch_delta_angle = -rc_ctrl->rc.ch_ry * 0.002f;
        direct_gimbal_cmd_ptr->yaw_delta_angle   = -rc_ctrl->rc.ch_rx * 0.0035f;
    }
}
void gimbal_vt032cmd(vt03_drv_t::vt03_ctrl_t const *rc_ctrl)
{
    pyro::read_scope_lock lock(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->get_lock());
    if (vt03_drv_t::gear_state_t::GEAR_LEFT == rc_ctrl->rc.gear.state)
    {
        direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        direct_gimbal_cmd_ptr->pitch_delta_angle = 0;
        direct_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }
    direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
    if (vt03_drv_t::gear_state_t::GEAR_RIGHT == rc_ctrl->rc.gear.state)
    {
        direct_gimbal_cmd_ptr->auto_aim = true;
    }
    else
    {
        direct_gimbal_cmd_ptr->auto_aim = false;
    }
    if (direct_gimbal_cmd_ptr->auto_aim)
    {
        direct_gimbal_cmd_ptr->target_pitch =
            state_bytes.input_data.shoot_pitch;
        direct_gimbal_cmd_ptr->target_yaw = state_bytes.input_data.shoot_yaw;
    }
    else
    {
        direct_gimbal_cmd_ptr->pitch_delta_angle =
            -rc_ctrl->rc.ch_ry * 0.0035f - rc_ctrl->mouse.y * 0.25f;
        direct_gimbal_cmd_ptr->yaw_delta_angle =
            -rc_ctrl->rc.ch_rx * 0.0035f - rc_ctrl->mouse.x * 0.6f;
    }
}

void chassis_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl)
{
    pyro::read_scope_lock lock(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());

    static int8_t vx      = 0;
    static int8_t vy      = 0;
    static int8_t wz      = 0;
    static uint8_t active = 0;

    pyro::can_tx_drv_t::clear(0x101);

    if (pyro::dr16_drv_t::sw_state_t::SW_MID != rc_ctrl->rc.s_r.state)
    {
        vx     = 0;
        vy     = 0;
        wz     = 0;
        active = 0;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can3));
        return;
    }
    vx     = static_cast<int8_t>(rc_ctrl->rc.ch_ly * 127);
    vy     = static_cast<int8_t>(-rc_ctrl->rc.ch_lx * 127);
    wz     = 0;
    active = 1;
    pyro::can_tx_drv_t::add_data(0x101, 8, vx);
    pyro::can_tx_drv_t::add_data(0x101, 8, vy);
    pyro::can_tx_drv_t::add_data(0x101, 8, wz);
    pyro::can_tx_drv_t::add_data(0x101, 1, active);
    pyro::can_tx_drv_t::send(0x101,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can3));
}

void chassis_vt032cmd(vt03_drv_t::vt03_ctrl_t const *rc_ctrl)
{
    pyro::read_scope_lock lock(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->get_lock());
    static int8_t vx      = 0;
    static int8_t vy      = 0;
    static int8_t wz      = 0;
    static uint8_t active = 0;

    pyro::can_tx_drv_t::clear(0x101);
    if (vt03_drv_t::gear_state_t::GEAR_MID != rc_ctrl->rc.gear.state)
    {
        vx     = 0;
        vy     = 0;
        wz     = 0;
        active = 0;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can3));
        return;
    }
    vx     = static_cast<int8_t>(rc_ctrl->key.w.state   ? 127
                                 : rc_ctrl->key.s.state ? -127
                                                        : rc_ctrl->rc.ch_ly * 127);
    vy     = static_cast<int8_t>(rc_ctrl->key.a.state   ? 127
                                 : rc_ctrl->key.d.state ? -127
                                                        : -rc_ctrl->rc.ch_lx * 127);
    static float shift_using_time = 0;
    static bool gyroscope_en = false;
    if (vt03_drv_t::key_ctrl_t::KEY_PRESSED == rc_ctrl->key.shift.ctrl
        && shift_using_time != rc_ctrl->key.shift.change_time)
    {
        gyroscope_en = !gyroscope_en;
        shift_using_time = rc_ctrl->key.shift.change_time;
    }
    wz     = static_cast<int8_t>(rc_ctrl->rc.wheel * 127);
    if (gyroscope_en)
    {
        wz = 127;
    }
    active = 1;
    pyro::can_tx_drv_t::add_data(0x101, 8, vx);
    pyro::can_tx_drv_t::add_data(0x101, 8, vy);
    pyro::can_tx_drv_t::add_data(0x101, 8, wz);
    pyro::can_tx_drv_t::add_data(0x101, 1, active);
    pyro::can_tx_drv_t::send(0x101,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can3));
}
