#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_com_canrx.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_com_cantx.h"
#include "pyro_referee.h"

using namespace pyro;

static pyro::hybrid_chassis_t *hybrid_chassis_ptr       = nullptr;
static pyro::hybrid_cmd_t *hybrid_cmd_ptr               = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;
static pyro::hybrid_deps_t *hybrid_deps_ptr             = nullptr;
static void chassis_rxcmd(void const *rc_ctrl);
static void chassis_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl);
static void deps_init();
static void quaternion_tx();

extern "C"
{


    void hero_chassis_thread(void *argument)
    {
        while (true)
        {
            chassis_rxcmd(rc_ctrl_ptr);
            // chassis_dr162cmd(rc_ctrl_ptr);
            quaternion_tx();
            hybrid_chassis_ptr->set_command(*hybrid_cmd_ptr);
            vTaskDelay(1);
        }
    }
    void hero_chassis_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can1, 0x101);
        hybrid_cmd_ptr     = new pyro::hybrid_cmd_t();
        hybrid_chassis_ptr = pyro::hybrid_chassis_t::instance();
        rc_ctrl_ptr        = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        deps_init();
        hybrid_chassis_ptr->configure(*hybrid_deps_ptr);
        hybrid_chassis_ptr->start();
        xTaskCreate(hero_chassis_thread, "start_hero_chassis_thread", 128,
                    nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}
void chassis_rxcmd(void const *rc_ctrl)
{
    std::array<uint8_t, 8> raw_data{};
    if (pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x101,
                                     raw_data))
    {
        hybrid_cmd_ptr->vx =
            2.5f * static_cast<float>(static_cast<int8_t>(raw_data[0])) /
            127.0f;
        hybrid_cmd_ptr->vy =
            2.5f * static_cast<float>(static_cast<int8_t>(raw_data[1])) /
            127.0f;
        hybrid_cmd_ptr->mode =
            static_cast<pyro::cmd_base_t::mode_t>(raw_data[3] & 0x01);
        hybrid_cmd_ptr->track_en    = true;
        hybrid_cmd_ptr->leg_retract = (raw_data[3] & 0x04) != 0;
    }
}


void chassis_dr162cmd(dr16_drv_t::dr16_ctrl_t const *rc_ctrl)
{
    pyro::read_scope_lock lock(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());

    if (pyro::dr16_drv_t::sw_state_t::SW_MID != rc_ctrl->rc.s_r.state)
    {
        hybrid_cmd_ptr->vx          = 0;
        hybrid_cmd_ptr->vy          = 0;
        hybrid_cmd_ptr->wz          = 0;
        hybrid_cmd_ptr->delta_pitch = 0;
        hybrid_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
    }
    else
    {
        hybrid_cmd_ptr->vx          = 2 * rc_ctrl->rc.ch_ly;
        hybrid_cmd_ptr->vy          = -2 * rc_ctrl->rc.ch_lx;
        hybrid_cmd_ptr->delta_pitch = 0.002f * rc_ctrl->rc.ch_ry;
        hybrid_cmd_ptr->delta_yaw   = -0.003f * rc_ctrl->rc.ch_rx;
        hybrid_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::ACTIVE;
        if (pyro::dr16_drv_t::sw_state_t::SW_DOWN != rc_ctrl->rc.s_l.state)
        {
            hybrid_cmd_ptr->track_en = true;
            if (pyro::dr16_drv_t::sw_state_t::SW_MID == rc_ctrl->rc.s_l.state)
            {
                hybrid_cmd_ptr->leg_retract = true;
            }
            else
            {
                hybrid_cmd_ptr->leg_retract = false;
            }
        }
        else
        {
            hybrid_cmd_ptr->track_en = false;
        }
    }
}

static void deps_init()
{
    hybrid_deps_ptr = new pyro::hybrid_deps_t();
    hybrid_deps_ptr->motor_deps.mecanum[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::can_hub_t::can3); // FL Wheel
    hybrid_deps_ptr->motor_deps.mecanum[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::can_hub_t::can3); // FR Wheel
    hybrid_deps_ptr->motor_deps.mecanum[2] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::can_hub_t::can3); // RL Wheel
    hybrid_deps_ptr->motor_deps.mecanum[3] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                        pyro::can_hub_t::can3); // RR Wheel
    hybrid_deps_ptr->motor_deps.track[0] =
        new pyro::dm_motor_drv_t(0x11, 0x21, pyro::can_hub_t::can3);
    hybrid_deps_ptr->motor_deps.track[1] =
        new pyro::dm_motor_drv_t(0x12, 0x22, pyro::can_hub_t::can3);
    hybrid_deps_ptr->motor_deps.leg[0] =
        new pyro::dm_motor_drv_t(0x31, 0x41, pyro::can_hub_t::can2);
    hybrid_deps_ptr->motor_deps.leg[1] =
        new pyro::dm_motor_drv_t(0x32, 0x42, pyro::can_hub_t::can2);

    hybrid_deps_ptr->motor_deps.yaw = new pyro::dji_gm_6020_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can1);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_torque_range(-11.0f, 11.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_torque_range(-11.0f, 11.0f);

    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_rotate_range(-5.655f, 5.655f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_rotate_range(-5.655f, 5.655f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_torque_range(-27.0f, 27.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_torque_range(-27.0f, 27.0f);
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    hybrid_deps_ptr->pid_deps.mecanum_pid[0] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[1] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[2] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[3] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);

    hybrid_deps_ptr->pid_deps.follow_yaw_pid =
        new pid_t(5.0f, 0.0f, 0.1f, 0.0f, 10.0f, 200, 100, 4);

    hybrid_deps_ptr->pid_deps.track_pid[0] =
        new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.track_pid[1] =
        new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 20, 10, 4);

    hybrid_deps_ptr->pid_deps.pitch_pid = new pid_t(
        260.0f, 0.1f, 20.0f, 20.0f, 200.0f, 200, 100, 4,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);
    hybrid_deps_ptr->pid_deps.roll_pid = new pid_t(
        360.0f, 0.1f, 10.0f, 20.0f, 200.0f, 200, 100, 4,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);

    hybrid_deps_ptr->pid_deps.leg_pos_pid[0] =
        new pid_t(12.0f, 0.12f, 0.008f, 0.1f, 0.6f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.leg_pos_pid[1] =
        new pid_t(12.0f, 0.12f, 0.008f, 0.1f, 0.6f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.leg_vel_pid[0] =
        new pid_t(500.0f, 4.0f, 0.008f, 10.0f, 300.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.leg_vel_pid[1] =
        new pid_t(500.0f, 4.0f, 0.008f, 10.0f, 300.0f, 20, 10, 4);
    // hybrid_deps_ptr->pid_deps.leg_pos_pid[0] =
    // new pid_t(7.2f, 0.1f, 0.02f, 0.2f, 7.0f, 20, 10, 4);
    // hybrid_deps_ptr->pid_deps.leg_pos_pid[1] =
    //     new pid_t(7.2f, 0.1f, 0.02f, 0.2f, 7.0f, 20, 10, 4);
    // hybrid_deps_ptr->pid_deps.leg_vel_pid[0] =
    //     new pid_t(1000.0f, 2.0f, 0.2f, 2000.0f, 10000.0f, 20, 10, 4);
    // hybrid_deps_ptr->pid_deps.leg_vel_pid[1] =
    //     new pid_t(1000.0f, 2.0f, 0.2f, 2000.0f, 10000.0f, 20, 10, 4);
}

void quaternion_tx()
{
    static float q0, q1, q2, q3;
    static uint8_t calibration_times = 0;
    static bool last_gimbal_out =
        referee_drv_t::get_instance()
            ->get_data()
            .robot_status.power_management_gimbal_output;
    ins_drv_t::get_instance()->get_quaternion(&q0, &q1, &q2, &q3);
    // 假设你已经算出了底盘的四元数 (float 类型, 范围 [-1.0, 1.0])
    pyro::can_tx_drv_t::clear(0x103);
    pyro::can_tx_drv_t::clear(0x105);
    // 缩放并转换为 int16_t
    const auto send_q0 = static_cast<int16_t>(q0 * 32767.0f);
    const auto send_q1 = static_cast<int16_t>(q1 * 32767.0f);
    const auto send_q2 = static_cast<int16_t>(q2 * 32767.0f);
    const auto send_q3 = static_cast<int16_t>(q3 * 32767.0f);

    if (last_gimbal_out == false &&
        referee_drv_t::get_instance()
                ->get_data()
                .robot_status.power_management_gimbal_output == true)
    {
        calibration_times++;
    }
    last_gimbal_out = referee_drv_t::get_instance()
                          ->get_data()
                          .robot_status.power_management_gimbal_output;


    can_tx_drv_t::add_data(0x103, 16, send_q0);
    can_tx_drv_t::add_data(0x103, 16, send_q1);
    can_tx_drv_t::add_data(0x103, 16, send_q2);
    can_tx_drv_t::add_data(0x103, 16, send_q3);

    can_tx_drv_t::add_data(0x105, 8, calibration_times);

    can_tx_drv_t::send(
        0x103, can_hub_t::get_instance()->hub_get_can_obj(can_hub_t::can1));
    can_tx_drv_t::send(
        0x105, can_hub_t::get_instance()->hub_get_can_obj(can_hub_t::can1));
}
