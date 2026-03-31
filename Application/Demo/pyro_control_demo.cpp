#include "pyro_core_config.h"
#if CONTROL_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_rud_chassis.h"
#include "pyro_rw_lock.h"
#include "stm32h723xx.h"
#include "Yaw/pyro_yaw.h"
#include "pyro_dji_motor_drv.h"

#ifdef __cplusplus
using namespace pyro;
rud_cfg_t rud_config{};
yaw_cfg_t yaw_config{};
extern "C"
{
    void pyro_control_demo(void *arg)
    {
        pyro::rud_cmd_t rud_cmd_obj{};
        pyro::yaw_cmd_t yaw_cmd_obj{};
        pyro::dr16_drv_t::dr16_ctrl_t dr16_data;


        rud_config.motor.rudder[0] =
            new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                        can_hub_t::can2); // FL Rudder
        rud_config.motor.rudder[1] =
            new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                        can_hub_t::can2); // BL Rudder
        rud_config.motor.rudder[2] =
            new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                        can_hub_t::can1); // BR Rudder
        rud_config.motor.rudder[3] =
            new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                        can_hub_t::can1); // FR Rudder

        rud_config.motor.wheel[0] =
            new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                      can_hub_t::can2); // FL Wheel
        rud_config.motor.wheel[1] =
            new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                      can_hub_t::can2); // BL Wheel
        rud_config.motor.wheel[2] =
            new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                      can_hub_t::can1); // BR Wheel
        rud_config.motor.wheel[3] =
            new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                      can_hub_t::can1); // FR Wheel

        rud_config.pid.wheel_pid[0] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        rud_config.pid.wheel_pid[1] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        rud_config.pid.wheel_pid[2] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        rud_config.pid.wheel_pid[3] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);

        rud_config.pid.rud_pos_pid[0] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rud_config.pid.rud_pos_pid[1] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rud_config.pid.rud_pos_pid[2] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rud_config.pid.rud_pos_pid[3] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);

        rud_config.pid.rud_spd_pid[0] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rud_config.pid.rud_spd_pid[1] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rud_config.pid.rud_spd_pid[2] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rud_config.pid.rud_spd_pid[3] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);

        rud_config.pid.follow_yaw_pid =
            new pid_t(3.6f, 0.01f, 0.003f, 0.1f, 5.0f);

        rud_config.rud_pos_moving_offset[0] = 1.01472831f;
        rud_config.rud_pos_moving_offset[0] = -0.29145637f;
        rud_config.rud_pos_moving_offset[0] = -1.87299052f;
        rud_config.rud_pos_moving_offset[0] = -1.04003897f;        

        power_control_drv_t &power_controller = power_control_drv_t::get_instance();
        power_control_drv_t::motor_coefficient_t coef1;
        coef1.k1 = 0;
        coef1.k2 = 0;
        coef1.k3 = 0;
        coef1.k4 = 0;
        power_controller.set_motor_coefficient(1, coef1);

        power_control_drv_t::motor_coefficient_t coef2;
        coef2.k1 = 0;
        coef2.k2 = 0;
        coef2.k3 = 0;
        coef2.k4 = 0;
        power_controller.set_motor_coefficient(2, coef2);

        power_control_drv_t::motor_coefficient_t coef3;
        coef3.k1 = 0;
        coef3.k2 = 0;
        coef3.k3 = 0;
        coef3.k4 = 0;
        power_controller.set_motor_coefficient(3, coef3);

        power_control_drv_t::motor_coefficient_t coef4;
        coef4.k1 = 0;
        coef4.k2 = 0;
        coef4.k3 = 0;
        coef4.k4 = 0;
        power_controller.set_motor_coefficient(4, coef4);



        yaw_config.motor.yaw =
            new dm_motor_drv_t(0x01, 0x02, pyro::can_hub_t::can2);
        yaw_config.motor.yaw->set_position_range(-PI, PI);
        yaw_config.motor.yaw->set_rotate_range(-20, 20);
        yaw_config.motor.yaw->set_torque_range(-10, 10);

        yaw_config.pid.yaw_pos_pid =
            new pid_t(20.0f, 0.2f, 0.02f, 0.5f, 10.0f, 15, 150, 4);
        yaw_config.pid.yaw_spd_pid =
            new pid_t(0.3f, 0.003f, 0.0003f, 0.1f, 3.0f, 15, 150, 4);

        yaw_config.yaw_offset = -2.40028524f;

        pyro::rud_chassis_t::instance()->configure(rud_config);
        pyro::yaw_t::instance()->configure(yaw_config);
        pyro::rud_chassis_t::instance()->start();
        pyro::yaw_t::instance()->start();

        while (true)
        {
            {
                pyro::rc_drv_t *dr16_drv =
                    pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
                pyro::read_scope_lock rc_read_lock(dr16_drv->get_lock());
                const auto *p_dr16 =
                    static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(
                        dr16_drv->read());
                if (p_dr16 != nullptr)
                    dr16_data = *p_dr16;
            }
            if (abs(dr16_data.rc.ch_lx) < 0.3f &&
                abs(dr16_data.rc.ch_ly) < 0.3f &&
                abs(dr16_data.rc.ch_rx) < 0.3f)
            {
                dr16_data.rc.ch_lx = 0.0f;
                dr16_data.rc.ch_ly = 0.0f;
                dr16_data.rc.ch_rx = 0.0f;
            }

            // chassis_control
            if (pyro::dr16_drv_t::sw_state_t::SW_UP == dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode       = pyro::cmd_base_t::mode_t::PASSIVE;
                yaw_cmd_obj.mode       = pyro::cmd_base_t::mode_t::PASSIVE;
                rud_cmd_obj.follow_yaw = false;
                rud_cmd_obj.timestamp  = 0;
                rud_cmd_obj.vx         = 0.0f;
                rud_cmd_obj.vy         = 0.0f;
                rud_cmd_obj.wz         = 0.0f;
                rud_cmd_obj.yaw_error  = 0.0f;
            }
            else if (pyro::dr16_drv_t::sw_state_t::SW_MID ==
                     dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode       = pyro::cmd_base_t::mode_t::ACTIVE;
                yaw_cmd_obj.mode       = pyro::cmd_base_t::mode_t::ACTIVE;
                rud_cmd_obj.follow_yaw = true;
                rud_cmd_obj.timestamp  = 0;
                rud_cmd_obj.vx         = dr16_data.rc.ch_lx * 2.0f;
                rud_cmd_obj.vy         = dr16_data.rc.ch_ly * 2.0f;
                yaw_cmd_obj.target_yaw_imu_angle -= dr16_data.rc.ch_rx * 0.01f;
                rud_cmd_obj.yaw_error =
                    pyro::yaw_t::instance()->get_yaw_error();
            }
            else if (pyro::dr16_drv_t::sw_state_t::SW_DOWN ==
                     dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode       = pyro::cmd_base_t::mode_t::ACTIVE;
                yaw_cmd_obj.mode       = pyro::cmd_base_t::mode_t::ACTIVE;
                rud_cmd_obj.follow_yaw = false;
                rud_cmd_obj.timestamp  = 0;
                rud_cmd_obj.vx =
                    dr16_data.rc.ch_lx *
                        cosf(pyro::yaw_t::instance()->get_yaw_error()) -
                    dr16_data.rc.ch_ly *
                        sinf(pyro::yaw_t::instance()->get_yaw_error());
                rud_cmd_obj.vy =
                    dr16_data.rc.ch_ly *
                        cosf(pyro::yaw_t::instance()->get_yaw_error()) +
                    dr16_data.rc.ch_lx *
                        sinf(pyro::yaw_t::instance()->get_yaw_error());
                rud_cmd_obj.wz = 2.0f;
                yaw_cmd_obj.target_yaw_imu_angle -= dr16_data.rc.ch_rx * 0.01f;
            }

            pyro::rud_chassis_t::instance()->set_command(rud_cmd_obj);
            pyro::yaw_t::instance()->set_command(yaw_cmd_obj);

            vTaskDelay(1);
        }
    }
}
#endif
#endif
