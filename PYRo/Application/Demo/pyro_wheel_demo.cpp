#include "pyro_core_config.h"
#if WHEEL_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_chassis_drv.h"
#include "pyro_dr16_rc_drv.h"

#ifdef __cplusplus

pyro::rc_drv_t *dr16_drv;

float offset_angle_1 = 0.0f;
float offset_angle_2 = 0.0f;

extern "C"
{
    pyro::can_drv_t *can1_drv;
    pyro::can_drv_t *can2_drv;
    pyro::can_drv_t *can3_drv;

    std::array<uint8_t, 8> can1_data;
    std::array<uint8_t, 8> can2_data;

    std::vector<uint8_t> can1_data_vec;
    std::vector<uint8_t> can2_data_vec;

    pyro::dji_m3508_motor_drv_t *m3508_drv_1;
    pyro::dji_m3508_motor_drv_t *m3508_drv_2;
    pyro::dji_m3508_motor_drv_t *m3508_drv_3;
    pyro::dji_m3508_motor_drv_t *m3508_drv_4;

    pyro::dji_gm_6020_motor_drv_t *gm6020_drv_1;
    pyro::dji_gm_6020_motor_drv_t *gm6020_drv_2;

    pyro::wheel_drv_t *wheel_drv_1;
    pyro::wheel_drv_t *wheel_drv_2;
    pyro::wheel_drv_t *wheel_drv_3;
    pyro::wheel_drv_t *wheel_drv_4;

    pyro::pid_ctrl_t *speed_pid_1;
    pyro::pid_ctrl_t *speed_pid_2;
    pyro::pid_ctrl_t *speed_pid_3;
    pyro::pid_ctrl_t *speed_pid_4;

    pyro::steering_wheel_drv_t *steering_wheel_drv_1;
    pyro::steering_wheel_drv_t *steering_wheel_drv_2;

    pyro::chassis_drv_t *chassis_drv;

    pyro::pid_ctrl_t *rudder_rotate_pid_1;
    pyro::pid_ctrl_t *rudder_position_pid_1;
    pyro::pid_ctrl_t *rudder_rotate_pid_2;
    pyro::pid_ctrl_t *rudder_position_pid_2;

    void pyro_wheel_demo(void *arg)
    {
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart5)->enable_rx_dma();
        dr16_drv = new pyro::dr16_drv_t(pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart5));
        dr16_drv->init();
        dr16_drv->enable();

        pyro::can_hub_t::get_instance();
        can1_drv = new pyro::can_drv_t(&hfdcan1);
        can2_drv = new pyro::can_drv_t(&hfdcan2);
        can3_drv = new pyro::can_drv_t(&hfdcan3);

        can1_drv->init();
        can2_drv->init();
        can3_drv->init();

        can1_drv->start();
        can2_drv->start();
        can3_drv->start();

        speed_pid_1 = new pyro::pid_ctrl_t(20.0f, 0.0f, 0.00f);
        speed_pid_2 = new pyro::pid_ctrl_t(20.0f, 0.0f, 0.00f);
        speed_pid_3 = new pyro::pid_ctrl_t(20.0f, 0.0f, 0.00f);
        speed_pid_4 = new pyro::pid_ctrl_t(20.0f, 0.0f, 0.00f);

        speed_pid_1->set_output_limits(100.0f);
        speed_pid_2->set_output_limits(100.0f);
        speed_pid_3->set_output_limits(100.0f);
        speed_pid_4->set_output_limits(100.0f);

        rudder_position_pid_1 = new pyro::pid_ctrl_t(20.0f, 0.0f, 0.00f);
        rudder_position_pid_2 = new pyro::pid_ctrl_t(20.0f, 0.0f, 0.00f);
        rudder_rotate_pid_1 = new pyro::pid_ctrl_t(0.3f, 0.0f, 0.00f);
        rudder_rotate_pid_2 = new pyro::pid_ctrl_t(0.3f, 0.0f, 0.00f);

        rudder_position_pid_1->set_output_limits(1000.0f);
        rudder_position_pid_2->set_output_limits(1000.0f);
        rudder_rotate_pid_1->set_output_limits(3.0f);
        rudder_rotate_pid_2->set_output_limits(3.0f);

        rudder_position_pid_1->set_integral_limits(50.0f);
        rudder_rotate_pid_1->set_integral_limits(50.0f);
        rudder_position_pid_2->set_integral_limits(50.0f);
        rudder_rotate_pid_2->set_integral_limits(50.0f);

        m3508_drv_1 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can2);
        m3508_drv_2 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);
        m3508_drv_3 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can1);
        m3508_drv_4 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can1);

        gm6020_drv_1 = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);
        gm6020_drv_2 = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can1);

        wheel_drv_1 = new pyro::wheel_drv_t(
            m3508_drv_1,
            *speed_pid_1,
            0.065f);

        wheel_drv_2 = new pyro::wheel_drv_t(
            m3508_drv_2,
            *speed_pid_2,
            0.06f);

        wheel_drv_3 = new pyro::wheel_drv_t(
           m3508_drv_3,
            *speed_pid_3,
            0.06f);
        
        wheel_drv_4 = new pyro::wheel_drv_t(
            m3508_drv_4,
            *speed_pid_4,
            0.065f);

        wheel_drv_1->set_gear_ratio(19.0f);
        wheel_drv_2->set_gear_ratio(19.0f);
        wheel_drv_3->set_gear_ratio(19.0f);
        wheel_drv_4->set_gear_ratio(19.0f);

        steering_wheel_drv_1 = new pyro::steering_wheel_drv_t(
            wheel_drv_2,
            gm6020_drv_1,
            *rudder_rotate_pid_1,
            *rudder_position_pid_1);

        steering_wheel_drv_2 = new pyro::steering_wheel_drv_t(
            wheel_drv_3,
            gm6020_drv_2,
            *rudder_rotate_pid_2,
            *rudder_position_pid_2);

        steering_wheel_drv_1->set_offset_radian(0.959505022f);
        steering_wheel_drv_2->set_offset_radian(4.52447653f);

        chassis_drv = new pyro::chassis_drv_t(
            steering_wheel_drv_1,
            steering_wheel_drv_2,
            wheel_drv_1,
            wheel_drv_4,
            dr16_drv);

        while (true)
        {
            chassis_drv->update_feedback();

            chassis_drv->chassis_control();

            vTaskDelay(1);
        }
    }
}
#endif
#endif
