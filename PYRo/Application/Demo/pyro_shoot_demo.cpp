#include "pyro_core_config.h"
#if SHOOT_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_shoot_base.h"

#ifdef __cplusplus

pyro::rc_drv_t *dr16_drv;
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

    pyro::dji_m2006_motor_drv_t *m2006_drv;

    pyro::fric_drv_t *fric_drv_1;
    pyro::fric_drv_t *fric_drv_2;

    pyro::trigger_drv_t *trigger_drv;

    pyro::pid_ctrl_t *speed_pid_1;
    pyro::pid_ctrl_t *speed_pid_2;
    pyro::pid_ctrl_t *speed_pid_3;
    pyro::pid_ctrl_t *positon_pid;
    
    // pyro::shoot_drv_t *shoot_drv;

    void pyro_shoot_demo(void *arg)
    { 
    //             pyro::get_uart5().enable_rx_dma();
    //     dr16_drv = new pyro::dr16_drv_t(&pyro::get_uart5());
    //     dr16_drv->init();
    //     dr16_drv->enable();

    //     pyro::can_hub_t::get_instance();
    //     can1_drv = new pyro::can_drv_t(&hfdcan1);
    //     can2_drv = new pyro::can_drv_t(&hfdcan2);
    //     can3_drv = new pyro::can_drv_t(&hfdcan3);

    //     can1_drv->init();
    //     can2_drv->init();
    //     can3_drv->init();

    //     can1_drv->start();
    //     can2_drv->start();
    //     can3_drv->start();

    //     speed_pid_1 = new pyro::pid_ctrl_t(7.0f, 0.0f, 0.00f);
    //     speed_pid_2 = new pyro::pid_ctrl_t(7.0f, 0.0f, 0.00f);
    //     speed_pid_3 = new pyro::pid_ctrl_t(5.0f, 0.1f, 0.00f);

    //     speed_pid_1->set_output_limits(15.0f);
    //     speed_pid_2->set_output_limits(15.0f);
    //     speed_pid_3->set_output_limits(15.0f);

    //     positon_pid = new pyro::pid_ctrl_t(0.01f, 0.01f, 0.00f);

    //     positon_pid->set_output_limits(15.0f);

    //     m3508_drv_1 = new pyro::dji_m3508_motor_drv_t(
    //         pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can2);
    //     m3508_drv_2 = new pyro::dji_m3508_motor_drv_t(
    //         pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can2);
    //     m2006_drv = new pyro::dji_m2006_motor_drv_t(
    //         pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);


    //     fric_drv_1 = new pyro::fric_drv_t(
    //         m3508_drv_1, 
    //         *speed_pid_1,
    //         0.03f);

    //     fric_drv_2 = new pyro::fric_drv_t(
    //         m3508_drv_2, 
    //         *speed_pid_2,
    //         0.03f);

    //     trigger_drv = new pyro::trigger_drv_t(
    //         m2006_drv,
    //         *speed_pid_3,
    //         *positon_pid);

    //     fric_drv_1->set_speed(15.0f);
    //     fric_drv_2->set_speed(-15.0f);
    //     trigger_drv->set_rotate(-10.0f);

    //     trigger_drv->set_gear_ratio(36.0f);

    //     shoot_drv = new pyro::shoot_drv_t(
    //         fric_drv_1,
    //         fric_drv_2,
    //         trigger_drv,
    //         dr16_drv);

    //     while (true)
    //     {
    //         shoot_drv->update_feedback();
    //         shoot_drv->shoot_control();

    //         vTaskDelay(1);
    //     }
    }



}

#endif
#endif
