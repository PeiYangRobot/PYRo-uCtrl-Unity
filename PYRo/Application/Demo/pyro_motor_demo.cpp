#include "pyro_core_config.h"
#if MOTOR_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

#ifdef __cplusplus

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

    pyro::dm_motor_drv_t *dm_drv;
    float rot;

    void pyro_motor_demo(void *arg)
    {
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

        m3508_drv_1 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can2);
        m3508_drv_2 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);
        m3508_drv_3 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can1);
        m3508_drv_4 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can1);

        //dm_drv = new pyro::dm_motor_drv_t(0x10, 0x20, pyro::can_hub_t::can1);
        dm_drv = new pyro::dm_motor_drv_t(0x5, 0x4, pyro::can_hub_t::can1);
        dm_drv->set_position_range(-pyro::PI, pyro::PI);
        dm_drv->set_rotate_range(-20, 20);
        dm_drv->set_torque_range(-10, 10);

        HAL_Delay(1000);
        dm_drv->enable();

        while (true)
        {
            dm_drv->update_feedback();
            dm_drv->send_torque(1);
            rot = dm_drv->get_current_position();

            m3508_drv_1->update_feedback();
            m3508_drv_2->update_feedback();
            m3508_drv_3->update_feedback();
            m3508_drv_4->update_feedback();
            m3508_drv_1->send_torque(0.2);
            m3508_drv_2->send_torque(0.2);
            m3508_drv_3->send_torque(0.2);
            m3508_drv_4->send_torque(0.2);

            vTaskDelay(1);
        }
    }
}
#endif
#endif
