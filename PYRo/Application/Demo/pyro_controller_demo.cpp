#include "pyro_core_config.h"
#if CONTROLLER_DEMO_EN

#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"

#include "pyro_position_controller.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_pid_ctrl.h"

#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_uart_drv.h"


extern "C"
{
    pyro::can_drv_t *can1_drv;
    pyro::can_drv_t *can2_drv;
    pyro::can_drv_t *can3_drv;

    pyro::position_controller_t *ctrl;
    pyro::dm_motor_drv_t *motor;
    pyro::pid_ctrl_t *spd_pid,*pos_pid;
    float rot;

    float angle=0;

    pyro::dr16_drv_t *dr16_drv;
    void pyro_controller_demo(void *arg)
    {
        pyro::get_uart5().enable_rx_dma();
        dr16_drv = new pyro::dr16_drv_t(&pyro::get_uart5());
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

        motor = new pyro::dm_motor_drv_t(0x5, 0x4, pyro::can_hub_t::can1);
        motor->set_position_range(-pyro::PI, pyro::PI);
        motor->set_rotate_range(-20, 20);
        motor->set_torque_range(-10, 10);
        
        spd_pid   = new pyro::pid_ctrl_t(1.0f, 0.1f, 0.0f);
        spd_pid->set_output_limits(10.0f);
        spd_pid->set_integral_limits(5.0f);

        pos_pid   = new pyro::pid_ctrl_t(1.6f, 0.0f, 0.0f);
        pos_pid->set_output_limits(20.0f);
        pos_pid->set_integral_limits(5.0f);

        ctrl  = new pyro::position_controller_t(
            motor, pos_pid, spd_pid);
        ctrl->set_target(-1.0f);
        vTaskDelay(1000);
        motor->enable();
        for(;;)
        {

            static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->get_p_ctrl());
            static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->get_p_last_ctrl());
            float dangle = static_cast<float>(p_ctrl->rc.ch[0]) / 660.0f * 0.01;
            angle+=dangle;

            if(angle>pyro::PI)
                angle-=pyro::PI*2;
            else if(angle<-pyro::PI)
                angle+=pyro::PI*2;

            ctrl->update();

            ctrl->set_target(angle);

            ctrl->control(0.001f);
            vTaskDelay(1);
        }
    }
}

#endif