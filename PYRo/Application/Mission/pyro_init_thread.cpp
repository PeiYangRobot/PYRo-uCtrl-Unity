#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"

extern "C"
{
    pyro::can_drv_t *can1_drv;
    pyro::can_drv_t *can2_drv;
    pyro::can_drv_t *can3_drv;

    void pyro_init_thread(void *argument)
    {
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart1)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart5)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart7)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart10)
            ->enable_rx_dma();

        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->init();
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->enable();

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

        vTaskDelete(nullptr);
    }
}