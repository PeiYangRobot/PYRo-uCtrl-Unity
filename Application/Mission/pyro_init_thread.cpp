#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "pyro_ms53l0m_drv.h"
#include "pyro_supercap_drv.h"
#include "pyro_referee.h"

namespace pyro
{
extern "C"
{
    can_drv_t *can1_drv;
    can_drv_t *can2_drv;
    can_drv_t *can3_drv;
    ins_drv_t *ins_drv;
    referee_drv_t *referee_drv;
    ms53l0m_drv_t *ms53l0m_drv;


    void pyro_init_thread(void *argument)
    {
        dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        uart_drv_t::get_instance(uart_drv_t::which_uart::uart1)
            ->enable_rx_dma();
        uart_drv_t::get_instance(uart_drv_t::which_uart::uart5)
            ->enable_rx_dma();
        uart_drv_t::get_instance(uart_drv_t::which_uart::uart7)
            ->enable_rx_dma();
        uart_drv_t::get_instance(uart_drv_t::which_uart::uart10)
            ->enable_rx_dma();

        rc_hub_t::get_instance(rc_hub_t::DR16)->enable();
        rc_hub_t::get_instance(rc_hub_t::VT03)->enable();

        can_hub_t::get_instance();
        can1_drv = new can_drv_t(&hfdcan1);
        can2_drv = new can_drv_t(&hfdcan2);
        can3_drv = new can_drv_t(&hfdcan3);
        can1_drv->init();
        can2_drv->init();
        can3_drv->init();
        can1_drv->start();
        can2_drv->start();
        can3_drv->start();

        ins_drv = ins_drv_t::get_instance();
        ins_drv->init();

        referee_drv = referee_drv_t::get_instance();
        referee_drv->init();

        supercap_drv_t::get_instance()->start_rx();

        // ms53l0m_drv = ms53l0m_drv_t::get_instance();
        // ms53l0m_drv->init();


        vTaskDelete(nullptr);
    }
}
} // namespace pyro