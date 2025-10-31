#include "pyro_core_config.h"
#include "pyro_uart_drv.h"
#if RC_DEMO_EN
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_uart_drv.h"
#include "pyro_rc_hub.h"

#ifdef __cplusplus

extern "C"
{
    pyro::rc_drv_t *dr16_drv;
    void pyro_rc_demo(void *arg)
    {
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart5)->enable_rx_dma();
        dr16_drv = pyro::rc_hub_t::get_instance(
            pyro::rc_hub_t::DR16);
        dr16_drv->init();
        dr16_drv->enable();
        static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            dr16_drv->get_p_ctrl());
        static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            dr16_drv->get_p_last_ctrl());
        vTaskDelete(nullptr);
    }
}
#endif
#endif
