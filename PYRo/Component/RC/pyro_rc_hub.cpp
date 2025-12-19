/**
* @file pyro_rc_hub.cpp
 * @brief Implementation file for the PYRO RC Hub.
 *
 * Implements the `get_instance` factory method which constructs
 * and returns the specific RC driver singletons (DR16, VT03).
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-11-14
 * @copyright [Copyright Information Here]
 */

#include "pyro_rc_hub.h"

namespace pyro
{
/**
 * @brief Gets the singleton instance of a specific RC driver.
 *
 * This method uses static initialization (Magic Statics) to create
 * the required UART driver and the corresponding RC driver
 * (DR16 or VT03) on their first call.
 *
 * @param which_rc The type of RC driver to get (e.g., DR16).
 * @return Pointer to the `rc_drv_t` base class of the requested driver.
 */
rc_drv_t *rc_hub_t::get_instance(which_rc_t which_rc)
{
    switch (which_rc)
    {
        case DR16:
        {
            static uart_drv_t *dr16_uart =
                uart_drv_t::get_instance(uart_drv_t::uart5);
            static dr16_drv_t dr16_rc_drv(dr16_uart);
            return &dr16_rc_drv;
        }
        case VT03:
        {
            static uart_drv_t *vt03_uart =
                uart_drv_t::get_instance(uart_drv_t::uart1);
            static vt03_drv_t vt03_rc_drv(vt03_uart);
            return &vt03_rc_drv;
        }
        default:;
    }
    return nullptr;
}

} // namespace pyro