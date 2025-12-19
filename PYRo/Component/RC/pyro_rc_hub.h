/**
* @file pyro_rc_hub.h
 * @brief Header file for the PYRO Remote Control (RC) Hub.
 *
 * This file defines the `pyro::rc_hub_t` class, which acts as a
 * factory or central access point (Singleton hub) for obtaining
 * instances of specific RC drivers (like DR16 or VT03).
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-11-14
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_RC_HUB_H__
#define __PYRO_RC_HUB_H__

#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"

namespace pyro
{
/**
 * @brief A factory class to access different RC driver singletons.
 *
 * Provides a static `get_instance` method to retrieve specific
 * RC driver implementations (e.g., DR16, VT03).
 */
class rc_hub_t
{
public:
    /**
     * @brief Default constructor for rc_hub_t.
     * (Note: This class is intended for static use.)
     */
    rc_hub_t() = delete;
    /**
     * @brief Default destructor for rc_hub_t.
     */
    ~rc_hub_t() = delete;

    /**
     * @brief Enum to identify which RC driver instance to retrieve.
     */
    enum which_rc_t
    {
        VT03 = 0,
        DR16 = 1,
    };

    /**
     * @brief Gets the singleton instance of a specific RC driver.
     * @param which_rc The type of RC driver to get (e.g., DR16).
     * @return Pointer to the `rc_drv_t` base class of the requested driver.
     */
    static rc_drv_t *get_instance(which_rc_t which_rc);


};
}
#endif