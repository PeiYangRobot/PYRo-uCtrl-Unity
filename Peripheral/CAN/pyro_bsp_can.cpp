/**
 * @file pyro_bsp_can.cpp
 * @brief Implementation of the Board Support Package for CAN.
 * CAN板级支持包实现文件。
 *
 * @author Antigravity
 * @version 1.0.0
 * @date 2026-07-03
 */

#include "pyro_bsp_can.h"
#include "fdcan.h"

namespace pyro
{

can_drv_t& bsp_can::get_can1()
{
    static can_drv_t instance(&hfdcan1);
    return instance;
}

can_drv_t& bsp_can::get_can2()
{
    static can_drv_t instance(&hfdcan2);
    return instance;
}

can_drv_t& bsp_can::get_can3()
{
    static can_drv_t instance(&hfdcan3);
    return instance;
}

can_drv_t* bsp_can::get_can(const which_can which)
{
    switch (which)
    {
        case can1:
            return &get_can1();
        case can2:
            return &get_can2();
        case can3:
            return &get_can3();
        default:
            return nullptr;
    }
}

status_t bsp_can::init_all()
{
    CHECK_PYRO_RET(get_can1().init());
    CHECK_PYRO_RET(get_can2().init());
    CHECK_PYRO_RET(get_can3().init());

    CHECK_PYRO_RET(get_can1().start());
    CHECK_PYRO_RET(get_can2().start());
    CHECK_PYRO_RET(get_can3().start());
    return PYRO_OK;
}

} // namespace pyro
