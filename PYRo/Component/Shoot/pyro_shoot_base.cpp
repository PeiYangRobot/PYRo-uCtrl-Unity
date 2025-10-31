#include "pyro_shoot_base.h"

namespace pyro
{
shoot_base_t::shoot_base_t()
{
}

void shoot_base_t::set_continuous_mode_delay(uint16_t delay)
{
    _continuous_mode_delay = delay;
}

void shoot_base_t::dr16_cmd()
{
    rc_drv_t *dr16_drv = rc_hub_t::get_instance(rc_hub_t::which_rc_t::DR16);
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            dr16_drv->get_p_ctrl());
    static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            dr16_drv->get_p_last_ctrl());
    if(dr16_drv_t::DR16_SW_UP == static_cast<uint8_t>(p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT]))
    {
        _total_mode = ZERO_FORCE;
    }
    else if(dr16_drv_t::DR16_SW_MID == static_cast<uint8_t>(p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT]))
    {
        _total_mode = RC_CONTROL;
    }
    else if(dr16_drv_t::DR16_SW_DOWN == static_cast<uint8_t>(p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT]))
    {
        _total_mode = AUTO_AIM_CONTROL;
    }

    if(RC_CONTROL == _total_mode)
    {
        switch (static_cast<uint8_t>(p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT]))
        {
        case dr16_drv_t::DR16_SW_UP:
            _ready_mode = SHOOT_READY_STOP;
            break;
        case dr16_drv_t::DR16_SW_MID:
            _ready_mode = SHOOT_READY_SETUP;
            break;
        case dr16_drv_t::DR16_SW_DOWN:
            _ready_mode = SHOOT_READY_START;
            _continuous_delay++;
            if(_continuous_delay >= _continuous_mode_delay)
            {
                _ready_mode = SHOOT_READY_CONTINUOUS;
                _continuous_delay = 0;
            }
            break;
        default:
            break;
        }

    }
}

void shoot_base_t::vt03_cmd()
{
}

}