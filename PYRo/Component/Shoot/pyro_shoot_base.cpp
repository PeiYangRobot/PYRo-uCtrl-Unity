#include "pyro_shoot_base.h"

namespace pyro
{
shoot_base_t::shoot_base_t()
{
    rc_drv_t *dr16_drv = rc_hub_t::get_instance(rc_hub_t::which_rc_t::DR16);

}

void shoot_base_t::set_continuous_mode_delay(uint16_t delay)
{
    _continuous_mode_delay = delay;}

void shoot_base_t::dr16_cmd(void const *rc_ctrl)
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const*>(
            rc_ctrl);

    // if(dr16_drv_t::DR16_SW_UP == p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state)
    // {
    //     _ready_mode = SHOOT_READY_STOP;
    //     _local_mode = SHOOT_STOP;
    //     _total_mode = ZERO_FORCE;
    // }
    // else if(dr16_drv_t::DR16_SW_MID == p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state)
    // {
    //     _total_mode = RC_CONTROL;
    // }
    // else if(dr16_drv_t::DR16_SW_DOWN == p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state)
    // {
    //     _total_mode = AUTO_AIM_CONTROL;
    // }
    //
    // if(RC_CONTROL == _total_mode)
    // {
    //     if(dr16_drv_t::DR16_SW_UP == p_ctrl->rc.s[dr16_drv_t::DR16_SW_LEFT].state)
    //     {
    //         _ready_mode = SHOOT_READY_STOP;
    //     }
    //     else
    //     {
    //         if(dr16_drv_t::DR16_SW_UP_TO_MID == p_ctrl->rc.s[dr16_drv_t::DR16_SW_LEFT].ctrl)
    //         {
    //             _ready_mode = SHOOT_READY_SETUP;
    //             _continuous_delay = 0;
    //         }
    //         else if(dr16_drv_t::DR16_SW_MID_TO_DOWN == p_ctrl->rc.s[dr16_drv_t::DR16_SW_LEFT].ctrl)
    //         {
    //             _ready_mode = SHOOT_READY_START;
    //             _continuous_delay = 0;
    //         }
    //         else if(dr16_drv_t::DR16_SW_DOWN == p_ctrl->rc.s[dr16_drv_t::DR16_SW_LEFT].state)
    //         {
    //             _continuous_delay++;
    //             if(_continuous_delay >= _continuous_mode_delay)
    //             {
    //                 _ready_mode = SHOOT_READY_CONTINUOUS;
    //             }
    //         }
    //         else if(dr16_drv_t::DR16_SW_DOWN_TO_MID == p_ctrl->rc.s[dr16_drv_t::DR16_SW_LEFT].ctrl)
    //         {
    //             _ready_mode = SHOOT_READY_SETUP;
    //         }
    //
    //     }
        
    // }
}


void shoot_base_t::vt03_cmd()
{
}

void shoot_base_t::set_fric_speed(float target_speed)
{
}

void shoot_base_t::update_feedback()
{
}

void shoot_base_t::zero_force()
{
}

void shoot_base_t::control()
{
}
}