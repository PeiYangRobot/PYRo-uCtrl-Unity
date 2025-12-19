#include "pyro_shoot_17mm_control.h"

namespace pyro
{
shoot_17mm_control_t::shoot_17mm_control_t(trigger_drv_t *trigger_drv,
                                           fric_drv_t *fric_drv_1,
                                           fric_drv_t *fric_drv_2
                                           )
     : _trigger_drv(trigger_drv),
       _fric_drv{fric_drv_1, fric_drv_2}
{
}

void shoot_17mm_control_t::set_trigger_rotate(float target_rotate)
{
    _trigger_rotate = target_rotate;
}

void shoot_17mm_control_t::set_fric_speed(float target_speed)
{
    _fric_speed = target_speed;
}
    
void shoot_17mm_control_t::update_feedback()
{
    _fric_drv[0]->update_feedback();
    _fric_drv[1]->update_feedback();
    _trigger_drv->update_feedback();
}

void shoot_17mm_control_t::zero_force()
{
    if(abs(_fric_drv[0]->get_speed()) < 0.2f && abs(_fric_drv[1]->get_speed()) < 0.2f)
    {
        _fric_drv[0]->zero_force();
        _fric_drv[1]->zero_force();
        _trigger_drv->zero_force();
    }
    else
    {
        _fric_drv[0]->set_speed(0.0f);
        _fric_drv[1]->set_speed(0.0f);
        _fric_drv[0]->control();
        _fric_drv[1]->control();
        _trigger_drv->zero_force();
    }
    _fric_drv[0]->zero_force();
    _fric_drv[1]->zero_force();
    _trigger_drv->zero_force();
}

void shoot_17mm_control_t::set_control()
{   
    if(RC_CONTROL == _total_mode) 
    {
        if(SHOOT_READY_STOP == _ready_mode)
        {
            _local_mode = SHOOT_STOP;
        }
        else
        {
            if(SHOOT_STOP == _local_mode)
            {
                if(SHOOT_READY_SETUP == _ready_mode)
                {
                    _local_mode = SHOOT_SETUP;
                }
            }
            else if(SHOOT_SETUP == _local_mode)
            {
                if(abs(_trigger_drv->get_radian() - _trigger_drv->get_target_radian()) > 0.1f)
                {
                    _local_mode = SHOOT_CALIBRATION;
                }
                if(abs(_fric_drv[0]->get_speed() - _fric_drv[0]->get_target_speed()) < 0.5f &&
                   abs(_fric_drv[1]->get_speed() - _fric_drv[1]->get_target_speed()) < 0.5f 
                   && abs(_trigger_drv->get_rotate()) < 0.02f
                )
                {
                    _local_mode = SHOOT_READY;
                }
            }
            else if(SHOOT_READY == _local_mode)
            {
                if(SHOOT_READY_START == _ready_mode)
                {
                    _local_mode = SHOOT_START;
                }
                if(SHOOT_READY_CONTINUOUS == _ready_mode)
                {
                    _local_mode = SHOOT_CONTINUOUS;
                }
            }
            else if(SHOOT_START == _local_mode)
            {
                _trigger_drv->step_forward();
                _local_mode = SHOOT_WAIT;
            }
            else if(SHOOT_WAIT == _local_mode)
            {
                if(abs(_trigger_drv->get_radian() - _trigger_drv->get_target_radian()) < 0.05f)
                {
                    _local_mode = SHOOT_SETUP;
                }
            }
            else if(SHOOT_CONTINUOUS == _local_mode)
            {
                if(SHOOT_READY_SETUP == _ready_mode)
                {
                    _local_mode = SHOOT_SETUP;
                }
            }
            else if(SHOOT_CALIBRATION == _local_mode)
            {
                if(abs(_trigger_drv->get_rotate()) < 0.05f)
                {
                    _local_mode = SHOOT_SETUP;
                }
            }
            else if(SHOOT_CALIBRATION == _local_mode)
            {
                if(_trigger_drv->get_rotate() < 0.1f)
                {
                    _local_mode = SHOOT_SETUP;
                }
            }




            // if(SHOOT_READY_SETUP == _ready_mode)
            // {
            //     _local_mode = SHOOT_SETUP;
            // }

            // if(SHOOT_SETUP == _local_mode)
            // {
            //     if(abs(_fric_drv[0]->get_speed() - _fric_drv[0]->get_target_speed()) < 0.5f &&
            //     abs(_fric_drv[1]->get_speed() - _fric_drv[1]->get_target_speed()) < 0.5f)
            //     {
            //         _local_mode = SHOOT_READY;
            //     }  
            // }

            // if(SHOOT_READY == _local_mode)
            // {
            //     if(SHOOT_READY_START == _ready_mode)
            //     {
            //         _local_mode = SHOOT_START;
            //     }
            // }

            // if(SHOOT_WAIT == _local_mode)
            // {
            //     if(SHOOT_READY_START == _ready_mode)
            //     {
            //         _local_mode = SHOOT_START;
            //     }
            //     else if(SHOOT_READY_CONTINUOUS == _ready_mode)
            //     {
            //         _local_mode = SHOOT_CONTINUOUS;
            //     }
            // }
        }
    }
    else if(AUTO_AIM_CONTROL == _total_mode)
    {
        _local_mode = SHOOT_STOP;
    }
    else
    {
        _local_mode = SHOOT_STOP;
    }
}

void shoot_17mm_control_t::control()
{
    if(RC_CONTROL == _total_mode)
    {
        if(SHOOT_STOP == _local_mode)
        {
            zero_force();
        }
        else
        {
            for(uint8_t i = 0; i < 2; i++)
            {
                _fric_drv[i]->set_speed(_fric_speed);
            }

            if(SHOOT_SETUP == _local_mode)
            {
                _trigger_drv->set_rotate(0.0f);
                // if(_reverse)
                // {
                //     _trigger_drv->set_rotate(-2.0f);
                // }
            }
            else if(SHOOT_READY == _local_mode)
            {
                _trigger_drv->set_rotate(0.0f);
            }
            else if (SHOOT_CONTINUOUS == _local_mode)
            {
                _trigger_drv->set_rotate(_trigger_rotate);
            }
            else if(SHOOT_CALIBRATION == _local_mode)
            {
                _trigger_drv->set_rotate(-3.0f);
            }

            _fric_drv[0]->control();
            _fric_drv[1]->control();
            _trigger_drv->control();



            
            // if(_trigger_lock)
            // {
            // _local_mode = SHOOT_WAIT;
            // }
            // else
            // {
            //     if(SHOOT_SETUP == _local_mode || SHOOT_READY == _local_mode)
            //     {
                    
            //         _trigger_drv->step_forward(0.0f);
            //     }  
            //     else if(SHOOT_START == _local_mode)
            //     {
            //         _trigger_drv->step_forward();
            //         // _trigger_lock = true;
            //         _local_mode = SHOOT_WAIT;
            //     }
            //     else if(SHOOT_CONTINUOUS == _local_mode)
            //     {
            //         _trigger_drv->set_rotate(_trigger_rotate);
            //     }
            //     else if(SHOOT_WAIT == _local_mode)
            //     {
            //         if(abs(_trigger_drv->get_radian() - _trigger_drv->get_target_radian()) < 0.01f)
            //         {
            //             // _trigger_lock = false;
            //         }
            //     }
            // }
        }
    }
    else if(AUTO_AIM_CONTROL == _total_mode)
    {
        zero_force();
    }
    else
    {
        zero_force();
    }

    // for(uint8_t i = 0; i < 2; i++)
    // {
    //     _fric_drv[i]->control();
    // }
    // _trigger_drv->control();
}

}

