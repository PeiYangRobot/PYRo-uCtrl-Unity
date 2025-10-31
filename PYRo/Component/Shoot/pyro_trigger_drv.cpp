#include "pyro_trigger_drv.h"
#include "cmsis_os.h"

#define BLOCK_THRESHOLD 400
#define BLOCK_SPEED 0.3f
#define BLOCK_TIME 10

namespace pyro
{

trigger_drv_t::trigger_drv_t(motor_base_t *motor_base,
                             const pid_ctrl_t &_rotate_pid, 
                             const pid_ctrl_t &_position_pid,
                             float step_radian
                     )
    : motor_base(motor_base),
      _rotate_pid(_rotate_pid),
      _position_pid(_position_pid),
      _step_radian(step_radian)
{
}

void trigger_drv_t::set_dt(float dt)
{
    _dt = dt;
}

void trigger_drv_t::set_gear_ratio(float gear_ratio)
{
    _gear_ratio = gear_ratio;
}

void trigger_drv_t::set_rotate(float target_rotate)
{
    if (POSITION == _mode)
    {
        _rotate_pid.reset();
        _position_pid.reset();
    }
    _mode = ROTATE;
    _target_rotate = target_rotate;
}

void trigger_drv_t::set_radian(float target_radian)
{
    if (ROTATE == _mode)
    {
        _rotate_pid.reset();
    }
    _mode = POSITION;
    _target_radian = target_radian;
}

float trigger_drv_t::get_rotate()
{
    return _current_rotate;
}

float trigger_drv_t::get_radian()
{
    return _current_radian;
}

void trigger_drv_t::zero_force()
{
    motor_base->send_torque(0.0f);
}

void trigger_drv_t::update_feedback()
{
    motor_base->update_feedback();
    _current_rotate = motor_base->get_current_rotate() / _gear_ratio;
    _current_motor_radian = motor_base->get_current_position();
    _current_radian = _update_trigger_radian();

}

void trigger_drv_t::control()
{
    update_feedback();
    if(ROTATE == _mode)
    {
        float torque_cmd = _rotate_pid.compute(_target_rotate, _current_rotate, _dt);
        motor_base->send_torque(torque_cmd);
    }
    else if(POSITION == _mode) 
    {
        float rotate_cmd = _position_pid.compute(_target_radian, _current_radian, _dt);
        float torque_cmd = _rotate_pid.compute(rotate_cmd, _current_rotate, _dt);
        motor_base->send_torque(torque_cmd);
    }
}

float trigger_drv_t::_update_trigger_radian()
{
    if(_is_first_update)
    {
        _last_motor_radian = _current_motor_radian + PI;
        _is_first_update = false;
        return _last_motor_radian / _gear_ratio;
    }
    
    float current_motor_radian = _current_motor_radian + PI;

    float delta_motor_radian = current_motor_radian - _last_motor_radian;
    if(delta_motor_radian < -PI)
    {
        _motor_total_count += 1;
        delta_motor_radian += 2 * PI;
    }
    else if(delta_motor_radian > PI)
    {
        _motor_total_count -= 1;
        delta_motor_radian -= 2 * PI;
    }

    _last_motor_radian = current_motor_radian;

    float total_motor_radian = current_motor_radian + _motor_total_count * 2 * PI;
    float gear_radian = total_motor_radian / _gear_ratio;
    if(gear_radian < -PI)
        gear_radian += 2 * PI;
    else if(gear_radian > PI)
        gear_radian -= 2 * PI;

    return gear_radian;
}

}