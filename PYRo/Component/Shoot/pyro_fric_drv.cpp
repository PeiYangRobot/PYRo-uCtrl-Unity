#include "pyro_fric_drv.h"

namespace pyro
{
fric_drv_t::fric_drv_t(motor_base_t *motor_base,
                       const pid_t &speed_pid,
                       float radius,
                       rotate_direction_t direction)
    : _motor_base(motor_base),
      _speed_pid(speed_pid),
      _radius(radius),
      _direction(direction)
{
};

void fric_drv_t::set_dt(float dt)
{
    _dt = dt;
}

void fric_drv_t::set_speed(float target_speed)
{
    if (rotate_direction_t::CLOCKWISE == _direction)
    {
        _target_speed = abs(target_speed);
    }
    else 
    {
        _target_speed = -abs(target_speed);
    }
}

void fric_drv_t::zero_force()
{
    _motor_base->send_torque(0.0f);
}

float fric_drv_t::get_speed()
{
    return _current_speed;
}

float fric_drv_t::get_target_speed()
{
    return _target_speed;
}

void fric_drv_t::update_feedback()
{
    _motor_base->update_feedback();
    _current_speed = _motor_base->get_current_rotate() * _radius;
}

void fric_drv_t::control()
{
    float torque_cmd = _speed_pid.calculate(_target_speed, _current_speed);
    _motor_base->send_torque(torque_cmd);
}

}
