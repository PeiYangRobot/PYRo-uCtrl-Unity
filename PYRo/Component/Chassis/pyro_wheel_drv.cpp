#include "pyro_wheel_drv.h"

namespace pyro
{

wheel_drv_t::wheel_drv_t(motor_base_t *motor_base,
                                   const pid_ctrl_t &speed_pid, float radius)
    : motor_base(motor_base),
      _speed_pid(speed_pid),
      _radius(radius)
      
{
}

void wheel_drv_t::set_gear_ratio(float gear_ratio)
{
    _gear_ratio = gear_ratio;
}

void wheel_drv_t::set_speed(float target_speed)
{
    float torque_cmd = _speed_pid.compute(target_speed, _current_speed, 0.001f);
    motor_base->send_torque(torque_cmd);
    _target_speed = target_speed;
}

void wheel_drv_t::zero_force()
{
    motor_base->send_torque(0.0f);
}

float wheel_drv_t::get_target_speed()
{
    return _target_speed;
}

float *wheel_drv_t::get_p_target_speed()
{
    return & _target_speed;
}

float wheel_drv_t::get_current_speed()
{
    return _current_speed;
}

float *wheel_drv_t::get_p_current_speed()
{
    return &_current_speed;
}

void wheel_drv_t::update_feedback()
{
    motor_base->update_feedback();
    _current_speed = motor_base->get_current_rotate() / _gear_ratio * _radius;
}


}; // namespace pyro
