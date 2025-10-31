#include "pyro_yaw_drv.h"

namespace pyro
{
yaw_drv_t::yaw_drv_t(motor_base_t *motor_base,
                const pid_ctrl_t &rotate_pid,
                const pid_ctrl_t &position_pid)
    : motor_base(motor_base),
      _yaw_rotate_pid(rotate_pid),
      _yaw_position_pid(position_pid)
{
}

void yaw_drv_t::set_offset_radian(float offset_radian)
{
    _offset_radian = offset_radian;
}

void yaw_drv_t::set_radian(float target_radian)
{
    if(_current_radian < 0)
    {
        _current_radian += 2*PI;
    }

    if(target_radian < 0)
    {
        target_radian += 2*PI;
    }

    float radian_diff = target_radian - _current_radian;

    if(radian_diff > PI)
    {
        radian_diff -= 2 * PI;
    }
    else if (radian_diff < -PI)
    {
        radian_diff += 2 * PI;
    }

    float rotate_cmd =
        _yaw_position_pid.compute(_current_radian + radian_diff, _current_radian, 0.001f);
    float torque_cmd =
        _yaw_rotate_pid.compute(rotate_cmd, motor_base->get_current_rotate(), 0.001f);
    motor_base->send_torque(torque_cmd);
}

void yaw_drv_t::zero_force()
{
    motor_base->send_torque(0.0f);
}

void yaw_drv_t::update_feedback()
{
    motor_base->update_feedback();
    _current_radian = motor_base->get_current_position() - _offset_radian;
}

float yaw_drv_t::get_target_radian()
{
    return _target_radian;
}

}