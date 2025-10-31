#include "pyro_steering_wheel_drv.h"

namespace pyro
{

steering_wheel_drv_t::steering_wheel_drv_t(wheel_drv_t *wheel_drv,
                                           motor_base_t *rudder_motor_base,
                                           const pid_ctrl_t &rudder_rotate_pid,
                                           const pid_ctrl_t &rudder_position_pid)
    : wheel_drv(wheel_drv),
      rudder_motor_base(rudder_motor_base),
      _rudder_rotate_pid(rudder_rotate_pid),
      _rudder_position_pid(rudder_position_pid)
{
}

void steering_wheel_drv_t::set_offset_radian(float offset_radian)
{
    _offset_radian = offset_radian;
}

void steering_wheel_drv_t::set_radian(float target_radian)
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
        _rudder_position_pid.compute(_current_radian + radian_diff, _current_radian, 0.001f);
    float torque_cmd =
        _rudder_rotate_pid.compute(rotate_cmd, rudder_motor_base->get_current_rotate(), 0.001f);
    rudder_motor_base->send_torque(torque_cmd);
}

void steering_wheel_drv_t::zero_force()
{
    rudder_motor_base->send_torque(0.0f);
    wheel_drv->motor_base->send_torque(0.0f);
}

void steering_wheel_drv_t::update_feedback()
{
    wheel_drv->update_feedback();
    rudder_motor_base->update_feedback();
    _current_radian = rudder_motor_base->get_current_position() - _offset_radian;

}

float steering_wheel_drv_t::get_target_radian()
{
    return _target_radian;
}

}
