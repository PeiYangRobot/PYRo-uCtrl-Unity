#include "pyro_wheel_drv.h"

namespace pyro
{

pyro_wheel_drv_t::pyro_wheel_drv_t(motor_base_t *motor_base, rc_drv_t *rc_drv,
                                   const pid_ctrl_t &speed_pid, float radius)
    : motor_base(motor_base),
      _speed_pid(speed_pid),
      _radius(radius)
      
{
    rc_drv->config_rc_cmd([this](rc_drv_t *rc_drv) -> void { get_mode(rc_drv); });  
};

void pyro_wheel_drv_t::set_gear_ratio(float gear_ratio)
{
    _gear_ratio = gear_ratio;
}

void pyro_wheel_drv_t::set_speed(float target_speed)
{
    float current_mps = _rpm_to_mps( motor_base->get_current_rotate() );
    float torque_cmd = _speed_pid.compute(target_speed, current_mps, 0.001f);
    motor_base->send_torque(torque_cmd);
}

void pyro_wheel_drv_t::get_mode(rc_drv_t *rc_drv)
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            rc_drv->get_p_ctrl());
    _target_speed = static_cast<float>(p_ctrl->rc.ch[0]) / 660.0f * 10.0f * 2.0f * 3.14159f * _radius; 
}

float pyro_wheel_drv_t::get_target_speed()
{
    return _target_speed;
}

float pyro_wheel_drv_t::get_current_speed()
{
    return _rpm_to_mps( motor_base->get_current_rotate() );
}

float pyro_wheel_drv_t::_rpm_to_mps(float rpm)
{
    float wheel_rpm = rpm / _gear_ratio;
    return 2.0f * 3.14159f * _radius * (wheel_rpm / 60.0f);
}

}; // namespace pyro
