#include "pyro_velocity_controller.h"

namespace pyro
{
    velocity_controller_t::velocity_controller_t(motor_base_t* motor, pid_ctrl_t* spd_pid)
        : closed_controller_t(motor), _spd_pid(spd_pid)
    {
        _target_spd = 0.0f;
        _feedback_spd = 0.0f;
        _control_value = 0.0f;
    }
    void velocity_controller_t::set_target(float target)
    {
        _target_spd = target;
    }

    void velocity_controller_t::update()
    {
        _motor->update_feedback();
        _feedback_spd = _motor->get_current_rotate();
    }

    void velocity_controller_t::control(float dt)
    {
        _control_value = _spd_pid->compute(_target_spd, _feedback_spd, dt);
        _motor->send_torque(_control_value);
    }

};