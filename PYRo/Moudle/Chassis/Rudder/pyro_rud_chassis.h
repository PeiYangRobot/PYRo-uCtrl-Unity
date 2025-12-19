#ifndef __PYRO_RUD_CHASSIS_H__
#define __PYRO_RUD_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_chassis_base.h"
#include "pyro_kin_rudder.h"
#include "pyro_motor_base.h"

namespace pyro
{
class rud_chassis_t final : public chassis_base_t
{
  public:
    struct cmd_rud_t final : cmd_base_t
    {
        cmd_rud_t() : cmd_base_t(type_t::RUDDER)
        {
        }
        // Additional mecanum-specific command parameters can be added here
    };
    rud_chassis_t() : chassis_base_t(type_t::RUDDER)
    {
    }
    ~rud_chassis_t() override;

    void init() override;
    void set_command(const cmd_base_t &cmd) override;
    void update_feedback() override;
    void kinematics_solve() override;
    void chassis_control() override;
    void power_control() override;
    void send_motor_command() override;

  private:
    cmd_rud_t _cmd_rud;
    rudder_kin_t::rudder_states_t _target_states{};
    rudder_kin_t::rudder_states_t _current_states{};
    rudder_kin_t *_kinematics{};
    float _wheel_output[4]{};         // Wheel torque command output
    float _rudder_target_speed[4]{};  // Rudder speed target for angle control
    float _rudder_current_speed[4]{}; // Rudder current speed feedback
    float _rudder_output[4]{};        // Rudder torque command output

    // Rudder angle offset calibration
    float _rudder_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    motor_base_t *_wheel_motor[4]{};  // FL, FR, BL, BR
    motor_base_t *_rudder_motor[4]{}; // Rudder motor

    pid_t *_wheel_speed_pid[4]{};  // Wheel Speed PID controller
    pid_t *_rudder_angle_pid[4]{}; // Rudder angle PID controller
    pid_t *_rudder_speed_pid[4]{}; // Rudder speed PID controller
    pid_t *_follow_angle_pid{};    // Chassis follow angle PID
};
} // namespace pyro
#endif
