#ifndef __PYRO_WHEEL_DRV_H__
#define __PYRO_WHEEL_DRV_H__

#include "pyro_dji_motor_drv.h"
#include "pyro_pid_ctrl.h"

namespace pyro
{
class wheel_drv_t
{
  public:
    wheel_drv_t(motor_base_t *motor_base,
                     const pid_ctrl_t &speed_pid, float radius);
    wheel_drv_t()
    {
    }

    void set_gear_ratio(float gear_ratio);
    void set_speed(float target_speed);
    void zero_force();
    float get_target_speed();
    float *get_p_target_speed();
    float get_current_speed();
    float *get_p_current_speed();
    void update_feedback();

    motor_base_t *motor_base;

  private:
    pid_ctrl_t _speed_pid;
    float _radius;
    float _target_speed;
    float _current_speed;
    float _gear_ratio;

};
};

#endif

