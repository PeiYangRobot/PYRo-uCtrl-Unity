#ifndef __PYRO_STEERING_WHEEL_DRV_H__
#define __PYRO_STEERING_WHEEL_DRV_H__

#include "pyro_wheel_drv.h"

namespace pyro
{
class steering_wheel_drv_t
{
  public:
    steering_wheel_drv_t(wheel_drv_t *wheel_drv,   
                         motor_base_t *rudder_motor_base,
                         const pid_ctrl_t &rudder_rotate_pid,
                         const pid_ctrl_t &rudder_position_pid);
    ~steering_wheel_drv_t()
    {
    }

    void set_offset_radian(float offset_radian);
    void set_radian(float target_radian);
    void zero_force();
    void update_feedback();
    float get_target_radian();
    float _offset_radian;
    wheel_drv_t *wheel_drv;
    motor_base_t *rudder_motor_base;

  private:
    pid_ctrl_t _rudder_rotate_pid;
    pid_ctrl_t _rudder_position_pid;
    float _target_radian;
    float _current_radian;
    
};

}

#endif

