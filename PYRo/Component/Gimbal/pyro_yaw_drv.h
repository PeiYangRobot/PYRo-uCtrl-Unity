#ifndef __PYRO_YAW_DRV_H__
#define __PYRO_YAW_DRV_H__

#include "pyro_motor_base.h"
#include "pyro_pid_ctrl.h"

namespace pyro
{
class yaw_drv_t
{
    public:
      yaw_drv_t(motor_base_t *motor_base,
                const pid_ctrl_t &rotate_pid,
                const pid_ctrl_t &position_pid);
      ~yaw_drv_t()
      {
      }

      void set_offset_radian(float offset_radian);
      void set_radian(float target_radian);
      void zero_force();
      void update_feedback();
      float get_target_radian();
      float _offset_radian;
      motor_base_t *motor_base;

    private:
      pid_ctrl_t _yaw_rotate_pid;
      pid_ctrl_t _yaw_position_pid;
      float _target_radian;
    float _current_radian;
};


}

#endif

