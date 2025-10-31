#ifndef __PYRO_CHASSIS_DRV_H__
#define __PYRO_CHASSIS_DRV_H__

#include "pyro_steering_wheel_drv.h"

namespace pyro
{
class chassis_drv_t
{
  public:
    chassis_drv_t(steering_wheel_drv_t *steering_wheel_drv_1,
                  steering_wheel_drv_t *steering_wheel_drv_2,
                  wheel_drv_t *wheel_drv_1,
                  wheel_drv_t *wheel_drv_2);
    ~chassis_drv_t()
    {
    }
    
    void dr16_cmd();
    void update_feedback();
    void zero_force();
    void chassis_control();

    wheel_drv_t *_wheel_drv_1;
    wheel_drv_t *_wheel_drv_2;
    steering_wheel_drv_t *_steering_wheel_drv_1;
    steering_wheel_drv_t *_steering_wheel_drv_2;

  private:
  float _vx, _vy, _wz; // The direction of the front of the vehicle represents the vy direction
  uint8_t _s_right;

};

}

#endif

