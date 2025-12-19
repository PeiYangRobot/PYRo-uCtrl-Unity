#ifndef __PYRO_SHOOT_17MM_CONTROL_H__
#define __PYRO_SHOOT_17MM_CONTROL_H__ 

#include "pyro_shoot_base.h"

namespace pyro
{
class shoot_17mm_control_t : public shoot_base_t
{
public:
    shoot_17mm_control_t(trigger_drv_t *trigger_drv,
                         fric_drv_t *fric_drv_1,
                         fric_drv_t *fric_drv_2
                        );
    ~shoot_17mm_control_t();

    void set_trigger_rotate(float target_rotate);
    void set_fric_speed(float target_speed) override;
    void update_feedback() override;
    void zero_force() override;
    void set_control();
    void control() override;

private:
    trigger_drv_t *_trigger_drv;
    float _first_radian{};
    float _trigger_rotate;
    float _fric_speed;
    fric_drv_t *_fric_drv[2];
};
    
}


#endif

