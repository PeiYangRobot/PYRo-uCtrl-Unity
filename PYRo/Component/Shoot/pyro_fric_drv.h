#ifndef __PYRO_FRIC_H__
#define __PYRO_FRIC_H__

#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_vofa.h"

namespace pyro
{
class fric_drv_t
{
public:
    enum rotate_direction_t
    {
        CLOCKWISE          = 0x00,
        COUNTERCLOCKWISE   = 0x01
    };

    fric_drv_t(motor_base_t *motor_base,
               const pid_t &speed_pid,
               float radius,
               rotate_direction_t direction
            );
    ~fric_drv_t()
    {
    }

    void set_dt(float dt);
    void set_speed(float target_speed);
    void zero_force();
    float get_speed();
    float get_target_speed();
    void update_feedback();
    void control();

private:
    motor_base_t *_motor_base;
    pid_t _speed_pid;
    float _radius;
    rotate_direction_t _direction;
    float _dt = 0.001f;
    float _target_speed;
    float _current_speed;

friend class vofa_drv_t;
};


}
#endif

