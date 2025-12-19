#ifndef __VELOCITY_CONTROLLER_H__
#define __VELOCITY_CONTROLLER_H__

#include "pyro_closed_controller.h"
#include "pyro_algo_pid.h"

namespace pyro
{

class velocity_controller_t : public closed_controller_t
{
    public:
        velocity_controller_t(motor_base_t* motor, pid_t* spd_pid);
        void set_target(float target) override;
        virtual void update() override;
        void control(float dt) override;
    protected:
        pid_t* _spd_pid;
        float _target_spd;
        float _feedback_spd;
        float _control_value;
};

};

#endif