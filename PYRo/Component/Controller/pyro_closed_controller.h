#ifndef __CLOSED_CONTROLLER_H__
#define __CLOSED_CONTROLLER_H__

#include "pyro_motor_base.h"


namespace pyro
{
    class closed_controller_t
    {
        public:
            closed_controller_t(motor_base_t *motor): _motor(motor){};
            virtual void set_target(float target) = 0 ;
            virtual void update() = 0 ;
            virtual void control(float dt) = 0 ;
        protected:
            motor_base_t *_motor;
    };
};

#endif