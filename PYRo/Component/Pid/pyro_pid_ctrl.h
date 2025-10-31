#ifndef __PYRO_PID_CTRL_H__
#define __PYRO_PID_CTRL_H__

#include <stdint.h>

#include "pyro_core_def.h"
namespace pyro
{
class pid_ctrl_t // classic pid
{
    public:
        pid_ctrl_t();
        pid_ctrl_t(float kp, float ki, float kd);
        ~pid_ctrl_t();

        float compute(float reference, float feedback, float dt);

        void reset();
        status_t set_kp(float kp);
        status_t set_ki(float ki);
        status_t set_kd(float kd);

        float get_kp() const;
        float get_ki() const;
        float get_kd() const;

        status_t set_output_limits(float max);
        status_t set_integral_limits(float max);
    private:
        uint32_t _time_stamp;

        float _dt;
        float _kp, _ki, _kd;

        float _reference,_feedback;
        float _error, _error_last;

        float _integral;
        float _integral_max;
        float _derivative;

        float _pout, _iout, _dout;
        float _output;
        float _output_max;
};

};


#endif