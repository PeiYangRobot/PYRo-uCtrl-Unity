#include "pyro_pid_ctrl.h"

namespace pyro
{

    static float constraint(float value, float max)
    {
        if(value > max)
            return max;
        if(value < -max)
            return -max;
        return value;
    }
    pid_ctrl_t::pid_ctrl_t()
    {
        reset();
    }

    pid_ctrl_t::pid_ctrl_t(float kp, float ki, float kd)
    :   _kp(kp), _ki(ki), _kd(kd), _dt(0.0f), _reference(0.0f), _feedback(0.0f),
        _error(0.0f), _error_last(0.0f), _integral(0.0f), _derivative(0.0f),
        _pout(0.0f), _iout(0.0f), _dout(0.0f), _output(0.0f)
    {}

    pid_ctrl_t::~pid_ctrl_t(){}

    float pid_ctrl_t::compute(float reference, float feedback, float dt)
    {
        _dt = dt;
        _reference = reference;
        _feedback = feedback;
        _error = _reference - _feedback;
        _integral += _error * _dt;
        _integral = constraint(_integral, _integral_max);
        _derivative = (_error - _error_last) / _dt;
        _pout = _kp * _error;
        _iout = _ki * _integral;
        _dout = _kd * _derivative;
        _output = _pout + _iout + _dout;
        _output = constraint(_output, _output_max);
        _error_last = _error;
        return _output;
    }

    void pid_ctrl_t::reset()
    {
        _dt = 0.0f;
        _reference = 0.0f;
        _feedback = 0.0f;
        _error = 0.0f;
        _error_last = 0.0f;
        _integral = 0.0f;
        _derivative = 0.0f;
        _pout = 0.0f;
    }

    status_t pid_ctrl_t::set_kp(float kp)
    {
        _kp = kp;
        if(_kp < 0.0f)
            return PYRO_WARNING;
        else
            return PYRO_OK;
    }

    status_t pid_ctrl_t::set_ki(float ki)
    { 
        _ki = ki;
        if(_ki < 0.0f)
            return PYRO_WARNING;
        else
            return PYRO_OK;
    }

    status_t pid_ctrl_t::set_kd(float kd)
    {
        _kd = kd;
        if(_kd < 0.0f)
            return PYRO_WARNING;
        else
            return PYRO_OK;
    }
    float pid_ctrl_t::get_kp() const
    {
        return _kp;
    }

    float pid_ctrl_t::get_ki() const
    {
        return _ki;
    }

    float pid_ctrl_t::get_kd() const
    {
        return _kd;
    }

    status_t pid_ctrl_t::set_output_limits(float max)
    {
        _output_max = max;
        if(_output_max < 0.0f)
            return PYRO_ERROR;
        else
            return PYRO_OK;
    }
    status_t pid_ctrl_t::set_integral_limits(float max)
    {
        _integral_max = max;
        if(_integral_max < 0.0f)
            return PYRO_ERROR;
        else
            return PYRO_OK;
    }
};