#include "pyro_algo_pd.h"

#include "pyro_core_def.h"
#include "pyro_dwt_drv.h"
#include <algorithm>
#include <cmath>

namespace pyro
{
namespace
{

float cutoff_to_rc(const float cutoff_hz)
{
    return cutoff_hz > 0.0f
               ? 1.0f / (2.0f * PI * cutoff_hz)
               : 0.0f;
}

uint8_t normalize_lpf_order(const uint8_t order)
{
    return order == 0 ? 0 : std::min(order, pd_ctrl_t::MAX_LPF_ORDER);
}

} // namespace

pd_ctrl_t::pd_ctrl_t(const float kp, const float kd, const float max_out)
    : pd_ctrl_t(kp, 0.0f, kd, 0.0f, max_out, 0.0f, 0, 0.0f, 0,
                NONE)
{
}

pd_ctrl_t::pd_ctrl_t(const float kp, const float kd, const float max_out,
                     const float deadband)
    : pd_ctrl_t(kp, kd, max_out)
{
    _deadband = std::fabs(deadband);
}

pd_ctrl_t::pd_ctrl_t(const float kp, const float ki, const float kd,
                     const float integral_limit, const float max_out,
                     const uint8_t improve)
    : pd_ctrl_t(kp, ki, kd, integral_limit, max_out, 0.0f, 0, 0.0f, 0,
                improve)
{
}

pd_ctrl_t::pd_ctrl_t(const float kp, const float kd, const float max_out,
                     const float output_cutoff_hz,
                     const uint8_t output_lpf_order,
                     const float derivative_cutoff_hz,
                     const uint8_t derivative_lpf_order,
                     const uint8_t improve)
    : pd_ctrl_t(kp, 0.0f, kd, 0.0f, max_out, output_cutoff_hz,
                output_lpf_order, derivative_cutoff_hz,
                derivative_lpf_order, improve)
{
}

pd_ctrl_t::pd_ctrl_t(const float kp, const float ki, const float kd,
                     const float integral_limit, const float max_out,
                     const float output_cutoff_hz,
                     const uint8_t output_lpf_order,
                     const float derivative_cutoff_hz,
                     const uint8_t derivative_lpf_order,
                     const uint8_t improve)
    : _kp(kp), _ki(ki), _kd(kd), _max_out(max_out),
      _integral_limit(std::fabs(integral_limit)), _deadband(0.0f),
      _output_lpf_rc(cutoff_to_rc(output_cutoff_hz)),
      _derivative_lpf_rc(cutoff_to_rc(derivative_cutoff_hz)),
      _output_lpf_order(normalize_lpf_order(output_lpf_order)),
      _derivative_lpf_order(normalize_lpf_order(derivative_lpf_order)),
      _improve(improve), _reference(0.0f), _measurement(0.0f),
      _error(0.0f), _last_error(0.0f), _measurement_speed(0.0f),
      _p_out(0.0f), _i_out(0.0f), _i_term(0.0f), _d_out(0.0f),
      _output(0.0f), _dt(0.0f), _dwt_cnt(0), _output_lpf_state{},
      _derivative_lpf_state{}
{
}

float pd_ctrl_t::calculate(const float reference, const float measurement,
                           const float measurement_speed)
{
    _reference         = reference;
    _measurement       = measurement;
    _error             = _reference - _measurement;
    _measurement_speed = measurement_speed;

    const uint8_t timed_features =
        INTEGRAL | OUTPUT_FILTER | DERIVATIVE_FILTER;
    if ((_improve & timed_features) != 0)
    {
        calculate_dt();
        if (_dt < 1.0e-9f)
        {
            return _output;
        }
    }
    else
    {
        _dt = 0.0f;
    }

    const bool outside_deadband = std::fabs(_error) > _deadband;
    _p_out = outside_deadband ? _kp * _error : 0.0f;
    _d_out = -_kd * _measurement_speed;

    if ((_improve & INTEGRAL) != 0 && outside_deadband)
    {
        integrate();
        if ((_improve & INTEGRAL_LIMIT) != 0)
        {
            limit_integral();
        }
        _i_out += _i_term;
    }
    else
    {
        _i_term = 0.0f;
    }

    if ((_improve & DERIVATIVE_FILTER) != 0)
    {
        filter_derivative();
    }

    _output = _p_out + _i_out + _d_out;

    if ((_improve & OUTPUT_FILTER) != 0)
    {
        filter_output();
    }

    limit_output();

    _last_error = _error;
    return _output;
}

void pd_ctrl_t::clear()
{
    _reference         = 0.0f;
    _measurement       = 0.0f;
    _error             = 0.0f;
    _last_error        = 0.0f;
    _measurement_speed = 0.0f;
    _p_out             = 0.0f;
    _i_out             = 0.0f;
    _i_term            = 0.0f;
    _d_out             = 0.0f;
    _output            = 0.0f;
    _dt                = 0.0f;
    _dwt_cnt           = 0;

    std::fill_n(_output_lpf_state, MAX_LPF_ORDER, 0.0f);
    std::fill_n(_derivative_lpf_state, MAX_LPF_ORDER, 0.0f);
}

void pd_ctrl_t::set_gains(const float kp, const float kd)
{
    _kp = kp;
    _kd = kd;
}

void pd_ctrl_t::set_gains(const float kp, const float ki, const float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void pd_ctrl_t::set_max_out(const float max_out)
{
    _max_out = max_out;
}

void pd_ctrl_t::set_deadband(const float deadband)
{
    _deadband = std::fabs(deadband);
}

void pd_ctrl_t::set_improve(const uint8_t improve)
{
    _improve = improve;
}

void pd_ctrl_t::calculate_dt()
{
    _dt = dwt_drv_t::get_delta_t(&_dwt_cnt);
}

void pd_ctrl_t::integrate()
{
    if ((_improve & TRAPEZOID_INTEGRAL) != 0)
    {
        _i_term = _ki * ((_error + _last_error) * 0.5f) * _dt;
    }
    else
    {
        _i_term = _ki * _error * _dt;
    }
}

void pd_ctrl_t::limit_integral()
{
    const float proposed_i_out = _i_out + _i_term;
    const float proposed_output = _p_out + proposed_i_out + _d_out;

    if (_max_out > 0.0f && std::fabs(proposed_output) > _max_out &&
        _error * _i_out > 0.0f)
    {
        _i_term = 0.0f;
    }

    const float limited_i_out = _i_out + _i_term;
    if (limited_i_out > _integral_limit)
    {
        _i_term = _integral_limit - _i_out;
    }
    else if (limited_i_out < -_integral_limit)
    {
        _i_term = -_integral_limit - _i_out;
    }
}

void pd_ctrl_t::filter_derivative()
{
    if (_derivative_lpf_rc <= 0.0f || _derivative_lpf_order == 0)
    {
        return;
    }

    const float alpha = _dt / (_derivative_lpf_rc + _dt);
    float input       = _d_out;
    for (uint8_t i = 0; i < _derivative_lpf_order; ++i)
    {
        _derivative_lpf_state[i] =
            input * alpha + _derivative_lpf_state[i] * (1.0f - alpha);
        input = _derivative_lpf_state[i];
    }
    _d_out = input;
}

void pd_ctrl_t::filter_output()
{
    if (_output_lpf_rc <= 0.0f || _output_lpf_order == 0)
    {
        return;
    }

    const float alpha = _dt / (_output_lpf_rc + _dt);
    float input       = _output;
    for (uint8_t i = 0; i < _output_lpf_order; ++i)
    {
        _output_lpf_state[i] =
            input * alpha + _output_lpf_state[i] * (1.0f - alpha);
        input = _output_lpf_state[i];
    }
    _output = input;
}

void pd_ctrl_t::limit_output()
{
    if (_max_out > 0.0f)
    {
        _output = std::clamp(_output, -_max_out, _max_out);
    }

    if (_output_lpf_order > 0)
    {
        _output_lpf_state[_output_lpf_order - 1] = _output;
    }
}

} // namespace pyro
