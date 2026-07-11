/**
 * @file pyro_algo_pid.cpp
 * @brief Implementation file for the PYRO C++ PID Controller class.
 *
 * This file implements the `pyro::pid_t` methods, including constructors
 * and the main PID calculation logic, integrating N-order cascaded LPFs.
 *
 * @author Wang Hongxi (Original C), Lucky (C++ Refactor)
 * @version 1.1.5
 * @date 2025-11-16
 * @copyright [Copyright Information Here]
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_algo_pid.h"
#include "pyro_core_def.h"
#include "pyro_dwt_drv.h" // For pyro::dwt_drv_t
#include <cmath>          // For std::fabs
#include <algorithm>      // For std::clamp

namespace pyro
{
/* Constructor Implementation ------------------------------------------------*/

/**
 * @brief Constructor 1: Basic PID (delegates to Constructor 3).
 */
pid_t::pid_t(const float kp, const float ki, const float kd,
             const float integral_limit, const float max_out,
             const uint8_t improve)
    : pid_t(max_out,        // max_out
            integral_limit, // integral_limit
            0.0f,           // deadband
            kp, ki, kd,     // Kp, Ki, Kd
            0.0f, 0.0f,     // A, B (ChangingIntegrationRate)
            0.0f, 1,        // output_cutoff_hz, output_lpf_order
            0.0f, 1,        // derivative_cutoff_hz, derivative_lpf_order
            0,              // ols_order
            improve)        // improve
{
    // All work done by delegation
}

/**
 * @brief Constructor 2: PID with filters/OLS (delegates to Constructor 3).
 */
pid_t::pid_t(const float kp, const float ki, const float kd,
             const float integral_limit, const float max_out,
             const float output_cutoff_hz, const uint8_t output_lpf_order,
             const float derivative_cutoff_hz, const uint8_t derivative_lpf_order,
             const uint16_t ols_order,
             const uint8_t improve)
    : pid_t(max_out,               // max_out
            integral_limit,        // integral_limit
            0.0f,                  // deadband
            kp, ki, kd,            // Kp, Ki, Kd
            0.0f, 0.0f,            // A, B (ChangingIntegrationRate)
            output_cutoff_hz,      // output_cutoff_hz
            output_lpf_order,      // output_lpf_order
            derivative_cutoff_hz,  // derivative_cutoff_hz
            derivative_lpf_order,  // derivative_lpf_order
            ols_order,             // ols_order
            improve)               // improve
{
    // All work done by delegation
}

/**
 * @brief Constructor 3: Full-featured (Main Constructor).
 */
pid_t::pid_t(const float max_out, const float integral_limit,
             const float deadband, const float kp, const float ki,
             const float kd, const float A, const float B,
             const float output_cutoff_hz, const uint8_t output_lpf_order,
             const float derivative_cutoff_hz, const uint8_t derivative_lpf_order,
             const uint16_t ols_order,
             const uint8_t improve)
    : // --- C++ Member Initializer List ---
      _kp(kp), _ki(ki), _kd(kd), _max_out(max_out),
      _integral_limit(integral_limit), _deadband(deadband), _coef_a(A),
      _coef_b(B),
      // --- Calculate RC constant from Hz ---
      // If cutoff_hz <= 0, set RC to 0.0f (disabling the filter)
      _output_lpf_rc((output_cutoff_hz > 0.0f)
                         ? (1.0f / (2.0f * PI * output_cutoff_hz))
                         : 0.0f),
      _derivative_lpf_rc((derivative_cutoff_hz > 0.0f)
                             ? (1.0f / (2.0f * PI * derivative_cutoff_hz))
                             : 0.0f),
      // Ensure filter orders are constrained to valid limits [1, MAX_LPF_ORDER]
      _output_lpf_order(std::clamp(output_lpf_order, (uint8_t)1, MAX_LPF_ORDER)),
      _derivative_lpf_order(std::clamp(derivative_lpf_order, (uint8_t)1, MAX_LPF_ORDER)),
      _ols_order(ols_order),
      _improve(improve), _ols(ols_order) // OLS instance is constructed here
{
    // Constructor body
    clear(); // Initialize all state variables
    _error_handler = {0, error_type_t::NONE};
}

/* Public Methods ------------------------------------------------------------*/

/**
 * @brief Calculates the PID output.
 */
float pid_t::calculate(const float ref, const float measure)
{
    if (_improve & improvement_t::ERROR_HANDLE)
    {
        handle_error();
    }

    // Get time delta from DWT
    _dt = dwt_drv_t::get_delta_t(&_dwt_cnt);

    // Prevent division by zero if dt is too small or 0
    if (_dt < 1e-9f)
    {
        _last_measure = measure;
        _last_err     = _err;
        return _output; // Keep last output
    }

    _measure = measure;
    _ref     = ref;
    _err     = _ref - _measure; // Standard PID error

    // Call user function 1 (if registered)
    if (_user_func1)
    {
        _user_func1(this);
    }

    // Only calculate PID if error is outside the deadband
    if (std::fabs(_err) > _deadband)
    {
        // --- Calculate P, I ---
        _p_out  = _kp * _err;
        _i_term = _ki * _err * _dt;

        // --- D Term Calculation (FIXED OLS LOGIC) ---
        // Ensure OLS is updated only once, with the correct source
        if (_improve & improvement_t::DERIVATIVE_ON_MEASUREMENT)
        {
            // D-on-M (Derivative on Measurement)
            if (_ols_order > 2)
            {
                _ols.update(_dt, -_measure); // 1. Only update OLS with -Measure
                _d_out = _kd * _ols.get_derivative();
            }
            else
            {
                _d_out = _kd * (_last_measure - _measure) / _dt;
            }
        }
        else
        {
            // D-on-Error (Standard Derivative)
            if (_ols_order > 2)
            {
                _ols.update(_dt, _err); // 1. Only update OLS with Error
                _d_out = _kd * _ols.get_derivative();
            }
            else
            {
                _d_out = _kd * (_err - _last_err) / _dt;
            }
        }
        // --- End of D Term Calculation ---

        // Call user function 2 (if registered)
        if (_user_func2)
        {
            _user_func2(this);
        }

        // --- Apply PID Improvements ---
        if (_improve & improvement_t::TRAPEZOID_INTEGRAL)
        {
            trapezoid_integral();
        }
        if (_improve & improvement_t::CHANGING_INTEGRATION_RATE)
        {
            changing_integration_rate();
        }
        if (_improve & improvement_t::DERIVATIVE_FILTER)
        {
            filter_derivative();
        }
        // Integral limit must be applied before I-term accumulation
        if (_improve & improvement_t::INTEGRAL_LIMIT)
        {
            limit_integral();
        }

        // Accumulate Integral
        _i_out += _i_term;

        // --- Calculate Total Output ---
        _output = _p_out + _i_out + _d_out;

        if (_improve & improvement_t::OUTPUT_FILTER)
        {
            filter_output();
        }

        limit_output(); // Apply final output limit

        limit_proportion(); // (Original C code logic)
    }

    // --- Update 'Last' States ---
    _last_measure = _measure;
    _last_err     = _err;
    _last_i_term  = _i_term;

    return _output;
}

/**
 * @brief Clears the internal PID state.
 */
void pid_t::clear()
{
    _ref          = 0.0f;
    _measure      = 0.0f;
    _err          = 0.0f;
    _last_err     = 0.0f;
    _p_out        = 0.0f;
    _i_out        = 0.0f;
    _d_out        = 0.0f;
    _i_term       = 0.0f;
    _last_i_term  = 0.0f;
    _output       = 0.0f;
    _last_measure = 0.0f;
    _dwt_cnt      = 0; // Reset DWT counter
    _dt           = 0.0f;

    // Clear filter arrays
    for (uint8_t i = 0; i < MAX_LPF_ORDER; ++i)
    {
        _out_lpf_state[i] = 0.0f;
        _d_lpf_state[i]   = 0.0f;
    }

    // Note: _ols is not cleared, it contains the history
}

/**
 * @brief Sets new PID gains.
 */
void pid_t::set_gains(const float kp, const float ki, const float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

/**
 * @brief Registers User Function 1.
 */
void pid_t::set_user_func1(const user_func_t func)
{
    _user_func1 = func;
}

/**
 * @brief Registers User Function 2.
 */
void pid_t::set_user_func2(const user_func_t func)
{
    _user_func2 = func;
}

/* Private Helper Functions --------------------------------------------------*/

/**
 * @brief Applies trapezoidal integration for the I-term.
 */
void pid_t::trapezoid_integral()
{
    _i_term = _ki * ((_err + _last_err) / 2.0f) * _dt;
}

/**
 * @brief Applies changing integration rate.
 */
void pid_t::changing_integration_rate()
{
    if (_err * _i_out > 0) // Integral is accumulating
    {
        if (std::fabs(_err) <= _coef_b)
        {
            return; // Full integration speed
        }
        if (std::fabs(_err) <= (_coef_a + _coef_b))
        {
            _i_term *= (_coef_a - std::fabs(_err) + _coef_b) / _coef_a;
        }
        else
        {
            _i_term = 0.0f; // Stop integration
        }
    }
}

/**
 * @brief Applies integral limiting and anti-windup.
 */
void pid_t::limit_integral()
{
    const float temp_Iout = _i_out + _i_term;
    const float temp_Output =
        _p_out + temp_Iout + _d_out; // Pre-calculated output

    // Anti-Windup: Stop integrating if output is saturated
    if (std::fabs(temp_Output) > _max_out)
    {
        if (_err * _i_out > 0) // Check if integral is still accumulating
        {
            _i_term = 0.0f; // Stop integration
        }
    }

    // Clamp I-Term: Hard limit on the integral value
    if (temp_Iout > _integral_limit)
    {
        _i_term = 0.0f;
        _i_out  = _integral_limit;
    }
    else if (temp_Iout < -_integral_limit)
    {
        _i_term = 0.0f;
        _i_out  = -_integral_limit;
    }
}

/**
 * @brief Applies an N-order cascaded low-pass filter to the derivative term.
 */
void pid_t::filter_derivative()
{
    if (_derivative_lpf_rc > 0.0f && _derivative_lpf_order > 0)
    {
        float alpha = _dt / (_derivative_lpf_rc + _dt);
        float input = _d_out;

        for (uint8_t i = 0; i < _derivative_lpf_order; ++i)
        {
            _d_lpf_state[i] = input * alpha + _d_lpf_state[i] * (1.0f - alpha);
            input = _d_lpf_state[i];
        }
        _d_out = input;
    }
}

/**
 * @brief Applies an N-order cascaded low-pass filter to the final output.
 */
void pid_t::filter_output()
{
    if (_output_lpf_rc > 0.0f && _output_lpf_order > 0)
    {
        float alpha = _dt / (_output_lpf_rc + _dt);
        float input = _output;

        for (uint8_t i = 0; i < _output_lpf_order; ++i)
        {
            _out_lpf_state[i] = input * alpha + _out_lpf_state[i] * (1.0f - alpha);
            input = _out_lpf_state[i];
        }
        _output = input;
    }
}

/**
 * @brief Clamps the final output to [-max_out, max_out] and prevents filter windup.
 */
void pid_t::limit_output()
{
    if (_output > _max_out)
    {
        _output = _max_out;
    }
    else if (_output < -_max_out)
    {
        _output = -_max_out;
    }

    // Anti-windup for the cascaded output filter
    if (_output_lpf_order > 0)
    {
        _out_lpf_state[_output_lpf_order - 1] = _output;
    }
}

/**
 * @brief Clamps the proportional term (legacy from C code).
 */
void pid_t::limit_proportion()
{
    if (_p_out > _max_out)
    {
        _p_out = _max_out;
    }
    else if (_p_out < -_max_out)
    {
        _p_out = -_max_out;
    }
}

/**
 * @brief Handles error conditions, e.g., motor blocked detection.
 */
void pid_t::handle_error()
{
    if (_output < _max_out * 0.001f || std::fabs(_ref) < 0.0001f)
    {
        return;
    }

    // Check for division-by-zero risk
    const float ref_abs = std::fabs(_ref);
    if (ref_abs > 1e-6f) // Avoid division by near-zero ref
    {
        if ((std::fabs(_ref - _measure) / ref_abs) > 0.95f)
        {
            _error_handler.error_count++;
        }
        else
        {
            _error_handler.error_count = 0;
        }
    }
    else
    {
        _error_handler.error_count = 0; // Ref is near zero, not stalled
    }

    if (_error_handler.error_count > 500)
    {
        _error_handler.error_type = error_type_t::MOTOR_BLOCKED;
    }
}

} // namespace pyro