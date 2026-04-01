/**
 * @file pyro_algo_ols.cpp
 * @brief Implementation file for the PYRO C++ OLS class.
 *
 * This file implements the `pyro::ols_t` methods.
 *
 * @author Wang Hongxi (Original C)
 * @author Lucky (C++ Refactor)
 * @version 1.0.0
 * @date 2025-11-13
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_algo_ols.h"
#include <cmath> // For fabsf

namespace pyro
{

/**
 * @brief Constructor, replaces OLS_Init
 */
ols_t::ols_t(uint16_t order)
    : _order(order), _count(0), _k(0.0f), _b(0.0f), _deviation(0.0f)
{
    // C++ way: use vector::resize for automatic memory management.
    // We need at least 2 points for linear regression.
    if (_order < 2)
    {
        _order = 2;
    }

    _x.resize(_order, 0.0f);
    _y.resize(_order, 0.0f);
}

/**
 * @brief Update function, combines OLS_Update, OLS_Derivative, OLS_Smooth
 */
void ols_t::update(float deltax, float y)
{
    // 1. Shift the data window (sliding window)
    const float temp = _x[1];
    for (uint16_t i = 0; i < _order - 1; ++i)
    {
        _x[i] = _x[i + 1] - temp;
        _y[i] = _y[i + 1];
    }
    // Add new data
    _x[_order - 1] = _x[_order - 2] + deltax;
    _y[_order - 1] = y;

    if (_count < _order)
    {
        _count++;
    }

    // 2. Recalculate statistical sums
    float t[4]                 = {0.0f, 0.0f, 0.0f, 0.0f};

    // Loop start index is consistent with C code
    const uint16_t start_index = _order - _count;
    for (uint16_t i = start_index; i < _order; ++i)
    {
        t[0] += _x[i] * _x[i]; // sum(x^2)
        t[1] += _x[i];         // sum(x)
        t[2] += _x[i] * _y[i]; // sum(xy)
        t[3] += _y[i];         // sum(y)
    }

    // 3. Calculate k (slope) and b (intercept)
    const float denominator = (t[0] * static_cast<float>(_order) - t[1] * t[1]);

    // **Safety Check**: Missing divide-by-zero protection in C code
    if (std::fabs(denominator) > 1e-9f)
    {
        _k = (t[2] * static_cast<float>(_order) - t[1] * t[3]) / denominator;
        _b = (t[0] * t[3] - t[1] * t[2]) / denominator;
    }
    else
    {
        // Denominator is zero (all x values are the same), cannot calc slope
        _k = 0.0f;
        // If k=0, b should be the average of y
        _b = (_count > 0) ? (t[3] / static_cast<float>(_count)) : 0.0f;
    }

    // 4. Calculate deviation
    _deviation = 0.0f;
    for (uint16_t i = start_index; i < _order; ++i)
    {
        _deviation += std::fabs(_k * _x[i] + _b - _y[i]);
    }

    // Replicate C code logic (divide by Order, not Count)
    _deviation /= static_cast<float>(_order);
}

/**
 * @brief Getter: Get derivative (k)
 */
float ols_t::get_derivative() const
{
    return _k;
}

/**
 * @brief Getter: Get smoothed value (y = k*x + b)
 */
float ols_t::get_smooth() const
{
    // Use .back() to get the last element of std::vector
    return _k * _x.back() + _b;
}

/**
 * @brief Getter: Get Mean Absolute Deviation
 */
float ols_t::get_mean_absolute_deviation() const
{
    return _deviation;
}

} // namespace pyro