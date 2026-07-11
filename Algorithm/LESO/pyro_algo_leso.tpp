/**
 * @file pyro_algo_leso.tpp
 * @brief Template implementation file for the generic N-th order PYRO C++ LESO.
 * Do not include this file directly in your source code; include the .h file instead.
 */

#ifndef __PYRO_ALGO_LESO_H__
#error "pyro_algo_leso.tpp should only be included by pyro_algo_leso.h"
#endif

#include <algorithm> // For std::clamp
#include <cmath>     // For std::pow

namespace pyro
{

template <uint32_t Order>
leso_t<Order>::leso_t(const float omega_o, const float b, const float z_limit)
    : _omega_o(omega_o), _b(b), _z_limit(z_limit)
{
    _beta.fill(0.0f);
    _z.fill(0.0f);
    _next_z.fill(0.0f);
    calculate_betas();
}

template <uint32_t Order>
void leso_t<Order>::update(const float measure, const float u)
{
    _dt = dwt_drv_t::get_delta_t(&_dwt_cnt);

    if (_dt < 1e-9f)
    {
        return;
    }

    const float err = _z[0] - measure;

    // --- Generic N-th Order Forward Euler Integration ---
    for (uint32_t i = 0; i < Order - 1; ++i)
    {
        float b_u_term = (i == Order - 2) ? (_b * u) : 0.0f;
        _next_z[i] = _z[i] + _dt * (_z[i + 1] - _beta[i] * err + b_u_term);
    }

    // For the last state (disturbance): z_{n-1}
    _next_z[Order - 1] = _z[Order - 1] + _dt * (-_beta[Order - 1] * err);

    // Apply Disturbance Anti-windup
    if (_z_limit > 0.0f)
    {
        _next_z[Order - 1] = std::clamp(_next_z[Order - 1], -_z_limit, _z_limit);
    }

    // Update internal states
    _z = _next_z;
}

template <uint32_t Order>
void leso_t<Order>::clear()
{
    _z.fill(0.0f);
    _next_z.fill(0.0f);
    _dwt_cnt = 0;
    _dt      = 0.0f;
}

template <uint32_t Order>
void leso_t<Order>::set_params(const float omega_o, const float b)
{
    _omega_o = omega_o;
    _b       = b;
    calculate_betas();
}

template <uint32_t Order>
void leso_t<Order>::set_z_limit(const float limit)
{
    _z_limit = limit > 0.0f ? limit : 0.0f;
}

template <uint32_t Order>
float leso_t<Order>::get_z(const uint32_t index) const
{
    if (index < Order)
    {
        return _z[index];
    }
    return 0.0f;
}

template <uint32_t Order>
float leso_t<Order>::get_disturbance() const
{
    return _z[Order - 1];
}

template <uint32_t Order>
float leso_t<Order>::get_b() const
{
    return _b;
}

template <uint32_t Order>
constexpr uint32_t leso_t<Order>::get_order() const
{
    return Order;
}

template <uint32_t Order>
constexpr uint32_t leso_t<Order>::calculate_nCr(uint32_t n, uint32_t k)
{
    if (k > n) return 0;
    if (k * 2 > n) k = n - k;
    uint32_t res = 1;
    for (uint32_t i = 1; i <= k; ++i)
    {
        res = res * (n - i + 1) / i;
    }
    return res;
}

template <uint32_t Order>
void leso_t<Order>::calculate_betas()
{
    for (uint32_t i = 1; i <= Order; ++i)
    {
        uint32_t nCr = calculate_nCr(Order, i);
        _beta[i - 1] = static_cast<float>(nCr) * std::pow(_omega_o, static_cast<float>(i));
    }
}

} // namespace pyro