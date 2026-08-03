#include "pyro_algo_common.h"

#include "arm_math.h"
#include <cmath>

namespace pyro
{

float wrap2pi_f32(const float input)
{
    return std::fmod(input, 2.0f * PI);
}

// Legacy-compatible angle normalization used by the wheel-legged control path.
// Keep wrap2pi_f32() unchanged for existing users of the migrated common layer.
float wrap2pi_f32_normalized(const float input)
{
    float ret = std::fmod(input, 2.0f * PI);
    if (ret >= PI)
    {
        ret -= 2.0f * PI;
    }
    else if (ret < -PI)
    {
        ret += 2.0f * PI;
    }
    return ret;
}

float evaluate_polynomial_ascending(const float x, const float *coeffs,
                                    const uint32_t degree)
{
    // Coefficients are ordered as c0 + c1*x + ... + cn*x^n.
    float result = coeffs[degree];
    for (uint32_t i = degree; i > 0; --i)
    {
        result = result * x + coeffs[i - 1];
    }
    return result;
}

float radps_to_rpm(const float radps)
{
    return radps * 9.5492966f;
}

float calculate_angle_diff(const float current, const float target)
{
    const float diff = std::fabs(current - target);
    return std::fmin(diff, 2.0f * PI - diff);
}

float evaluate_polynomial(const float x, const float *coeffs,
                          const uint32_t degree)
{
    float result = coeffs[0];
    for (uint32_t i = 1; i <= degree; ++i)
    {
        result = result * x + coeffs[i];
    }
    return result;
}

float mps_to_rpm(const float mps, const float radius)
{
    if (radius < 1.0e-4f)
    {
        return 0.0f;
    }
    return (mps / radius) * 9.5492966f;
}

float rpm_to_mps(const float rpm, const float radius)
{
    return rpm * radius / 9.5492966f;
}

float loop_fp32_constrain(float val, const float min_val, const float max_val)
{
    const float len = max_val - min_val;
    if (len < 1.0e-6f)
    {
        return val;
    }
    while (val > max_val)
    {
        val -= len;
    }
    while (val < min_val)
    {
        val += len;
    }
    return val;
}

float fp32_constrain(float val, const float min_val, const float max_val)
{
    if (val > max_val)
        return max_val;
    if (val < min_val)
        return min_val;
    return val;

}

} // namespace pyro
