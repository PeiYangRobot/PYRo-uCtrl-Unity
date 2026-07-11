#ifndef PYRO_ALGO_COMMON_H
#define PYRO_ALGO_COMMON_H

#include <cstdint>

namespace pyro
{

float wrap2pi_f32(float input);
float radps_to_rpm(float radps);
float calculate_angle_diff(float current, float target);
float evaluate_polynomial(float x, const float *coeffs, uint32_t degree);
float mps_to_rpm(float mps, float radius);
float rpm_to_mps(float rpm, float radius);
float loop_fp32_constrain(float val, float min_val, float max_val);

} // namespace pyro

#endif // PYRO_ALGO_COMMON_H
