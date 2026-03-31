#include "pyro_kin_rudder.h"
#include <arm_math.h>

namespace pyro
{

rudder_kin_t::rudder_kin_t(const float wheelbase, const float track_width)
{
    // Calculate geometry half-lengths for vector analysis
    _half_wheelbase   = abs(wheelbase) / 2.0f;
    _half_track_width = abs(track_width) / 2.0f;
}

void rudder_kin_t::calc_angle(float &target_angle, float current_angle,
                              int &direction)
{
    // 1. 修正目标角：带偏移并规约到 (-π, π]
    target_angle           = _normalize_angle(target_angle);

    // 2. 计算两种等价目标（不在这里wrap，以免跳变）
    const float target_a   = target_angle;      // 正向
    const float target_b   = target_angle + PI; // 反向候选（暂不规约）

    // 3. 确保两者相对于当前角的差是连续的
    const float delta_a    = _normalize_angle(target_a - current_angle);
    const float delta_b    = _normalize_angle(target_b - current_angle);

    // 4. 比较两者的绝对值
    const float abs_a      = fabsf(delta_a);
    const float abs_b      = fabsf(delta_b);

    // 滞回参数（防止切换）
    constexpr float hysteresis = 0.15f; // 约9°

    static uint8_t _chosen_mode;

    // 5. 判断选哪一个（并保持状态避免反复）
    if (_chosen_mode == 0 && abs_b < abs_a - hysteresis)
    {
        _chosen_mode = 1;
    }
    else if (_chosen_mode == 1 && abs_a < abs_b - hysteresis)
    {
        _chosen_mode = 0;
    }

    float chosen_target = _chosen_mode == 0 ? target_a : target_b;

    if (_chosen_mode == 0)
    {
        chosen_target = target_a;
        direction     = 1;
    }
    else
    {
        chosen_target = target_b;
        direction     = -1;
    }

    // 6. 再 wrap 一次最终目标差，得到连续误差
    const float angle_error = _normalize_angle(chosen_target - current_angle);

    target_angle      = angle_error + current_angle;
}

rudder_kin_t::rudder_states_t
rudder_kin_t::solve(const float vx, const float vy, const float wz,
                    const rudder_states_t &current_states) const
{
    rudder_states_t target_states{};
    target_states.modules[0].angle =
        atan2f(vx + wz * _half_wheelbase, vy + wz * _half_track_width);
    target_states.modules[1].angle =
        atan2f(vx - wz * _half_wheelbase, vy + wz * _half_track_width);
    target_states.modules[2].angle =
        atan2f(vx - wz * _half_wheelbase, vy - wz * _half_track_width);
    target_states.modules[3].angle =
        atan2f(vx + wz * _half_wheelbase, vy - wz * _half_track_width);

    for (int i = 0; i < 4; i++)
    {
        calc_angle(target_states.modules[i].angle,
                   current_states.modules[i].angle,
                   target_states.modules[i].direction);
    }

    target_states.modules[0].speed =
        hypotf(vx - wz * _half_wheelbase, vy - wz * _half_track_width) *
        target_states.modules[0].direction;
    target_states.modules[1].speed =
        hypotf(vx - wz * _half_wheelbase, vy + wz * _half_track_width) *
        target_states.modules[1].direction;
    target_states.modules[2].speed =
        hypotf(vx + wz * _half_wheelbase, vy + wz * _half_track_width) *
        target_states.modules[2].direction;
    target_states.modules[3].speed =
        hypotf(vx + wz * _half_wheelbase, vy - wz * _half_track_width) *
        target_states.modules[3].direction;

    return target_states;
}

// rudder_kin_t::solve(const float vx, const float vy, const float wz,
//                     const rudder_states_t &current_states) const
// {
//     rudder_states_t target_states{}; // Zero initialization
//
//     // Optimization: Group rotation vectors by shared coordinates.
//     //
//     // V_rot_x = -wz * y  (Depends on Track Width / Left-Right pos)
//     // V_rot_y =  wz * x  (Depends on Wheelbase / Front-Back pos)
//     //
//     // Left wheels (FL, BL) share the same +Y position -> Same V_rot_x
//     // Right wheels (FR, BR) share the same -Y position -> Same V_rot_x
//     // Front wheels (FL, FR) share the same +X position -> Same V_rot_y
//     // Back wheels (BL, BR) share the same -X position -> Same V_rot_y
//
//     // 1. Calculate shared rotation components
//     // Left side X-component (y = +half_track)
//     const float rot_x_left  = -wz * _half_track_width;
//     // Right side X-component (y = -half_track)
//     const float rot_x_right = -wz * (-_half_track_width);
//
//     // Front side Y-component (x = +half_wheelbase)
//     const float rot_y_front = wz * _half_wheelbase;
//     // Back side Y-component (x = -half_wheelbase)
//     const float rot_y_back  = wz * (-_half_wheelbase);
//
//     // 2. Superpose vectors and optimize (V_target = V_body + V_rot)
//
//     // Front Left (Left X, Front Y)
//     _optimize_module(vx + rot_x_left, vy + rot_y_front,
//     current_states.modules[FL],
//                      target_states.modules[FL]);
//
//     // Front Right (Right X, Front Y)
//     _optimize_module(vx + rot_x_right, vy + rot_y_front,
//     current_states.modules[FR],
//                      target_states.modules[FR]);
//
//     // Back Left (Left X, Back Y)
//     _optimize_module(vx + rot_x_left, vy + rot_y_back,
//     current_states.modules[BL],
//                      target_states.modules[BL]);
//
//     // Back Right (Right X, Back Y)
//     _optimize_module(vx + rot_x_right, vy + rot_y_back,
//     current_states.modules[BR],
//                      target_states.modules[BR]);
//
//     return target_states;
// }
//
// void rudder_kin_t::_optimize_module(const float target_vx,
//                                     const float target_vy,
//                                     const module_state_t &current_state,
//                                     module_state_t &out_state) const
// {
//     // 1. Calculate magnitude (Using ARM DSP hardware instruction if
//     available) const float sum_sq = target_vx * target_vx + target_vy *
//     target_vy; float raw_speed;
//
//     if (sum_sq > 1e-9f)
//     {
//         arm_sqrt_f32(sum_sq, &raw_speed);
//     }
//     else
//     {
//         raw_speed = 0.0f;
//     }
//
//     // 2. Calculate target angle (atan2 from standard math library)
//     const float raw_angle = atan2f(target_vy, target_vx);
//
//     // 3. Deadband check: Prevent steering jitter at near-zero speed
//     if (raw_speed < _deadband)
//     {
//         out_state.speed = 0.0f;
//         out_state.angle = current_state.angle; // Maintain last angle
//         return;
//     }
//
//     // 4. Smart Selection (Optimization)
//     // Calculate the shortest rotation path
//     const float delta = _normalize_angle(raw_angle - current_state.angle);
//
//     // If the turn is > 90 degrees, it's more efficient to reverse the motor
//     // and turn the complementary angle.
//     if (fabsf(delta) > (PI / 2.0f))
//     {
//         // Invert speed and rotate target angle by 180 degrees
//         out_state.speed = -raw_speed;
//         out_state.angle = _normalize_angle(raw_angle + PI);
//     }
//     else
//     {
//         // Normal operation
//         out_state.speed = raw_speed;
//         out_state.angle = raw_angle;
//     }
// }
//
// void rudder_kin_t::compute_odometry(const rudder_states_t &states,
//                                     float &out_vx, float &out_vy,
//                                     float &out_wz) const
// {
//     // Forward Kinematics: Project wheel velocities back to body frame
//     // Using arm_dsp optimized trig functions (Lookup table + interpolation)
//
//     const float fl_vx = states.modules[FL].speed *
//     arm_cos_f32(states.modules[FL].angle); const float fl_vy =
//     states.modules[FL].speed * arm_sin_f32(states.modules[FL].angle);
//
//     const float fr_vx = states.modules[FR].speed *
//     arm_cos_f32(states.modules[FR].angle); const float fr_vy =
//     states.modules[FR].speed * arm_sin_f32(states.modules[FR].angle);
//
//     const float bl_vx = states.modules[BL].speed *
//     arm_cos_f32(states.modules[BL].angle); const float bl_vy =
//     states.modules[BL].speed * arm_sin_f32(states.modules[BL].angle);
//
//     const float br_vx = states.modules[BR].speed *
//     arm_cos_f32(states.modules[BR].angle); const float br_vy =
//     states.modules[BR].speed * arm_sin_f32(states.modules[BR].angle);
//
//     // 1. Average linear velocities
//     out_vx            = (fl_vx + fr_vx + bl_vx + br_vx) / 4.0f;
//     out_vy            = (fl_vy + fr_vy + bl_vy + br_vy) / 4.0f;
//
//     // 2. Estimate Angular Velocity
//     // Contribution from Y-axis velocity difference (Left vs Right)
//     // Wz_y = (Vy_right - Vy_left) / Track_Width
//     //
//     // Optimization note: (fr_vy + br_vy) is total Right Vy
//     //                    (fl_vy + bl_vy) is total Left Vy
//     const float wz_from_y =
//         ((fr_vy + br_vy) - (fl_vy + bl_vy)) / (4.0f * _half_track_width);
//
//     // Contribution from X-axis velocity difference (Front vs Back)
//     // Wz_x = (Vx_front - Vx_back) / Wheelbase
//     const float wz_from_x =
//         ((fl_vx + fr_vx) - (bl_vx + br_vx)) / (4.0f * _half_wheelbase);
//
//     // Average the two estimates
//     out_wz = (wz_from_y - wz_from_x) / 2.0f;
// }

float rudder_kin_t::_normalize_angle(float angle)
{
    // Normalize angle to [-PI, PI]
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

} // namespace pyro