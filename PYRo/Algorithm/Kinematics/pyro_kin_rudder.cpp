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

rudder_kin_t::rudder_states_t
rudder_kin_t::solve(const float vx, const float vy, const float wz,
                    const rudder_states_t &current_states) const
{
    rudder_states_t target_states{}; // Zero initialization

    // Optimization: Group rotation vectors by shared coordinates.
    //
    // V_rot_x = -wz * y  (Depends on Track Width / Left-Right pos)
    // V_rot_y =  wz * x  (Depends on Wheelbase / Front-Back pos)
    //
    // Left wheels (FL, BL) share the same +Y position -> Same V_rot_x
    // Right wheels (FR, BR) share the same -Y position -> Same V_rot_x
    // Front wheels (FL, FR) share the same +X position -> Same V_rot_y
    // Back wheels (BL, BR) share the same -X position -> Same V_rot_y

    // 1. Calculate shared rotation components
    // Left side X-component (y = +half_track)
    const float rot_x_left  = -wz * _half_track_width;
    // Right side X-component (y = -half_track)
    const float rot_x_right = -wz * (-_half_track_width);

    // Front side Y-component (x = +half_wheelbase)
    const float rot_y_front = wz * _half_wheelbase;
    // Back side Y-component (x = -half_wheelbase)
    const float rot_y_back  = wz * (-_half_wheelbase);

    // 2. Superpose vectors and optimize (V_target = V_body + V_rot)

    // Front Left (Left X, Front Y)
    _optimize_module(vx + rot_x_left, vy + rot_y_front, current_states.modules[FL],
                     target_states.modules[FL]);

    // Front Right (Right X, Front Y)
    _optimize_module(vx + rot_x_right, vy + rot_y_front, current_states.modules[FR],
                     target_states.modules[FR]);

    // Back Left (Left X, Back Y)
    _optimize_module(vx + rot_x_left, vy + rot_y_back, current_states.modules[BL],
                     target_states.modules[BL]);

    // Back Right (Right X, Back Y)
    _optimize_module(vx + rot_x_right, vy + rot_y_back, current_states.modules[BR],
                     target_states.modules[BR]);

    return target_states;
}

void rudder_kin_t::_optimize_module(const float target_vx,
                                    const float target_vy,
                                    const module_state_t &current_state,
                                    module_state_t &out_state) const
{
    // 1. Calculate magnitude (Using ARM DSP hardware instruction if available)
    const float sum_sq = target_vx * target_vx + target_vy * target_vy;
    float raw_speed;

    if (sum_sq > 1e-9f)
    {
        arm_sqrt_f32(sum_sq, &raw_speed);
    }
    else
    {
        raw_speed = 0.0f;
    }

    // 2. Calculate target angle (atan2 from standard math library)
    const float raw_angle = atan2f(target_vy, target_vx);

    // 3. Deadband check: Prevent steering jitter at near-zero speed
    if (raw_speed < _deadband)
    {
        out_state.speed = 0.0f;
        out_state.angle = current_state.angle; // Maintain last angle
        return;
    }

    // 4. Smart Selection (Optimization)
    // Calculate the shortest rotation path
    const float delta = _normalize_angle(raw_angle - current_state.angle);

    // If the turn is > 90 degrees, it's more efficient to reverse the motor
    // and turn the complementary angle.
    if (fabsf(delta) > (PI / 2.0f))
    {
        // Invert speed and rotate target angle by 180 degrees
        out_state.speed = -raw_speed;
        out_state.angle = _normalize_angle(raw_angle + PI);
    }
    else
    {
        // Normal operation
        out_state.speed = raw_speed;
        out_state.angle = raw_angle;
    }
}

void rudder_kin_t::compute_odometry(const rudder_states_t &states,
                                    float &out_vx, float &out_vy,
                                    float &out_wz) const
{
    // Forward Kinematics: Project wheel velocities back to body frame
    // Using arm_dsp optimized trig functions (Lookup table + interpolation)

    const float fl_vx = states.modules[FL].speed * arm_cos_f32(states.modules[FL].angle);
    const float fl_vy = states.modules[FL].speed * arm_sin_f32(states.modules[FL].angle);

    const float fr_vx = states.modules[FR].speed * arm_cos_f32(states.modules[FR].angle);
    const float fr_vy = states.modules[FR].speed * arm_sin_f32(states.modules[FR].angle);

    const float bl_vx = states.modules[BL].speed * arm_cos_f32(states.modules[BL].angle);
    const float bl_vy = states.modules[BL].speed * arm_sin_f32(states.modules[BL].angle);

    const float br_vx = states.modules[BR].speed * arm_cos_f32(states.modules[BR].angle);
    const float br_vy = states.modules[BR].speed * arm_sin_f32(states.modules[BR].angle);

    // 1. Average linear velocities
    out_vx            = (fl_vx + fr_vx + bl_vx + br_vx) / 4.0f;
    out_vy            = (fl_vy + fr_vy + bl_vy + br_vy) / 4.0f;

    // 2. Estimate Angular Velocity
    // Contribution from Y-axis velocity difference (Left vs Right)
    // Wz_y = (Vy_right - Vy_left) / Track_Width
    //
    // Optimization note: (fr_vy + br_vy) is total Right Vy
    //                    (fl_vy + bl_vy) is total Left Vy
    const float wz_from_y =
        ((fr_vy + br_vy) - (fl_vy + bl_vy)) / (4.0f * _half_track_width);

    // Contribution from X-axis velocity difference (Front vs Back)
    // Wz_x = (Vx_front - Vx_back) / Wheelbase
    const float wz_from_x =
        ((fl_vx + fr_vx) - (bl_vx + br_vx)) / (4.0f * _half_wheelbase);

    // Average the two estimates
    out_wz = (wz_from_y - wz_from_x) / 2.0f;
}

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