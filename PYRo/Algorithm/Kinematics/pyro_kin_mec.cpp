#include "pyro_kin_mec.h"
#include "arm_math.h"

namespace pyro
{

mecanum_kin_t::mecanum_kin_t(const float wheelbase, const float track_width)
{
    // Calculate geometry coefficient: k = (a + b)
    // a = wheelbase / 2
    // b = track_width / 2
    // Therefore k = (wheelbase + track_width) / 2
    _k_geom = (abs(wheelbase) + abs(track_width)) / 2.0f;
}

mecanum_kin_t::wheel_speeds_t
mecanum_kin_t::solve(const float vx, const float vy, const float wz) const
{
    wheel_speeds_t ws{}; // Zero initialization

    // Tangential speed contribution from rotation
    const float v_rot = wz * _k_geom;

    // Standard Inverse Kinematics formula for O-configuration Mecanum wheels
    // FL = Vx - Vy - rot
    // FR = Vx + Vy + rot
    // BL = Vx + Vy - rot
    // BR = Vx - Vy + rot

    ws.fl             = vx - vy - v_rot;
    ws.fr             = vx + vy + v_rot;
    ws.bl             = vx + vy - v_rot;
    ws.br             = vx - vy + v_rot;

    return ws;
}

void mecanum_kin_t::compute_odometry(const wheel_speeds_t &speeds,
                                     float &out_vx, float &out_vy,
                                     float &out_wz) const
{
    // Forward Kinematics formula
    // Derived based on O-configuration
    out_vx = (speeds.fl + speeds.fr + speeds.bl + speeds.br) / 4.0f;
    out_vy = (-speeds.fl + speeds.fr + speeds.bl - speeds.br) / 4.0f;

    // Avoid division by zero risk
    if (_k_geom > 1e-6f)
    {
        out_wz =
            (-speeds.fl + speeds.fr - speeds.bl + speeds.br) / (4.0f * _k_geom);
    }
    else
    {
        out_wz = 0.0f;
    }
}

} // namespace pyro