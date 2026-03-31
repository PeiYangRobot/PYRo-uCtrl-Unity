#include "pyro_kin_mec.h"
#include "arm_math.h"
#include <cmath>

namespace pyro
{

mecanum_kin_t::mecanum_kin_t(const float wheelbase, const float track_width)
{
    // Calculate geometry coefficient: k = (a + b)
    // a = wheelbase / 2
    // b = track_width / 2
    // Therefore k = (wheelbase + track_width) / 2
    _k_geom = (std::fabs(wheelbase) + std::fabs(track_width)) / 2.0f;
}

// ============================================================================
// Inverse Kinematics / 逆运动学 (推力重分配容错版)
// ============================================================================
mecanum_kin_t::wheel_speeds_t
mecanum_kin_t::solve(const float vx, const float vy, const float wz, const missing_mec_e missing) const
{
    wheel_speeds_t ws{}; // Zero initialization
    const float v_rot = wz * _k_geom;

    switch (missing)
    {
        case missing_mec_e::FL:
            ws.fl = 0.0f;
            ws.fr = vy + v_rot;
            ws.bl = vx - v_rot;
            ws.br = vx - vy;
            break;

        case missing_mec_e::FR:
            ws.fl = -vy - v_rot;
            ws.fr = 0.0f;
            ws.bl = vx + vy;
            ws.br = vx + v_rot;
            break;

        case missing_mec_e::BL:
            ws.fl = vx - v_rot;
            ws.fr = vx + vy;
            ws.bl = 0.0f;
            ws.br = -vy + v_rot;
            break;

        case missing_mec_e::BR:
            ws.fl = vx - vy;
            ws.fr = vx + v_rot;
            ws.bl = vy - v_rot;
            ws.br = 0.0f;
            break;

        case missing_mec_e::NONE:
        default:
            ws.fl = vx - vy - v_rot;
            ws.fr = vx + vy + v_rot;
            ws.bl = vx + vy - v_rot;
            ws.br = vx - vy + v_rot;
            break;
    }

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