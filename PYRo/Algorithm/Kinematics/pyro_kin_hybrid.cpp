#include "pyro_kin_hybrid.h"
#include "arm_math.h" 

namespace pyro
{

hybrid_kin_t::hybrid_kin_t(const float track_spacing, const float mec_wheelbase,
                           const float mec_track_width)
{
    _k_track = abs(track_spacing) / 2.0f;
    _k_mec = (abs(mec_wheelbase) + abs(mec_track_width)) / 2.0f;
}

hybrid_kin_t::hybrid_speeds_t
hybrid_kin_t::solve(const float vx, const float vy, const float wz, const drive_mode_t mode) const
{
    hybrid_speeds_t ws{}; // Zero initialize

    if (mode == drive_mode_t::CRUISING)
    {
        // --- Mode: CRUISING ---
        // Tracks are lifted or idle.
        ws.track_l = 0.0f;
        ws.track_r = 0.0f;

        // Mecanum handles full Omni-directional movement
        const float v_rot = wz * _k_mec;

        ws.mec_fl = vx - vy - v_rot;
        ws.mec_fr = vx + vy + v_rot;
        ws.mec_bl = vx + vy - v_rot;
        ws.mec_br = vx - vy + v_rot;
    }
    else
    {
        // --- Mode: CLIMBING / OBSTACLE ---
        // 1. Force Vy to 0. 
        // Physics constraint: Tracks have immense friction sideways. 
        // Attempting to strafe while tracks are grounded will stall motors.
        constexpr float effective_vy = 0.0f;

        // 2. Tracks Logic (Differential Drive)
        const float v_rot_track = wz * _k_track;
        ws.track_l = vx - v_rot_track;
        ws.track_r = vx + v_rot_track;

        // 3. Mecanum Logic (Assisting Differential Drive)
        // Even without Vy, we calculate Mecanum speeds to match the rotation
        // and forward movement, preventing drag.
        const float v_rot_mec = wz * _k_mec;

        ws.mec_fl = vx - effective_vy - v_rot_mec;
        ws.mec_fr = vx + effective_vy + v_rot_mec;
        ws.mec_bl = vx + effective_vy - v_rot_mec;
        ws.mec_br = vx - effective_vy + v_rot_mec;
    }

    return ws;
}

} // namespace pyro