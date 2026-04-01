#include "pyro_kin_hybrid.h"
#include "arm_math.h" 
#include <cmath> // For std::abs or fabsf

namespace pyro
{

// ============================================================================
// 1. Expert Constructor / 专家模式构造函数
// ============================================================================
hybrid_kin_t::hybrid_kin_t(const float track_spacing,
                           const float k_fl, const float k_fr,
                           const float k_bl, const float k_br)
{
    // Tracks are assumed symmetric left-to-right
    // 假设履带左右对称，取间距的一半
    _k_track = abs(track_spacing) / 2.0f;

    // Store directly calculated K-values for each wheel
    // 直接存储用户传入的独立 K 值
    _k_fl = abs(k_fl);
    _k_fr = abs(k_fr);
    _k_bl = abs(k_bl);
    _k_br = abs(k_br);
}

// ============================================================================
// 2. Standard Constructor / 标准模式构造函数
// ============================================================================
hybrid_kin_t::hybrid_kin_t(const float track_spacing,
                           const float mec_wheelbase, const float mec_track_width)
{
    _k_track = abs(track_spacing) / 2.0f;

    // Standard formula: K = (Wheelbase + TrackWidth) / 2
    // 标准底盘公式：K = (轴距 + 轮距) / 2  (等价于 半轴距 + 半轮距)
    const float k_std = (abs(mec_wheelbase) + abs(mec_track_width)) / 2.0f;

    // Apply unified K to all wheels
    // 将统一的 K 赋值给所有轮子
    _k_fl = k_std;
    _k_fr = k_std;
    _k_bl = k_std;
    _k_br = k_std;
}

// ============================================================================
// Kinematics Solver / 运动学解算器
// ============================================================================
hybrid_kin_t::hybrid_speeds_t
hybrid_kin_t::solve(const float vx, const float vy, const float wz, const bool track_en, const missing_mec_e missing) const
{
    hybrid_speeds_t ws{}; // 初始化为零

    const float v_rot_fl = wz * _k_fl;
    const float v_rot_fr = wz * _k_fr;
    const float v_rot_bl = wz * _k_bl;
    const float v_rot_br = wz * _k_br;

    // --- 1. Mecanum Control Allocation / 麦轮推力重分配逻辑 ---
    switch (missing)
    {
        case missing_mec_e::FL:
            ws.mec_fl = 0.0f;
            ws.mec_fr = vy + v_rot_fr;     // 去除 vx
            ws.mec_bl = vx - v_rot_bl;     // 去除 vy
            ws.mec_br = vx - vy;           // 去除 wz
            break;

        case missing_mec_e::FR:
            ws.mec_fl = -vy - v_rot_fl;    // 去除 vx
            ws.mec_fr = 0.0f;
            ws.mec_bl = vx + vy;           // 去除 wz
            ws.mec_br = vx + v_rot_br;     // 去除 vy
            break;

        case missing_mec_e::BL:
            ws.mec_fl = vx - v_rot_fl;     // 去除 vy
            ws.mec_fr = vx + vy;           // 去除 wz
            ws.mec_bl = 0.0f;
            ws.mec_br = -vy + v_rot_br;    // 去除 vx
            break;

        case missing_mec_e::BR:
            ws.mec_fl = vx - vy;           // 去除 wz
            ws.mec_fr = vx + v_rot_fr;     // 去除 vy
            ws.mec_bl = vy - v_rot_bl;     // 去除 vx
            ws.mec_br = 0.0f;
            break;

        case missing_mec_e::NONE:
        default:
            // 标准 4 轮全向解算
            ws.mec_fl = vx - vy - v_rot_fl;
            ws.mec_fr = vx + vy + v_rot_fr;
            ws.mec_bl = vx + vy - v_rot_bl;
            ws.mec_br = vx - vy + v_rot_br;
            break;
    }

    // --- 2. Track Logic / 履带逻辑 ---
    if (track_en)
    {
        // 爬坡模式：履带提供前后和旋转差速
        const float v_rot_track = wz * _k_track;
        ws.track_l = vx - v_rot_track;
        ws.track_r = vx + v_rot_track;
    }
    else
    {
        // 巡航模式：履带悬空或怠速
        ws.track_l = 0.0f;
        ws.track_r = 0.0f;
    }

    return ws;
}

} // namespace pyro