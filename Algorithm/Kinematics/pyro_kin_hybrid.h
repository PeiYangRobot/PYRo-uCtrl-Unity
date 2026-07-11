#ifndef __PYRO_KIN_HYBRID__
#define __PYRO_KIN_HYBRID__

namespace pyro
{

/**
 * @brief Hybrid Chassis Kinematics (Tracks + Mecanum)
 * 混合底盘运动学解算 (履带 + 麦克纳姆轮)
 * Supports standard and arbitrary asymmetric chassis geometries.
 * 支持标准对称底盘以及任意非对称畸形底盘。
 */
class hybrid_kin_t
{
  public:
    /**
     * @brief Missing Mecanum Wheel Enumeration
     * 缺失麦轮枚举类，用于容错控制
     */
    enum class missing_mec_e
    {
        NONE = 0, // 正常状态，无缺失
        FL,       // 左前轮缺失/失效
        FR,       // 右前轮缺失/失效
        BL,       // 左后轮缺失/失效
        BR        // 右后轮缺失/失效
    };

    /**
     * @brief Output structure containing target speeds for all 6 traction motors
     * 包含全部6个驱动电机目标速度的输出结构体
     */
    struct hybrid_speeds_t
    {
        // Front Tracks 前置履带
        float track_l;
        float track_r;

        // Rear Mecanum Wheels 后置麦克纳姆轮
        float mec_fl;
        float mec_fr;
        float mec_bl;
        float mec_br;
    };

    /**
     * @brief Output structure containing solved chassis velocities
     * 包含解算出的底盘平移和旋转速度的输出结构体
     */
    struct chassis_speeds_t
    {
        float vx; // Linear velocity X (m/s) 前后平移速度
        float vy; // Linear velocity Y (m/s) 左右平移速度
        float wz; // Angular velocity Z (rad/s) 自转角速度
    };

    /**
     * @brief 1. Expert Constructor (Independent K-values for each wheel)
     * 专家模式构造函数 (为每个轮子传入独立的K值)
     * For extreme asymmetric or shifted center-of-gravity setups.
     * 适用于重心严重偏移或非对称的异形底盘。
     * K = |X distance to Center of Rotation| + |Y distance to Center of Rotation|
     * K值定义: K = |轮子到旋转中心的X轴距离| + |轮子到旋转中心的Y轴距离|
     * @param track_spacing Distance between the centers of the two tracks (m) 左右履带中心间距(m)
     * @param k_fl          Left-Front Mecanum K-value 左前麦轮旋转系数
     * @param k_fr          Right-Front Mecanum K-value 右前麦轮旋转系数
     * @param k_bl          Back-Left Mecanum K-value 左后麦轮旋转系数
     * @param k_br          Back-Right Mecanum K-value 右后麦轮旋转系数
     */
    hybrid_kin_t(float track_spacing, float k_fl, float k_fr, float k_bl, float k_br);

    /**
     * @brief 2. Standard Constructor (Ideal rectangular chassis)
     * 标准模式构造函数 (理想矩形对称底盘)
     * Automatically calculates unified K-value from dimensions.
     * 自动通过长宽计算统一的K值。
     * @param track_spacing   Distance between the centers of the two tracks (m) 左右履带中心间距(m)
     * @param mec_wheelbase   Wheelbase of the mecanum section (m) 麦轮总轴距(前后轮心距离)(m)
     * @param mec_track_width Track width of the mecanum section (m) 麦轮总轮距(左右轮心距离)(m)
     */
    hybrid_kin_t(float track_spacing, float mec_wheelbase, float mec_track_width);

    /**
     * @brief Inverse Kinematics Solver 逆运动学解算器 (底盘速度 -> 轮转速，带容错)
     * @param vx       Linear velocity X (m/s) 前后平移速度
     * @param vy       Linear velocity Y (m/s) 左右平移速度
     * @param wz       Angular velocity Z (rad/s) 自转角速度
     * @param track_en true: Tracks enabled (Climbing mode), false: Cruising mode
     * @param missing  Missing wheel identifier 缺失/失效轮的枚举，默认无缺失
     * @return hybrid_speeds_t Solved speeds for all motors
     */
    [[nodiscard]] hybrid_speeds_t
    solve(float vx, float vy, float wz, bool track_en, missing_mec_e missing = missing_mec_e::NONE) const;

    /**
     * @brief Forward Kinematics Solver (Mecanum only)
     * 正运动学解算器 (麦轮转速 -> 底盘速度)
     * 根据当前麦轮的实际反馈速度，精准推算底盘的真实运动速度 (已包含容错控制映射)
     * @param mec_fl   Front-Left Mecanum wheel speed 左前麦轮转速
     * @param mec_fr   Front-Right Mecanum wheel speed 右前麦轮转速
     * @param mec_bl   Back-Left Mecanum wheel speed 左后麦轮转速
     * @param mec_br   Back-Right Mecanum wheel speed 右后麦轮转速
     * @param missing  Missing wheel identifier 缺失/失效轮的枚举，默认无缺失
     * @return chassis_speeds_t Solved chassis velocities 计算出的底盘速度
     */
    [[nodiscard]] chassis_speeds_t
    forward_solve(float mec_fl, float mec_fr, float mec_bl, float mec_br, missing_mec_e missing = missing_mec_e::NONE) const;

  private:
    float _k_track; // Track rotation coefficient 履带旋转系数 (track_spacing / 2)

    // Independent geometry coefficients for the 4 mecanum wheels
    // 四个麦轮独立的几何结构常数 (K = lx + ly)
    float _k_fl;
    float _k_fr;
    float _k_bl;
    float _k_br;
};

} // namespace pyro

#endif