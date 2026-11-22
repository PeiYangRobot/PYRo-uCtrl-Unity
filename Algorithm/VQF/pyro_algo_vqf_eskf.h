// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
/**
 * @file pyro_algo_vqf_eskf.h
 * @brief 六轴 VQF 近惯性系预处理 + 全局姿态误差 ESKF。
 *        Six-axis VQF preprocessing with a global attitude-error ESKF.
 *
 * ============================ 来源与命名 ============================
 * 本实现源自 VQF（Versatile Quaternion-based Filter）：
 *   论文 D. Laidig, T. Seel. "VQF: Highly Accurate IMU Orientation
 *        Estimation with Bias Estimation and Magnetic Disturbance
 *        Rejection." Information Fusion 2023, 91:187-204.
 *   代码 https://github.com/dlaidig/vqf    MIT，全文见同目录 LICENSE
 *
 * 类名由 vqf_t 改为 vqf_eskf_t，以反映本实现的实际构成：
 *   · 姿态传播沿用 VQF 的近惯性系双四元数分解（gyr_quat / acc_quat）
 *     与加速度低通、静止检测；
 *   · 误差状态估计用通用 ESKF（Algorithm/KF/pyro_algo_eskf.h）替代论文
 *     中的启发式增益调度，协方差传播走 F=[[I,M],[0,I]] 的 3×3 块路径。
 * 名字保留 vqf 前缀以标明算法来源，eskf 后缀标明实际的估计方法，
 * 避免读者误认为这是纯 VQF 或纯 ESKF。
 *
 * 本文件对 VQF 部分继续遵循 MIT；本项目的改动部分由项目作者另行授权。
 */
#ifndef PYRO_ALGO_VQF_ESKF_H
#define PYRO_ALGO_VQF_ESKF_H

#include "pyro_algo_eskf.h"
#include "pyro_algo_butterworth.h"

struct imu_data_t;

namespace pyro
{

/** 配置中的时间使用 s，角度使用 °，加速度使用 m/s²。 */
struct vqf_config_t
{
    float tau_acc = 3.0f; ///< 近惯性系二阶加速度低通时间常数。
    bool motion_bias_est_enabled = true; ///< 可信运动观测同时修正三轴零偏。
    bool rest_bias_est_enabled = true; ///< 静止检测及静止零偏观测。
    float bias_sigma_init = 0.5f; ///< 初始零偏标准差，°/s。
    float bias_forgetting_time = 100.0f; ///< 零偏随机游走增长到 0.1 °/s 的时间。
    float bias_clip = 2.0f; ///< 三轴零偏限幅及陀螺静止门限，°/s。
    float bias_sigma_rest = 0.03f; ///< 静止角速度观测标准差，°/s。
    float rest_min_t = 1.0f; ///< 静止确认时间。
    float rest_filter_tau = 0.5f; ///< 静止检测参考低通时间常数。
    float rest_th_gyr = 2.0f; ///< 角速度偏差阈值，°/s。
    float rest_th_acc = 0.5f; ///< 加速度偏差阈值，m/s²。

    float attitude_sigma_init = 30.0f; ///< 初始姿态误差标准差，°。
    float gyro_noise_density = 0.05f; ///< 陀螺白噪声密度，°/s/√Hz。
    float acc_noise = 0.3f; ///< 单轴加速度噪声标准差，m/s²。
    float acc_norm_tolerance = 1.0f; ///< 模长偏差降权尺度，m/s²。
    float acc_vibration_tolerance = 1.0f; ///< 近惯性系高频残差 RMS 降权尺度，m/s²。
    float acc_update_period = 0.010f; ///< ESKF 观测最小间隔，陀螺按全部时间戳传播。
    float max_sample_gap_s = 0.0f; ///< 每路允许的最大采样间隔。0 表示不限制。
    float acc_recovery_t = 0.2f; ///< 启动及受扰后，可信加速度的连续确认时间。
    float acc_nis_threshold = 9.21034f; ///< 两维倾斜新息门限，默认 χ²(2) 的 99% 分位数。
};

/**
 * 固定存储的项目定制 VQF/ESKF，同一实例由调用方同步。
 * 角速度输入 rad/s，加速度输入 m/s²，输出 q 为传感器系到 Z-up 系的 [w,x,y,z]。
 * 陀螺积分、低通与协方差均使用样本的真实时间戳。
 */
class vqf_eskf_t
{
  public:
    using config_t = vqf_config_t;
    using vector3_t = vector_t<3>;
    using quaternion_t = vector_t<4>;

    struct state_t
    {
        quaternion_t quat{1.0f, 0.0f, 0.0f, 0.0f};
        vector3_t bias{}; ///< rad/s。
        matrix_t<3, 3> bias_covariance{}; ///< (rad/s)²。
        matrix_t<6, 6> error_covariance{}; ///< 顺序 [全局 δθ(rad), 机体系 δb(rad/s)]。
        float bias_sigma = 0.0f; ///< 零偏标准差上界，rad/s。
        bool rest_detected = false;
        bool initialized = false; ///< 已完成可信重力方向初始化。
        float rest_t = 0.0f;
        vector3_t rest_gyr_lp{};
        vector3_t rest_acc_lp{};
        uint16_t consumed_gyr = 0;
        uint16_t consumed_acc = 0;
        uint16_t updated_gyr = 0; ///< 最近一次实际更新时间的样本下标 + 1，0 表示未更新。
        uint16_t updated_acc = 0;
        float acc_confidence = 0.0f; ///< 最近加速度的可信度，0..1。
        float acc_nis = 0.0f; ///< 最近一次两维倾斜观测的 NIS。
        bool acc_rejected = false; ///< 最近样本受到门控或处于恢复等待。
        uint32_t acc_corrections = 0; ///< 成功提交的 ESKF 倾斜观测数。
        uint32_t acc_rejections = 0; ///< 被拒绝或处于恢复等待的加速度帧数。
        kf_t<6, 3, true>::diagnostics_t bias_diagnostics{}; ///< 联合误差滤波器诊断，保留字段名。
    };

    /** 首次调用初始化，后续调用保留姿态/零偏并应用参数。
     * 非法输入保留配置和状态。改变低通/门控参数时重新确认可信加速度。
     * 初始标准差参数在 reset 时应用。 */
    status_t configure(const config_t &config = config_t{});
    /** 重置时间基准、姿态、协方差与所有滤波历史。bias/sigma 使用 rad/s。
     * sigma=0 使用 bias_sigma_init，重置后重新确认重力方向。 */
    status_t reset(const vector3_t &bias = vector3_t{}, float sigma = 0.0f);
    /** 按时间戳合并两路未消费样本，同时间戳先陀螺后加速度。
     * 各路各自单调并共享 tick_s，批次时间跨度 < 2^31 tick，支持 uint32 回绕。
     * 首帧建立时钟，重复时间戳消费后返回 WARNING。倒退帧消费后报 PARAM_ERROR。
     * 超过 max_sample_gap_s 的样本保留，调用方可 reset 后用它建立新基准。
     * 设备计数器重置或 tick_s 改变时调用 reset。
     * consumed_gyr/acc 为本次实际消费前缀，updated_gyr/acc 为最近一次更新时间的前缀长度。
     * 门控/恢复等待返回 WARNING 并继续本批。其他错误终止批次，保留已提交步骤。
     * 加速度帧在处理开始后计入消费量，避免重试时重复推进滤波历史。 */
    status_t update(const imu_data_t &data);
    /* 填充出参版本：热路径（1 kHz）用它避免把约 200 B 的 state_t 按值返回再拷贝。 */
    void get_state(state_t &out) const;
    /* 按值返回的便捷版本；测试与一次性诊断使用。 */
    [[nodiscard]] state_t get_state() const;

    // 诊断分支，0 正常，1 流非法，2 tick_s 非法，3 tick_s 改变，4 时间倒退，5 dt 非法，6 超过最大采样间隔。
    inline static volatile uint32_t err_stage = 0U;

  private:
    using error_kf_t = eskf_t<6, 3, true>;
    struct sample_clock_t
    {
        uint32_t timestamp = 0;
        bool initialized = false;
    };
    struct filter_state_t
    {
        quaternion_t gyr_quat{1.0f, 0.0f, 0.0f, 0.0f};
        quaternion_t acc_quat{1.0f, 0.0f, 0.0f, 0.0f};
        vector3_t bias{};
        vector3_t last_acc_lp{};
        vector3_t acc_reference{};
        vector3_t rest_gyr_lp{};
        vector3_t rest_acc_lp{};
        float vibration_variance = 0.0f;
        float trusted_acc_t = 0.0f;
        float observation_t = 0.0f;
        float rest_t = 0.0f;
        float acc_confidence = 0.0f;
        float acc_nis = 0.0f;
        bool reference_initialized = false;
        bool tilt_initialized = false;
        bool rest_gyr_ok = false;
        bool rest_detected = false;
        bool acc_rejected = false;
        uint32_t acc_corrections = 0;
        uint32_t acc_rejections = 0;
        uint16_t consumed_gyr = 0;
        uint16_t consumed_acc = 0;
        uint16_t updated_gyr = 0; ///< 最近一次实际更新时间的样本下标 + 1，0 表示未更新。
        uint16_t updated_acc = 0;
        sample_clock_t gyr_clock{};
        sample_clock_t acc_clock{};
        float timestamp_tick_s = 0.0f;
    };

    status_t _update_gyr(const vector3_t &gyr, float dt);
    status_t _update_acc(const vector3_t &acc, float dt);
    status_t _initialize_tilt();
    status_t _correct_acc(float interval);
    status_t _correct_rest(float interval);
    status_t _inject_error(const error_kf_t::state_t &delta);
    status_t _gate(error_kf_t::correction_t &correction, bool acceleration);
    status_t _correct(const vector3_t &residual, const error_kf_t::observation_t &H,
                      const error_kf_t::measurement_noise_t &R, bool acceleration);
    void _reject_acc();

    config_t _config{};
    filter_state_t _state{};
    error_kf_t _error_kf{};
    error_kf_t::diagnostics_t _diagnostics{};
    butterworth_lpf_t<3> _acc_lp{};
    butterworth_lpf_t<3> _rest_gyr_lp{};
    butterworth_lpf_t<3> _rest_acc_lp{};

    /* ---- 热路径复用暂存 ----------------------------------------------------
     * 原来 _update_gyr / _inject_error 每个样本都在栈上构造 7~9 个 matrix_t
     * 临时量，而 matrix_t 默认构造会逐字节清零（newlib memset 是 byte 循环），
     * PC 采样显示 memset 占 11% CPU、约 4 KB/ms 的零初始化。
     * 提为成员复用后不再逐样本清零。
     *
     * 误差状态转移 F=[[I,M],[0,I]] 与误差注入 G=[[J,0],[0,I]] 都是块结构，
     * 协方差传播因此直接用 3×3 块运算（见 _propagate_covariance / _rotate_covariance），
     * 不再构造 6×6 的 F/G。
     *
     * 约束：各块暂存每次使用前都被完整写满，因此无需初始化；_q 由 _update_gyr
     * 每拍完整写入（含显式清零的右下 3×3 非对角块）。 */
    matrix_t<3, 3> _jacobian{};   /* J_l(angle) */
    matrix_t<3, 3> _skew{};       /* [v]x */
    matrix_t<3, 3> _skew_sq{};    /* [v]x² = [v]x·[v]x */
    matrix_t<3, 3> _rot{};        /* R_start */
    matrix_t<3, 3> _avg_rot{};    /* A = R_start·J_l */
    matrix_t<3, 3> _avg_rot_t{};  /* Aᵀ */
    matrix_t<3, 3> _avg_rot_at{}; /* A·Aᵀ */
    error_kf_t::covariance_t _q{};   /* 本拍过程噪声 Q */
    matrix_t<6, 6> _p_prop{};        /* F·P·Fᵀ 或 G·P·Gᵀ，提交给 KF 前 */
    matrix_t<3, 3> _m{};             /* 过渡耦合块 M = -A·dt，即 F 的右上块 */
    matrix_t<3, 3> _m_t{};           /* Mᵀ；误差注入时复用为 Jᵀ */
    matrix_t<3, 3> _p00{};           /* P 的 3×3 分块（对称，P10 由 P01 转置） */
    matrix_t<3, 3> _p01{};
    matrix_t<3, 3> _p10{};
    matrix_t<3, 3> _p11{};
    matrix_t<3, 3> _t1{};            /* 块乘法暂存 */
    matrix_t<3, 3> _t2{};
    matrix_t<3, 3> _t3{};

    /* ---- 半角三角函数暂存（2026-11-17） ------------------------------------
     * _update_gyr 对同一陀螺样本要算两次三角函数：
     *   rotation_step(angle)         用半角 h = |angle|/2 的 sin/cos
     *   _compute_left_jacobian(angle) 用全角 |angle| 的 sin/cos
     * 让前者把 sin(h)/cos(h) 存进这两个成员并置 _trig_valid，
     * 后者用半角公式反推全角值，省下 2 次 sinf/cosf 调用。
     * dt>0 分支每次都会重写，不存在跨样本残留；为 false 时退回直接计算。 */
    float _trig_sin{0.0f};
    float _trig_cos{1.0f};
    bool  _trig_valid{false};

    status_t _compute_left_jacobian(const vector3_t &angle);
    status_t _load_p_blocks();  /* 取当前 P 的四个 3×3 块 */
    status_t _propagate_covariance(); /* F·P·Fᵀ → _p_prop（3 次 3×3） */
    status_t _rotate_covariance();    /* G·P·Gᵀ → _p_prop（3 次 3×3） */
    static void _get_block(const matrix_t<6, 6> &src, std::size_t row0,
                           std::size_t col0, matrix_t<3, 3> &dst);
    static void _set_block(matrix_t<6, 6> &dst, std::size_t row0,
                           std::size_t col0, const matrix_t<3, 3> &src);
};

} // namespace pyro
#endif
