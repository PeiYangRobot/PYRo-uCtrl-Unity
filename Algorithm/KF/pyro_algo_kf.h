/**
 * @file pyro_algo_kf.h
 * @brief 固定存储的线性 KF 与模型无关的 EKF 数值核心。
 *        Fixed-storage linear KF and model-independent EKF numerical core.
 */
#ifndef __PYRO_ALGO_KF_H__
#define __PYRO_ALGO_KF_H__

// 临时屏蔽 CMSIS 的 PI 宏，避免与 pyro::PI 常量冲突；随后恢复原宏。
// Temporarily hide the CMSIS PI macro to include pyro::PI, then restore it.
#pragma push_macro("PI")
#undef PI
#include "pyro_core_def.h"
#pragma pop_macro("PI")
#include "pyro_algo_matrix.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace pyro
{

/**
 * @brief 固定维度、逐拍传入模型矩阵的滤波器。
 *        Fixed-dimension filter with model matrices supplied on each call.
 * @tparam N 状态维度 / State dimension.
 * @tparam Z 最大量测维度，可大于 N / Maximum measurement dimension; may exceed
 * N.
 * @tparam CheckNumerics 是否检查数值有效性，默认关闭；开启后扫描输入和结果。
 *         Enable numerical validation; off by default; scans inputs and results.
 *         关闭时由调用方保证有限值、合法协方差和方差下限，缺测使用 mask。
 *         When off, caller ensures valid numeric inputs; missing rows use mask.
 *
 * 所有矩阵按行优先布局：元素 (row, col) 位于 row * 列数 + col。
 * All matrices are row-major: element (row, col) is at row * columns + col.
 * 大写符号表示矩阵，小写表示向量；私有数学量采用尾部下划线，如 F_、x_。
 * Uppercase symbols denote matrices, lowercase vectors; members use F_, x_,
 * etc. 对象持有内部数组；外部参数仅在调用期间借用，无堆分配或硬件依赖。
 * 不保存外部指针，复制后的实例独立拥有数据。
 * Internal arrays are owned; inputs are borrowed for the call only. No heap
 * allocation or hardware dependency. No stored pointers, so copies are
 * independent.
 *
 * 预测成功提交 x-/P-；修正拒绝或失败保留该预测；预测失败保留上一 x/P。
 * Successful prediction commits x-/P-. Rejected/failed correction keeps that
 * prediction; failed prediction keeps the previous x/P.
 * 同一预测后可依次融合独立量测；同一实例的并发访问由调用方同步。
 * Independent measurements may be corrected successively after one prediction.
 * Concurrent access to one instance must be synchronized by the caller.
 */
template <std::size_t N, std::size_t Z, bool CheckNumerics = false> class kf_t
{
    static_assert(N > 0 && Z > 0 && N <= 65535 && Z <= 65535,
                  "Invalid CMSIS matrix dimensions");

  public:
    /** @brief 协方差修正公式 / Posterior covariance update formula. */
    enum class covariance_update_t : uint8_t
    {
        // Joseph 形式：(I-KH)·P_pred·(I-KH)^T + K·R·K^T。
        // Joseph form: (I-KH)·P_pred·(I-KH)^T + K·R·K^T.
        JOSEPH,
        // 简化形式：P_pred - K·H·P_pred；兼容旧 EKF，不保证数值半正定性。
        // Simplified: P_pred - K·H·P_pred; legacy mode, no numerical PSD guarantee.
        SIMPLIFIED
    };

    /** @brief 观测残差协方差（新息协方差）求解方式。
     *         Measurement residual covariance (innovation covariance) solver.
     */
    enum class solver_t : uint8_t
    {
        // S = LL^T，再做三角求解；要求 S 正定。
        // Factor S = LL^T and use triangular solves; S must be positive definite.
        CHOLESKY,
        // 兼容路径：显式求逆，使用 S 的副本以保留原始观测残差协方差。
        // Compatibility path: invert a copy of S, preserving the original S.
        INVERSE
    };

    /** @brief 最近一次操作的阶段或原因 / Latest operation stage or reason. */
    enum class result_t : uint8_t
    {
        IDLE,      ///< 无操作或重置完成 / Idle or reset completed.
        PREDICTED, ///< 已提交预测 / Prediction committed.
        PREPARED, ///< 修正已准备，尚未提交 / Correction prepared, not committed.
        CORRECTED,      ///< 已提交修正 / Correction committed.
        NO_MEASUREMENT, ///< 无有效量测 / No active measurements.
        REJECTED, ///< 模型门控拒绝量测 / Measurement rejected by model gating.
        INVALID_INPUT, ///< 输入模型或参数无效 / Invalid model input or parameters.
        NOT_INITIALIZED, ///< 尚未初始化 / Not initialized.
        NOT_PREPARED, ///< 无可用的准备结果 / No prepared correction available.
        POLICY_FAILURE,   ///< 外部策略失败 / External policy failed.
        MATH_FAILURE,     ///< CMSIS 运算失败 / CMSIS operation failed.
        NON_FINITE_RESULT ///< 运算产生 NaN/Inf / Operation produced NaN or
                          ///< infinity.
    };

    /** @brief 操作状态和观测残差统计。
     *         Operation status and measurement residual statistics.
     */
    struct diagnostics_t
    {
        // PYRo 返回码 / PYRo return code.
        status_t status               = PYRO_OK;
        // 操作阶段或失败原因 / Operation stage or failure reason.
        result_t result               = result_t::IDLE;
        // CMSIS 原始错误码；不被后续成功运算覆盖。
        // Original CMSIS error code, preserved across subsequent successful work.
        arm_status math_status        = ARM_MATH_SUCCESS;
        // 本次准备或修正实际使用的量测行数 / Active rows in this
        // preparation/correction.
        std::size_t measurement_count = 0;
        // 归一化观测残差平方（标准名称：归一化新息平方，NIS）y^T S^-1 y。
        // Normalized innovation squared (NIS): squared, normalized residual.
        // 读取时检查 PREPARED/CORRECTED/REJECTED 状态。
        // Check PREPARED/CORRECTED/REJECTED before reading.
        float nis                     = 0.0f;
    };

    /// 是否编译数值扫描 / Whether numerical scans are compiled in.
    static constexpr bool check_numerics = CheckNumerics;

    /** @brief 状态向量 x 或预测状态 x-，N×1；各元素含义由模型定义。
     *         State or predicted-state vector x/x-, N×1; model-defined entries.
     */
    using state_t                        = vector_t<N>;
    /** @brief 状态空间协方差 P/P-/P0 或离散过程噪声 Qd，N×N。
     *         State-space covariance P/P-/P0 or discrete process noise Qd, N×N.
     * 对角项为各状态误差方差，非对角项为误差间协方差。
     * Diagonal entries are error variances; off-diagonal entries are
     * covariances. P(i,j) 的单位为状态 i 与 j 单位的乘积；对角项不是标准差。
     * P(i,j) has units of state i times state j; diagonals are not standard
     * deviations. Qd 已包含噪声映射/离散化，核心不会自动乘 dt 或 G。 Qd already
     * includes noise mapping/discretization; no implicit dt or G.
     */
    using covariance_t                   = matrix_t<N, N>;
    /** @brief 线性状态转移矩阵 F 或非线性状态函数的 Jacobian，N×N。
     *         Linear state transition F or nonlinear transition Jacobian, N×N.
     * 与 covariance_t 仅共享存储类型；F 无对称或正定要求。
     * Shares storage with covariance_t only; F need not be symmetric or PSD.
     * 别名不是不同的强类型，物理含义由接口参数确定。
     * Aliases are not distinct strong types; parameter roles define semantics.
     */
    using transition_t                   = matrix_t<N, N>;
    /** @brief 实际量测 z、预测量测 z_pred 或观测残差（新息）y，Z×1。
     *         Measured z, predicted z_pred, or residual (innovation) y, Z×1.
     * y = z - z_pred，即实际观测减预测观测；EKF 中 z_pred = h(x-)。
     * Measurement residual y = z - z_pred; in EKF, z_pred = h(x-).
     */
    using measurement_t                  = vector_t<Z>;
    /** @brief 线性观测矩阵 H 或非线性观测函数的 Jacobian，Z×N。
     *         Linear observation H or nonlinear measurement Jacobian, Z×N.
     * 每行对应一条量测；EKF 中 H*x- 不代替非线性 h(x-)。
     * Each row describes one measurement; H*x- does not replace EKF h(x-).
     */
    using observation_t                  = matrix_t<Z, N>;
    /** @brief 量测噪声协方差 R，或诊断中的观测残差协方差 S，Z×Z。
     *         Measurement noise covariance R or residual covariance S, Z×Z.
     * R 对角项为量测方差；非对角项表示量测噪声相关性，非标准差。
     * R stores variances/covariances, including correlations, not standard
     * deviations.
     */
    using measurement_noise_t            = matrix_t<Z, Z>;
    /** @brief 卡尔曼增益 K，N×Z，将观测残差映射为状态增量 K*y。
     *         Kalman gain K, N×Z; maps residual to state correction K*y.
     * 公共接口保持原始 Z 列顺序，缺失量测列为零。
     * Public gain keeps original Z-column order; missing columns are zero.
     */
    using gain_t                         = matrix_t<N, Z>;
    /** @brief 量测有效掩码，Z 个布尔值；true 启用，false 排除对应行。
     *         Measurement validity mask of Z booleans; true enables a row.
     */
    using measurement_mask_t             = std::array<bool, Z>;

    /** @brief 初始化时选择的数值策略 / Numerical policies selected at
     * initialization. */
    struct options_t
    {
        // 默认 Joseph 协方差修正 / Joseph covariance correction by default.
        covariance_update_t covariance_update = covariance_update_t::JOSEPH;
        // 默认 Cholesky 求解观测残差协方差 S。
        // Cholesky solve of residual covariance S by default.
        solver_t solver                       = solver_t::CHOLESKY;
        // 用 (Pij+Pji)/2 对称化 P / Symmetrize P with (Pij+Pji)/2.
        bool symmetrize                       = true;
        // N 个状态误差方差下限，非状态限幅；初始化、预测、修正后生效。
        // N variance floors, not state limits; applied at
        // reset/predict/correct.
        state_t min_variance{};
    };

    /**
     * @brief 首次初始化 x0/P0 和数值策略；不分配内存。
     *        Initialize x0/P0 and numerical policies without allocation.
     * @param x0 初始状态，N×1 / Initial state, N×1.
     * @param P0 初始误差协方差，N×N / Initial error covariance,
     * N×N.
     * @param options 求解、协方差公式和方差下限 / Solver, covariance mode and
     * floors.
     * @return PYRO_OK 成功；重复初始化返回 PYRO_ALREADY_INIT。
     *         PYRO_OK on success; PYRO_ALREADY_INIT on repeated initialization.
     *         无效参数返回 PYRO_PARAM_ERROR / Invalid parameters:
     * PYRO_PARAM_ERROR.
     */
    status_t init(const state_t &x0, const covariance_t &P0,
                  const options_t &options = options_t{});
    /** @brief 重置 x/P 和诊断、取消待提交修正，保留 options。
     *         Reset x/P and diagnostics, cancel prepared correction, keep
     * options.
     * @return PYRO_OK / PYRO_NOT_FOUND（未初始化 / uninitialized）/
     *         PYRO_PARAM_ERROR（无效 x0/P0 / invalid x0/P0）.
     */
    status_t reset(const state_t &x0, const covariance_t &P0);

    /**
     * @brief 通用预测：提交 x_pred，计算 P-=F*P*Fᵀ+Q_d。
     *        Common prediction: commit x_pred and propagate covariance.
     * @param x_pred 外部状态模型的预测结果，N×1；线性/非线性均用此接口。
     *               Externally predicted state, N×1, for any state model.
     * @param F 状态转移矩阵或雅可比，N×N，仅用于误差传播。
     *          Transition matrix / Jacobian, N×N, for covariance propagation.
     * @param Q_d 本周期离散过程噪声协方差，N×N，已完成噪声映射和离散化。
     *            Discrete process covariance, N×N, already mapped/discretized.
     * @param P_prior 可选输入 P；nullptr 使用当前 P，借用指针不保存。
     *                Optional input P; nullptr uses current P; never retained.
     * @return 成功提交 x/P，失败保留原值。 / Commit on success, otherwise
     * retain.
     */
    status_t predict(const state_t &x_pred, const transition_t &F,
                     const covariance_t &Q_d,
                     const covariance_t *P_prior = nullptr);

    /**
     * @brief 提交模型自算的传播协方差：核心只做 +Q_d、数值检查、对称化/方差下限与提交。
     *        Commit externally propagated covariance; the core only adds Q_d and
     *        bookkeeps.
     * @param x_pred 外部状态模型给出的预测状态，N×1 / Predicted state, N×1.
     * @param P_propagated 调用方算好的 F·P·Fᵀ，N×N / F·P·Fᵀ computed by the caller.
     * @param Q_d 本周期离散过程噪声，N×N / Discrete process covariance.
     * @return 成功提交；失败保留原 x/P / Commit on success, otherwise retain.
     * @details 与 predict(x_pred,F,Q_d) 的提交语义完全相同（含 Joseph 前的对称化与
     *          方差下限），只是省掉通用的两次 N×N 乘法。适用于状态转移有明显结构、
     *          可由模型用更少运算算出 F·P·Fᵀ 的场合（例如 F 为块三角）。
     *          Same commit semantics as predict(x_pred,F,Q_d) minus the two generic
     *          N×N multiplies; useful when the model can exploit F's structure.
     */
    status_t predict_covariance(const state_t &x_pred, const covariance_t &P_propagated,
                                const covariance_t &Q_d);

    /** @brief 修正过程的临时视图，仅在策略调用期间有效。
     *         Transient correction view, valid only during the policy call.
     * @details 默认 dx=K*y；策略调整 K 后无需自行计算 dx。
     *          Default dx=K*y, recomputed after any policy changes to K.
     *          设置 use_dx=true 可覆盖 dx；P 仍使用 K，一致性由模型负责。
     *          use_dx=true overrides dx; P still uses K; model owns
     * consistency.
     */
    struct correction_t
    {
        /// 观测残差（新息）y，Z×1 / Measurement residual (innovation), Z×1.
        const measurement_t &y;
        /// 观测残差协方差 S，Z×Z / Measurement residual covariance, Z×Z.
        const measurement_noise_t &S;
        const diagnostics_t &diagnostics; ///< 包含 NIS / Includes NIS.
        gain_t &K; ///< 可调整增益，N×Z / Adjustable gain, N×Z.
        state_t &dx; ///< 可选外部状态增量，N×1 / Optional external delta, N×1.
        bool use_dx = false; ///< 是否采用外部 dx / Whether to use external dx.
    };

    /** @brief 默认接受量测 / Default measurement acceptance policy. */
    struct accept_t
    {
        status_t operator()(correction_t &) const
        {
            return PYRO_OK;
        }
    };

    /**
     * @brief 通用修正：直接融合外部观测残差 y，保持模型无关。
     *        Common correction: fuse externally computed residual y.
     * @param y 观测残差（新息），Z×1；线性 y=z-H*x，非线性 y=z-h(x)。
     *          Residual (innovation), Z×1: linear z-H*x, nonlinear z-h(x).
     * @param H 观测矩阵或雅可比，Z×N / Observation matrix / Jacobian, Z×N.
     * @param R 量测噪声协方差，Z×Z / Measurement noise covariance, Z×Z.
     * @param mask 有效行掩码；nullptr 启用全部行；有效行由调用方保证有限。
     *             Active mask; nullptr enables all rows; active data must be
     * finite.
     * @param policy 可选无堆分配策略，接收 correction_t&，返回 status_t。
     *               Optional allocation-free policy taking correction_t&.
     *               PYRO_OK 接受，PYRO_WARNING 拒绝，其余值报告策略失败。
     *               PYRO_OK accepts, PYRO_WARNING rejects, other values fail.
     * @details 准备/门控/提交均在本次调用内完成，无外部待提交状态。
     *          Preparation, policy and commit happen within this single call.
     *          策略不能重入同一实例；失败或拒绝保留当前 x/P。
     *          Policy must not reenter this instance; rejection/failure keeps
     * x/P.
     */
    template <typename Policy = accept_t>
    status_t correct(const measurement_t &y, const observation_t &H,
                     const measurement_noise_t &R,
                     const measurement_mask_t *mask = nullptr,
                     Policy policy                  = Policy{});

    /** @brief 最近提交的 x，可能为预测或修正值 / Latest committed
     * predicted/corrected x. */
    [[nodiscard]] const state_t &get_state() const
    {
        return x_;
    }
    /** @brief 最近提交的误差协方差 P / Latest committed error covariance P. */
    [[nodiscard]] const covariance_t &get_covariance() const
    {
        return P_;
    }
    /** @brief 最近准备/应用的 K，不表示量测必然已接受；结合 result 读取。
     *         Latest prepared/applied K; check result to see whether it was
     * accepted.
     */
    [[nodiscard]] const gain_t &get_gain() const
    {
        return K_;
    }
    /** @brief 最近准备的观测残差 y，按原始 Z 次序，缺测行置零。
     *         Latest prepared residual y in original Z order; missing rows
     * are zero.
     */
    [[nodiscard]] const measurement_t &get_innovation() const
    {
        return y_;
    }
    /** @brief 观测残差协方差 S=H*P*H^T+R，保留原始 Z 行列。
     *         Residual covariance S=H*P*H^T+R in original Z order.
     * 求逆不会修改该结果 / Preserved during inversion.
     */
    [[nodiscard]] const measurement_noise_t &get_innovation_covariance() const
    {
        return S_;
    }
    /** @brief 返回最近操作状态和统计 / Return latest operation
     * status/statistics. */
    [[nodiscard]] const diagnostics_t &get_diagnostics() const
    {
        return _diagnostics;
    }
    /** @brief 返回初始化所选策略 / Return policies selected at initialization.
     */
    [[nodiscard]] const options_t &get_options() const
    {
        return _options;
    }
    /** @brief 是否已成功初始化 / Whether initialization has succeeded. */
    [[nodiscard]] bool is_initialized() const
    {
        return _initialized;
    }

  protected:
    // 协方差输入约束 / Covariance input constraints.
    static bool _covariance_valid(const float *values, std::size_t size);
    // 保存操作结果；首次 CMSIS 失败即停止 / Record result; stop at first CMSIS
    // failure.
    status_t _report(status_t status, result_t result,
                     arm_status math_status = ARM_MATH_SUCCESS);

  private:
    status_t _prepare(const measurement_t &y, const observation_t &H,
                      const measurement_noise_t &R,
                      const measurement_mask_t *mask);
    status_t _apply(const gain_t &K, const state_t *dx_override);
    // 求解观测残差协方差 S、增益 K 和 NIS。
    // Solve residual covariance S, gain K and NIS.
    status_t _solve_innovation();
    // 对称化和对角方差下限 / Symmetrization and diagonal variance floors.
    void _finish_covariance(covariance_t &P) const;

    // 初始化/修正准备标志 / Initialization and correction-preparation flags.
    bool _initialized         = false;
    bool _prepared            = false;
    // 有效行数 m 与原始行索引；内部压缩矩阵的行步长按 m 调整。
    // Active count m and original indices; packed matrix strides depend on m.
    std::size_t _active_count = 0;
    std::array<std::size_t, Z> _active{};
    // 数值配置和最近操作诊断 / Numerical policies and latest diagnostics.
    options_t _options{};
    diagnostics_t _diagnostics{};
    // N×1：已提交状态、候选状态和修正增量 / Committed/candidate state and
    // increment.
    state_t x_{}, x_next_{}, dx_{};
    // N×N：已提交/候选 P，以及三个协方差运算工作区。
    // N×N: committed/candidate P and three covariance workspaces.
    covariance_t P_{}, P_next_{}, nn1_{}, nn2_{}, nn3_{};
    // K 为 N×Z；其余为容量 N*Z 的工作区，容纳 N×m 或 m×N 紧凑矩阵。
    // K is N×Z; other N*Z buffers hold packed N×m or m×N intermediates.
    gain_t K_{}, K_active_{}, PHt_{}, nz_{};
    // 容量 Z*N：有效 H(m×N)、H^T(N×m) 和中间结果。
    // Capacity Z*N: active H(m×N), H^T(N×m) and intermediates.
    observation_t H_active_{}, Ht_active_{}, zn_{};
    // 原始次序观测残差 y、压缩残差和 NIS 求解中间结果。
    // Original/packed measurement residual and NIS solve intermediate.
    measurement_t y_{}, y_active_{}, y_solved_{};
    // 公共 S 为 Z×Z；内部 R/S/因子/逆及工作区按有效 m×m 紧凑存储。
    // Public S is Z×Z; internal R/S/factor/inverse/workspace use packed m×m.
    measurement_noise_t S_{}, R_active_{}, S_active_{}, L_{}, S_inv_{}, zz_{};
};

} // namespace pyro

#include "pyro_algo_kf.tpp"
#endif // __PYRO_ALGO_KF_H__
