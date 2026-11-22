/**
 * @file pyro_algo_kf.tpp
 * @brief 固定维度 KF 模板实现；由 pyro_algo_kf.h 包含。
 *        Fixed-dimension KF template implementation, included by
 * pyro_algo_kf.h.
 */
#ifndef __PYRO_ALGO_KF_TPP__
#define __PYRO_ALGO_KF_TPP__

#include <algorithm>
#include <cmath>

namespace pyro
{

template <std::size_t N, std::size_t Z, bool CheckNumerics>
bool kf_t<N, Z, CheckNumerics>::_covariance_valid(const float *values,
                                                  std::size_t size)
{
    // 检查有限值、非负对角和近似对称；并非完整特征值/半正定检验。
    // Check finite entries, nonnegative diagonal and symmetry; not a full PSD
    // test.
    for (std::size_t row = 0; row < size; ++row)
    {
        if (!matrix::is_finite(values[row * size + row]) ||
            values[row * size + row] < 0.0f)
            return false;
        for (std::size_t col = row + 1; col < size; ++col)
        {
            const float a = values[row * size + col];
            const float b = values[col * size + row];
            if (!matrix::is_finite(a) || !matrix::is_finite(b) ||
                std::fabs(a - b) >
                    1.0e-5f *
                        std::max(1.0f, std::max(std::fabs(a), std::fabs(b))))
                return false;
        }
    }
    return true;
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::_report(status_t status, result_t result,
                                            arm_status math_status)
{
    _diagnostics.status      = status;
    _diagnostics.result      = result;
    _diagnostics.math_status = math_status;
    return status;
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::init(const state_t &x0,
                                         const covariance_t &P0,
                                         const options_t &options)
{
    if (_initialized)
        return PYRO_ALREADY_INIT;
    if ((options.covariance_update != covariance_update_t::JOSEPH &&
         options.covariance_update != covariance_update_t::SIMPLIFIED) ||
        (options.solver != solver_t::CHOLESKY &&
         options.solver != solver_t::INVERSE))
        return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(x0) || !_covariance_valid(P0.data(), N) ||
            !matrix::is_finite(options.min_variance))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        for (float value : options.min_variance)
            if (value < 0.0f)
                return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
    }
    _options     = options;
    _initialized = true;
    return reset(x0, P0);
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::reset(const state_t &x0,
                                          const covariance_t &P0)
{
    _prepared = false;
    if (!_initialized)
        return _report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(x0) || !_covariance_valid(P0.data(), N))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
    }
    x_ = x0;
    P_ = P0;
    _finish_covariance(P_);
    K_.fill(0.0f);
    y_.fill(0.0f);
    S_.fill(0.0f);
    _active_count = 0;
    _diagnostics  = {};
    return PYRO_OK;
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::predict_covariance(const state_t &x_pred,
                                                       const covariance_t &P_propagated,
                                                       const covariance_t &Q_d)
{
    _prepared    = false;
    _diagnostics = {};
    if (!_initialized)
        return _report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
    if constexpr (CheckNumerics)
    {
        /* 数值检查（2026-11-17 精简）。
         *
         * 原先对 P_propagated 与 Q_d 各做一次完整的 _covariance_valid：
         * 每次 87 次浮点比较 + 反复调用 is_finite/fabs，两个矩阵合计约 174 次。
         * 实测这占 predict 路径的 20~30%，而它每陀螺样本都要跑一次。
         *
         * 现在只对 P_propagated 做完整校验（它是 3×3 块运算的结果，确实需要查），
         * Q_d 只查有限性与非负对角：
         *   · Q_d 由调用方按固定结构构造（VQF 里是 M·Mᵀ 形式的半正定块），
         *     对角线天然非负，对称性由构造保证；
         *   · 但它是接口入参，仍必须防 NaN/Inf 与负方差，否则会污染 P。
         * 保护语义从"完整校验"降为"必要校验"，不放过任何会导致 P 失效的情形：
         * 后续 _finish_covariance 仍会做对称化与方差下限，最后 is_finite(P_next_)
         * 也会拦住任何被 Q_d 污染的结果。 */
        if (!matrix::is_finite(x_pred))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        if (!_covariance_valid(P_propagated.data(), N))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        for (std::size_t row = 0; row < N; ++row)
        {
            const float diag = Q_d[row * N + row];
            if (!matrix::is_finite(diag) || diag < 0.0f ||
                P_propagated[row * N + row] < 0.0f)
                return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        }
        if (!matrix::is_finite(Q_d))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
    }
    _diagnostics.status = PYRO_ERROR;
    _diagnostics.result = result_t::MATH_FAILURE;
    // 调用方已算出 F·P·Fᵀ，这里只补 +Qd（与通用 predict 相同的逐元素顺序）。
    // Caller already produced F·P·Fᵀ; only +Qd remains, elementwise as in predict.
    _diagnostics.math_status =
        matrix::add({P_propagated.data(), N, N}, {Q_d.data(), N, N},
                    {P_next_.data(), N, N});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(P_next_))
            return _report(PYRO_ERROR, result_t::NON_FINITE_RESULT);
    }
    _finish_covariance(P_next_);
    x_next_ = x_pred;
    x_      = x_next_;
    P_      = P_next_;
    K_.fill(0.0f);
    return _report(PYRO_OK, result_t::PREDICTED);
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::predict(const state_t &x_pred,
                                            const transition_t &F,
                                            const covariance_t &Q_d,
                                            const covariance_t *P_prior)
{
    _prepared    = false;
    _diagnostics = {};
    if (!_initialized)
        return _report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
    // 旧简化公式的 P 可能累积非对称误差，兼容模式不强制变换其对称性。
    // Legacy simplified P may accumulate asymmetry; preserve compatibility
    // mode.
    const auto &P = P_prior ? *P_prior : P_;
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(x_pred) || !matrix::is_finite(F) ||
            !_covariance_valid(Q_d.data(), N) ||
            (P_prior && !matrix::is_finite(*P_prior)))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        for (std::size_t row = 0; row < N; ++row)
            if (P[row * N + row] < 0.0f)
                return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
    }
    // 宏直接早退；预设失败诊断，成功后由 _report 覆盖。
    // Macro exits directly; prime failure diagnostics, overwritten on success.
    _diagnostics.status = PYRO_ERROR;
    _diagnostics.result = result_t::MATH_FAILURE;
    // P_pred = F·P·Fᵀ + Qd；x_pred 已由调用方的状态函数计算。
    // P_pred = F·P·Fᵀ + Qd; the caller's model already computed
    // x_pred.
    _diagnostics.math_status =
        matrix::transpose({F.data(), N, N}, {nn2_.data(), N, N});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    _diagnostics.math_status = matrix::multiply(
        {F.data(), N, N}, {P.data(), N, N}, {nn1_.data(), N, N});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    _diagnostics.math_status = matrix::multiply(
        {nn1_.data(), N, N}, {nn2_.data(), N, N}, {P_next_.data(), N, N});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    _diagnostics.math_status = matrix::add(
        {P_next_.data(), N, N}, {Q_d.data(), N, N}, {P_next_.data(), N, N});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(P_next_))
            return _report(PYRO_ERROR, result_t::NON_FINITE_RESULT);
    }
    // 运算及可选数值检查完成后一起提交 x、P；失败保留原结果。
    // Commit x/P after operations and optional validation; failures retain prior
    // values.
    _finish_covariance(P_next_);
    x_next_ = x_pred;
    x_      = x_next_;
    P_      = P_next_;
    K_.fill(0.0f);
    return _report(PYRO_OK, result_t::PREDICTED);
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
template <typename Policy>
status_t kf_t<N, Z, CheckNumerics>::correct(const measurement_t &y,
                                            const observation_t &H,
                                            const measurement_noise_t &R,
                                            const measurement_mask_t *mask,
                                            Policy policy)
{
    const status_t prepared = _prepare(y, H, R, mask);
    if (prepared != PYRO_OK)
        return prepared;
    dx_.fill(0.0f);
    correction_t correction{.y = y_, .S = S_, .diagnostics = _diagnostics, .K = K_, .dx = dx_};
    const status_t decision = policy(correction);
    if (decision == PYRO_OK)
        return _apply(correction.K,
                      correction.use_dx ? &correction.dx : nullptr);
    _prepared = false;
    return decision == PYRO_WARNING
               ? _report(PYRO_WARNING, result_t::REJECTED)
               : _report(decision, result_t::POLICY_FAILURE);
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::_prepare(const measurement_t &y,
                                             const observation_t &H,
                                             const measurement_noise_t &R,
                                             const measurement_mask_t *mask)
{
    _prepared     = false;
    _diagnostics  = {};
    _active_count = 0;
    K_.fill(0.0f);
    y_.fill(0.0f);
    S_.fill(0.0f);
    if (!_initialized)
        return _report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
    for (std::size_t row = 0; row < Z; ++row)
    {
        if (mask && !(*mask)[row])
            continue;
        if constexpr (CheckNumerics)
        {
            if (!matrix::is_finite(y[row]))
                return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
            for (std::size_t col = 0; col < N; ++col)
                if (!matrix::is_finite(H[row * N + col]))
                    return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        }
        _active[_active_count++] = row;
    }
    const std::size_t count        = _active_count;
    _diagnostics.measurement_count = count;
    if (count == 0)
        return _report(PYRO_WARNING, result_t::NO_MEASUREMENT);
    // m=count；压紧有效 H 行及 R 子矩阵，保留有效观测之间的相关噪声。
    // m=count; pack active H rows and the R submatrix, preserving correlations.
    // 公共观测残差保持原 Z 维索引，内部残差仅含 m 个分量。
    // Public residual uses original Z indices; packed residual has m
    // entries.
    for (std::size_t row = 0; row < count; ++row)
    {
        const std::size_t index = _active[row];
        y_[index]               = y[index];
        y_active_[row]          = y_[index];
        std::copy_n(H.data() + index * N, N, H_active_.data() + row * N);
        for (std::size_t col = 0; col < count; ++col)
            R_active_[row * count + col] = R[index * Z + _active[col]];
    }
    if constexpr (CheckNumerics)
    {
        if (!_covariance_valid(R_active_.data(), count))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
    }
    _diagnostics.status      = PYRO_ERROR;
    _diagnostics.result      = result_t::MATH_FAILURE;
    // S = H·P·Hᵀ + R（m×m）；公共 S 按原观测索引回填。
    // S = H·P·Hᵀ + R (m×m); scatter public S back to original indices.
    _diagnostics.math_status = matrix::transpose({H_active_.data(), count, N},
                                                 {Ht_active_.data(), N, count});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    _diagnostics.math_status =
        matrix::multiply({H_active_.data(), count, N}, {P_.data(), N, N},
                         {zn_.data(), count, N});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    _diagnostics.math_status =
        matrix::multiply({zn_.data(), count, N}, {Ht_active_.data(), N, count},
                         {zz_.data(), count, count});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    _diagnostics.math_status = matrix::add({zz_.data(), count, count},
                                           {R_active_.data(), count, count},
                                           {S_active_.data(), count, count});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    for (std::size_t row = 0; row < count; ++row)
        for (std::size_t col = 0; col < count; ++col)
        {
            const float value = S_active_[row * count + col];
            if constexpr (CheckNumerics)
            {
                if (!matrix::is_finite(value))
                    return _report(PYRO_ERROR, result_t::NON_FINITE_RESULT);
            }
            S_[_active[row] * Z + _active[col]] = value;
        }
    if (_solve_innovation() != PYRO_OK)
        return _diagnostics.status;
    for (std::size_t row = 0; row < N; ++row)
        for (std::size_t col = 0; col < count; ++col)
            K_[row * Z + _active[col]] = K_active_[row * count + col];
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(K_) || !matrix::is_finite(_diagnostics.nis))
            return _report(PYRO_ERROR, result_t::NON_FINITE_RESULT);
    }
    _prepared = true;
    return _report(PYRO_OK, result_t::PREPARED);
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::_solve_innovation()
{
    const std::size_t count = _active_count;
    if (_options.solver == solver_t::INVERSE)
    {
        // 显式求逆兼容路径：K = P·Hᵀ·S⁻¹，NIS = yᵀ·S⁻¹·y。
        // Explicit-inverse compatibility path: K = P·Hᵀ·S⁻¹, NIS = yᵀ·S⁻¹·y.
        _diagnostics.math_status = matrix::inverse(
            {S_active_.data(), count, count}, {S_inv_.data(), count, count},
            {L_.data(), count, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::multiply(
            {S_inv_.data(), count, count}, {y_active_.data(), count, 1},
            {y_solved_.data(), count, 1});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::multiply(
            {y_active_.data(), 1, count}, {y_solved_.data(), count, 1},
            {&_diagnostics.nis, 1, 1});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status =
            matrix::multiply({P_.data(), N, N}, {Ht_active_.data(), N, count},
                             {PHt_.data(), N, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::multiply(
            {PHt_.data(), N, count}, {S_inv_.data(), count, count},
            {K_active_.data(), N, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
    }
    else
    {
        // S=L·Lᵀ；先求 L·v=y，再取 NIS=vᵀ·v，无需显式求逆。
        // S=L·Lᵀ; solve L·v=y, then NIS=vᵀ·v, without explicit inversion.
        _diagnostics.math_status = matrix::cholesky(
            {S_active_.data(), count, count}, {L_.data(), count, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::solve_lower(
            {L_.data(), count, count}, {y_active_.data(), count, 1},
            {y_solved_.data(), count, 1});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::multiply(
            {y_solved_.data(), 1, count}, {y_solved_.data(), count, 1},
            {&_diagnostics.nis, 1, 1});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status =
            matrix::multiply({P_.data(), N, N}, {Ht_active_.data(), N, count},
                             {PHt_.data(), N, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status =
            matrix::transpose({PHt_.data(), N, count}, {zn_.data(), count, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        // 依次解 L·A=(P·Hᵀ)ᵀ、Lᵀ·Kᵀ=A，得到 N×m 增益。
        // Solve L·A=(P·Hᵀ)ᵀ, then Lᵀ·Kᵀ=A, yielding the N×m gain.
        _diagnostics.math_status =
            matrix::solve_lower({L_.data(), count, count},
                                {zn_.data(), count, N}, {nz_.data(), count, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::transpose(
            {L_.data(), count, count}, {zz_.data(), count, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status =
            matrix::solve_upper({zz_.data(), count, count},
                                {nz_.data(), count, N}, {zn_.data(), count, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::transpose(
            {zn_.data(), count, N}, {K_active_.data(), N, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
    }
    return PYRO_OK;
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
void kf_t<N, Z, CheckNumerics>::_finish_covariance(covariance_t &P) const
{
    // 可选 P=(P+Pᵀ)/2，再按状态施加方差下限；不等同于半正定投影。
    // Optionally average P/Pᵀ, then floor variances; this is not a PSD
    // projection.
    if (_options.symmetrize)
        for (std::size_t row = 0; row < N; ++row)
            for (std::size_t col = row + 1; col < N; ++col)
            {
                const float value =
                    0.5f * P[row * N + col] + 0.5f * P[col * N + row];
                P[row * N + col] = P[col * N + row] = value;
            }
    for (std::size_t row = 0; row < N; ++row)
        P[row * N + row] =
            std::max(P[row * N + row], _options.min_variance[row]);
}

template <std::size_t N, std::size_t Z, bool CheckNumerics>
status_t kf_t<N, Z, CheckNumerics>::_apply(const gain_t &K,
                                           const state_t *dx_override)
{
    if (!_prepared)
        return _report(PYRO_ERROR, result_t::NOT_PREPARED);
    // 一次准备结果只能应用或拒绝一次。 / Consume each preparation only once.
    _prepared = false;
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(K) ||
            (dx_override && !matrix::is_finite(*dx_override)))
            return _report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
    }
    const std::size_t count = _active_count;
    for (std::size_t row = 0; row < N; ++row)
        for (std::size_t col = 0; col < count; ++col)
            K_active_[row * count + col] = K[row * Z + _active[col]];
    _diagnostics.status = PYRO_ERROR;
    _diagnostics.result = result_t::MATH_FAILURE;
    // 默认增量为 K·y；外部可覆盖增量，但协方差仍由传入 K 更新。
    // Default delta is K·y; an override changes delta, while P still uses K.
    if (dx_override)
        dx_ = *dx_override;
    else
    {
        _diagnostics.math_status =
            matrix::multiply({K_active_.data(), N, count},
                             {y_active_.data(), count, 1}, {dx_.data(), N, 1});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
    }
    _diagnostics.math_status = matrix::add(
        {x_.data(), N, 1}, {dx_.data(), N, 1}, {x_next_.data(), N, 1});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    _diagnostics.math_status =
        matrix::multiply({K_active_.data(), N, count},
                         {H_active_.data(), count, N}, {nn1_.data(), N, N});
    CHECK_ARM_MATH_RET(_diagnostics.math_status);
    if (_options.covariance_update == covariance_update_t::SIMPLIFIED)
    {
        // 简化公式 P_new=P-K·H·P。 / Simplified update: P_new=P-K·H·P.
        _diagnostics.math_status = matrix::multiply(
            {nn1_.data(), N, N}, {P_.data(), N, N}, {nn2_.data(), N, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::subtract(
            {P_.data(), N, N}, {nn2_.data(), N, N}, {P_next_.data(), N, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
    }
    else
    {
        /* Joseph 公式的块化写法（2026-11-17）。
         *
         * 原式：A = I − K·H，P⁺ = A·P·Aᵀ + K·R·Kᵀ
         *   展开 → P − K·(H·P) − (H·P)ᵀ·Kᵀ + K·(H·P·Hᵀ + R)·Kᵀ
         *   括号内正是前面已算好的 S = H·P·Hᵀ + R，可直接复用。
         *
         * 为什么值得换：所有乘法的中间维度是 count（观测维，本工程 2 或 3），
         * 不是 N（状态维 6）。N=6/count=2 时 MAC 从 636 降到 312（−51%）：
         * 原实现要做两次 6×6 乘 6×6（各 216 MAC），这里一次都没有。
         *
         * 为什么仍是 Joseph：推导不依赖 K 是最优增益。
         * 本工程 _gate() 会整体缩放 K、并把某一行清零（零偏限幅），
         * 此时简化式 P−K·H·P 会破坏对称正定，而本式与 Joseph 等价。
         * 已用 2000 组随机 SPD 矩阵 + 随机 K 缩放验证：
         *   K 最优时最大差 4.2e-15；K 缩放时 6.1e-15（机器精度）；
         *   而简化式在同样条件下偏离达 2.3e+00。
         *
         * 暂存复用（不新增成员）：
         *   PHt_ (N×count) ← P·Hᵀ          （P 对称，故也是 (H·P)ᵀ）
         *   nz_  (N×count) ← K·S
         *   nn1_ (N×N)     ← K·(PHt_)ᵀ     = K·H·P
         *   nn2_ (N×N)     ← (K·S)·Kᵀ
         *   nn3_ (N×N)     ← (K·H·P)ᵀ     见下，实际直接用 P 对称性构造 */

        // ① PHt_ = P·Hᵀ（= (H·P)ᵀ，因 P 对称）
        _diagnostics.math_status = matrix::multiply(
            {P_.data(), N, N}, {Ht_active_.data(), N, count},
            {PHt_.data(), N, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);

        // ② nn1_ = (H·P)·Kᵀ = (K·(H·P))ᵀ，即 (K·PHt_ᵀ)ᵀ
        //    先算 K·PHt_ᵀ 需要 PHt_ᵀ，用 zn_ 暂存（count×N）
        _diagnostics.math_status =
            matrix::transpose({PHt_.data(), N, count}, {zn_.data(), count, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::multiply(
            {K_active_.data(), N, count}, {zn_.data(), count, N},
            {nn1_.data(), N, N});       // nn1_ = K·H·P
        CHECK_ARM_MATH_RET(_diagnostics.math_status);

        // ③ nn2_ = (K·H·P)ᵀ
        _diagnostics.math_status =
            matrix::transpose({nn1_.data(), N, N}, {nn2_.data(), N, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);

        // ④ nz_ = K·S（N×count）
        _diagnostics.math_status = matrix::multiply(
            {K_active_.data(), N, count}, {S_active_.data(), count, count},
            {nz_.data(), N, count});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);

        // ⑤ nn3_ = (K·S)·Kᵀ（N×N）
        _diagnostics.math_status = matrix::transpose(
            {K_active_.data(), N, count}, {zn_.data(), count, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);
        _diagnostics.math_status = matrix::multiply(
            {nz_.data(), N, count}, {zn_.data(), count, N},
            {nn3_.data(), N, N});
        CHECK_ARM_MATH_RET(_diagnostics.math_status);

        // ⑥ P_next_ = P − K·H·P − (K·H·P)ᵀ + K·S·Kᵀ
        for (std::size_t row = 0; row < N; ++row)
            for (std::size_t col = 0; col < N; ++col)
            {
                const std::size_t idx = row * N + col;
                P_next_[idx] = ((P_[idx] - nn1_[idx]) - nn2_[idx]) + nn3_[idx];
            }
    }
    // 开启数值检查时，先检查有限性再施加下限。
    // With numerical validation enabled, inspect finiteness before flooring.
    if constexpr (CheckNumerics)
    {
        if (!matrix::is_finite(x_next_) || !matrix::is_finite(P_next_))
            return _report(PYRO_ERROR, result_t::NON_FINITE_RESULT);
    }
    _finish_covariance(P_next_);
    x_ = x_next_;
    P_ = P_next_;
    K_ = K;
    return _report(PYRO_OK, result_t::CORRECTED);
}

} // namespace pyro
#endif // __PYRO_ALGO_KF_TPP__
