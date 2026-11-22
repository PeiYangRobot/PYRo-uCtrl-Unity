/**
 * @file pyro_algo_lkf.h
 * @brief 通用 KF 核心的可选线性模型适配器。
 *        Optional linear-model adapter over the common KF core.
 */
#ifndef PYRO_ALGO_LKF_H
#define PYRO_ALGO_LKF_H

#include "pyro_algo_kf.h"

namespace pyro
{

/**
 * @brief 在线性模型层计算 F*x+B*u 和 z-H*x；数值核心使用统一接口。
 *        Compute F*x+B*u and z-H*x in the linear layer, using the common core.
 * @tparam N 状态维度 / State dimension.
 * @tparam Z 最大观测维度 / Maximum observation dimension.
 * @tparam U 控制维度，0 表示无控制 / Control dimension; 0 means no control.
 * @tparam CheckNumerics 数值检查，默认关闭 / Numeric validation, off by default.
 */
template <std::size_t N, std::size_t Z, std::size_t U = 0,
          bool CheckNumerics = false>
class lkf_t : public kf_t<N, Z, CheckNumerics>
{
    using base_t = kf_t<N, Z, CheckNumerics>;
    static_assert(U <= 65535, "Invalid CMSIS control dimension");

  public:
    using typename base_t::covariance_t;
    using typename base_t::measurement_mask_t;
    using typename base_t::measurement_noise_t;
    using typename base_t::measurement_t;
    using typename base_t::observation_t;
    using typename base_t::result_t;
    using typename base_t::state_t;
    using typename base_t::transition_t;
    using control_t = vector_t<U>; ///< 控制向量 u，U×1 / Control vector.
    using control_matrix_t = matrix_t<N, U>; ///< B，N×U / Control matrix.

    /// 线性预测 x_pred=F*x。 / Linear prediction x_pred=F*x.
    status_t predict(const transition_t &F, const covariance_t &Q_d)
    {
        if (!this->is_initialized())
            return this->_report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
        if constexpr (CheckNumerics)
        {
            if (!matrix::is_finite(F))
                return this->_report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        }
        const state_t x_pred = F * this->get_state();
        if constexpr (CheckNumerics)
        {
            if (!matrix::is_finite(x_pred))
                return this->_report(PYRO_ERROR, result_t::NON_FINITE_RESULT);
        }
        return base_t::predict(x_pred, F, Q_d);
    }

    /// 带控制预测 x_pred=F*x+B*u，要求 U>0。 / Controlled prediction, U>0.
    status_t predict(const transition_t &F, const covariance_t &Q_d,
                     const control_matrix_t &B, const control_t &u)
    {
        static_assert(U > 0, "Use predict(F,Q_d) when U is zero");
        if (!this->is_initialized())
            return this->_report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
        if constexpr (CheckNumerics)
        {
            if (!matrix::is_finite(F) || !matrix::is_finite(B) ||
                !matrix::is_finite(u))
                return this->_report(PYRO_PARAM_ERROR, result_t::INVALID_INPUT);
        }
        const state_t x_pred = F * this->get_state() + B * u;
        if constexpr (CheckNumerics)
        {
            if (!matrix::is_finite(x_pred))
                return this->_report(PYRO_ERROR, result_t::NON_FINITE_RESULT);
        }
        return base_t::predict(x_pred, F, Q_d);
    }

    /// 计算观测残差（新息）y=z-H*x 后修正；缺测规则同 EKF。
    /// Correct using measurement residual (innovation) y=z-H*x.
    /// Missing-measurement rules match EKF.
    template <typename Policy = typename base_t::accept_t>
    status_t correct(const measurement_t &z, const observation_t &H,
                     const measurement_noise_t &R,
                     const measurement_mask_t *mask = nullptr,
                     Policy policy                  = Policy{})
    {
        if (!this->is_initialized())
            return this->_report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
        measurement_t y{};
        measurement_mask_t active{};
        for (std::size_t row = 0; row < Z; ++row)
        {
            active[row] = !mask || (*mask)[row];
            if constexpr (CheckNumerics)
                active[row] = active[row] && matrix::is_finite(z[row]);
            if (!active[row])
                continue;
            if constexpr (CheckNumerics)
            {
                for (std::size_t col = 0; col < N; ++col)
                    if (!matrix::is_finite(H[row * N + col]))
                        return this->_report(PYRO_PARAM_ERROR,
                                             result_t::INVALID_INPUT);
            }
            float z_pred                 = 0.0f;
            const arm_status math_status = matrix::multiply(
                {H.data() + row * N, 1, N}, {this->get_state().data(), N, 1},
                {&z_pred, 1, 1});
            // 记录底层状态；宏早退时保留失败诊断。
            // Record backend status so macro early returns preserve diagnostics.
            this->_report(PYRO_ERROR, result_t::MATH_FAILURE, math_status);
            CHECK_ARM_MATH_RET(math_status);
            y[row] = z[row] - z_pred;
            if constexpr (CheckNumerics)
            {
                if (!matrix::is_finite(y[row]))
                    return this->_report(PYRO_ERROR,
                                         result_t::NON_FINITE_RESULT);
            }
        }
        return base_t::correct(y, H, R, &active, policy);
    }
};

} // namespace pyro
#endif // PYRO_ALGO_LKF_H
