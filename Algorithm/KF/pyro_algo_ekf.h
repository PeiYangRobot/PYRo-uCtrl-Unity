/**
 * @file pyro_algo_ekf.h
 * @brief 模型无关的 EKF：在通用 KF 接口上适配非线性观测。
 *        Model-independent EKF adapting nonlinear observations to the KF core.
 */
#ifndef PYRO_ALGO_EKF_H
#define PYRO_ALGO_EKF_H

#include "pyro_algo_kf.h"

namespace pyro
{

/**
 * @brief 通用扩展卡尔曼滤波器，不包含 IMU、四元数或其他应用模型。
 *        Generic EKF without IMU, quaternion or other application models.
 * @tparam N 状态维度 / State dimension.
 * @tparam Z 最大观测维度 / Maximum observation dimension.
 * @tparam CheckNumerics 数值检查，默认关闭 / Numeric validation, off by default.
 * @details 外部计算 x_pred=f(x,u)、F=∂f/∂x、z_pred=h(x)、H=∂h/∂x。
 *          Caller supplies x_pred=f(x,u), F=∂f/∂x, z_pred=h(x), H=∂h/∂x.
 *          predict 沿用核心接口；correct 构造观测残差后调用相同核心接口。
 *          predict uses the common API; correct adapts residual to that API.
 *          姿态归一化、噪声调度和门控属于外部模型/应用。
 *          Normalization, noise scheduling and gating belong to the
 * application.
 */
template <std::size_t N, std::size_t Z, bool CheckNumerics = false>
class ekf_t : public kf_t<N, Z, CheckNumerics>
{
    using base_t = kf_t<N, Z, CheckNumerics>;

  public:
    using typename base_t::measurement_mask_t;
    using typename base_t::measurement_noise_t;
    using typename base_t::measurement_t;
    using typename base_t::observation_t;
    using typename base_t::result_t;

    /**
     * @brief 计算观测残差（新息）y=z-z_pred 后调用通用 correct(y,H,R)。
     *        Form measurement residual (innovation) y=z-z_pred, then correct.
     * @param z 实际量测，Z×1；开启数值检查时非有限行自动作为缺测。
     *          Measured observation, Z×1; non-finite rows missing in checked mode.
     * @param z_pred 非线性 h(x)，Z×1；不能用 H*x 代替。
     *               Nonlinear h(x), Z×1, not replaced by H*x.
     * @param H 当前观测雅可比，Z×N / Current observation Jacobian, Z×N.
     * @param R 观测噪声协方差，Z×Z / Measurement covariance, Z×Z.
     * @param mask 可选有效行掩码 / Optional active-row mask.
     * @param policy 通用修正策略，含义同核心 correct。
     *               Common correction policy, as in core correct.
     */
    template <typename Policy = typename base_t::accept_t>
    status_t correct(const measurement_t &z, const measurement_t &z_pred,
                     const observation_t &H, const measurement_noise_t &R,
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
                if (!matrix::is_finite(z_pred[row]))
                    return this->_report(PYRO_PARAM_ERROR,
                                         result_t::INVALID_INPUT);
            }
            y[row] = z[row] - z_pred[row];
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
#endif // PYRO_ALGO_EKF_H
