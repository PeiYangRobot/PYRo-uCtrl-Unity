/**
 * @file pyro_algo_eskf.h
 * @brief 模型无关的 ESKF：在通用 KF 接口上适配误差状态注入。
 *        Model-independent ESKF adapting error-state injection to the KF core.
 */
#ifndef PYRO_ALGO_ESKF_H
#define PYRO_ALGO_ESKF_H

#include "pyro_algo_kf.h"

namespace pyro
{

/**
 * @brief 通用误差状态卡尔曼滤波器，不包含 IMU、四元数或其他应用模型。
 *        Generic ESKF without IMU, quaternion or other application models.
 * @tparam N 误差状态维度 / Error-state dimension.
 * @tparam Z 最大观测维度 / Maximum observation dimension.
 * @tparam CheckNumerics 数值检查，默认关闭 / Numeric validation, off by default.
 * @details 与 ekf_t 的分工：EKF 把非线性观测线性化后交给 kf_t；ESKF 在此之上
 *          补上「误差状态必须注入回名义状态」这一环——correct 除更新协方差外，
 *          还把本次实际施加的误差增量交给调用方，由模型自行注入姿态/零偏。
 *          EKF linearizes the observation; ESKF additionally hands the error
 *          delta back so the caller can inject it into the nominal state.
 *          姿态积分、雅可比、块结构与噪声调度属于外部模型/应用。
 *          Attitude integration, Jacobians, block structure and noise
 *          scheduling belong to the application.
 */
template <std::size_t N, std::size_t Z, bool CheckNumerics = false>
class eskf_t : public kf_t<N, Z, CheckNumerics>
{
    using base_t = kf_t<N, Z, CheckNumerics>;

  public:
    using typename base_t::covariance_t;
    using typename base_t::measurement_mask_t;
    using typename base_t::measurement_noise_t;
    using typename base_t::measurement_t;
    using typename base_t::observation_t;
    using typename base_t::result_t;
    using typename base_t::state_t;

    /**
     * @brief 提交一次观测，并回传本次实际施加的误差增量。
     *        Correct, returning the error delta that was applied.
     * @param y 观测残差（新息），Z×1 / Measurement residual (innovation).
     * @param H 观测矩阵/雅可比，Z×N / Observation matrix or Jacobian.
     * @param R 观测噪声，Z×Z / Measurement covariance.
     * @param dx 出参：本次实际施加的误差增量，N×1。
     *           Output: the delta actually applied, N×1.
     * @param mask 可选有效行掩码 / Optional active-row mask.
     * @param policy 通用修正策略（可改 K），含义同核心 correct。
     *               Common correction policy (may adjust K), as in core correct.
     * @details 与 ekf_t::correct 的唯一区别是多了 dx 出参。协方差仍由核心按
     *          最终 K 更新（Joseph），本类不重复计算，只把同一份 K*y 交出去，
     *          保证「协方差用的增量」与「注入名义状态的增量」严格一致。
     *          Only difference from ekf_t::correct is the dx output.
     */
    template <typename Policy = typename base_t::accept_t>
    status_t correct(const measurement_t &y, const observation_t &H,
                     const measurement_noise_t &R, state_t &dx,
                     const measurement_mask_t *mask = nullptr,
                     Policy policy                  = Policy{})
    {
        if (!this->is_initialized())
            return this->_report(PYRO_NOT_FOUND, result_t::NOT_INITIALIZED);
        const status_t result = base_t::correct(y, H, R, mask, policy);
        if (result != PYRO_OK)
        {
            dx.fill(0.0f);
            return result;
        }
        /* 核心已用最终 K 提交 x_；同一份增量交给调用方注入名义状态。 */
        dx = this->get_state();
        return PYRO_OK;
    }

    /**
     * @brief 注入完成后重置：误差状态归零，协方差替换为模型算好的 P_new。
     *        Post-injection reset: zero the delta, install the new covariance.
     * @param P_new 注入后重新算出的协方差（通常为 G*P*G^T），N×N。
     * @details ESKF 把增量注入名义状态后，误差状态回到零，协方差需被注入
     *          雅可比夹乘后的结果替换。模型侧算好 P_new 后由此提交。
     *          After injection the error state returns to zero; the covariance
     *          is replaced by the Jacobian-sandwiched result.
     */
    status_t reset_after_inject(const covariance_t &P_new)
    {
        return base_t::reset(state_t{}, P_new);
    }
};

} // namespace pyro
#endif // PYRO_ALGO_ESKF_H
