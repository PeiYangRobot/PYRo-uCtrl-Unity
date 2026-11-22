// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
/**
 * @file pyro_algo_butterworth.h
 * @brief 多通道二阶 Butterworth 低通 / Multi-channel second-order low-pass.
 * Continuous-time Butterworth with variable-step state propagation. See ../VQF/LICENSE.
 */
#ifndef PYRO_ALGO_BUTTERWORTH_H
#define PYRO_ALGO_BUTTERWORTH_H

#pragma push_macro("PI")
#undef PI
#include "pyro_core_def.h"
#pragma pop_macro("PI")
#include <array>
#include <cmath>
#include <cstddef>

namespace pyro
{

/**
 * @brief 固定通道数、固定存储的二阶低通，支持输入输出同址。
 *        Fixed-storage second-order filter with in-place multichannel processing.
 * @tparam Channels 通道数 / Channel count.
 * tau/dt 单位为秒，fc=sqrt(2)/(2*pi*tau)。
 * 时间不等间隔时，用二阶连续系统的精确状态转移，区间内输入按当前样本保持。
 * Exact continuous-state propagation under a held current input supports variable dt.
 * 启动期间输出前 tau 秒的时间加权均值，随后从稳态开始递推。
 * Startup uses a time-weighted mean, followed by steady-state initialization.
 * double 系数/状态保留高采样率、低截止频率下的递推精度；样本为 float。
 * 输入由调用方保证有限，成功 process 消耗一个样本。
 * Caller supplies finite inputs; each successful process consumes one sample.
 */
template <std::size_t Channels> class butterworth_lpf_t
{
    static_assert(Channels > 0, "At least one filter channel is required");

  public:
    /* 观测用：系数重算次数（含 exp/cos/sin）。稳态应几乎不增长；
     * 若持续增长说明 dt 在抖，VQF 会被 double 超越函数拖慢。
     * 多实例累加，仅供调试观察，不影响功能。 */
    inline static volatile uint32_t coeff_recalc_count = 0U;

    /** @brief 修改时间常数并保持输出连续，失败保留配置和状态。
     *         Set tau while preserving the output and its derivative.
     */
    status_t configure(float tau)
    {
        if (!std::isfinite(tau) || tau <= 0.0f)
            return PYRO_PARAM_ERROR;
        if (_configured && !_averaging)
        {
            const double ratio = static_cast<double>(tau) / static_cast<double>(_tau);
            for (auto &state : _state)
                state[1] *= ratio; // 第二状态为 tau * dy/dt，保持物理导数。
        }
        _tau           = tau;
        _cached_period = -1.0f;
        _configured    = true;
        return PYRO_OK;
    }

    void reset()
    {
        _state       = {};
        _sum         = {};
        _elapsed     = 0.0;
        _averaging   = true;
        _primed      = false;
    }

    status_t process(const float *input, float *output, float dt)
    {
        if (!_configured)
            return PYRO_NOT_FOUND;
        if (!input || !output || !std::isfinite(dt) || dt < 0.0f)
            return PYRO_PARAM_ERROR;

        if (dt == 0.0f)
        {
            for (std::size_t i = 0; i < Channels; ++i)
            {
                if (!_primed)
                    _state[i] = {input[i], 0.0};
                output[i] = static_cast<float>(_state[i][0]);
            }
            _primed = true;
            return PYRO_OK;
        }

        _primed = true;
        const double period = static_cast<double>(dt);
        const double tau = static_cast<double>(_tau);
        if (_averaging)
        {
            _elapsed += period;
            for (std::size_t i = 0; i < Channels; ++i)
            {
                _sum[i] += static_cast<double>(input[i]) * period;
                _state[i] = {_sum[i] / _elapsed, 0.0};
                output[i] = static_cast<float>(_state[i][0]);
            }
            if (_elapsed >= tau)
                _averaging = false;
            return PYRO_OK;
        }

        /* dt 缓存：变步长下只在 dt 改变时重算系数（含 exp/cos/sin，M4F 无双精度
         * FPU 时每次约 20~40 us）。用相对容差替代精确相等：dt 由
         * ticks * timestamp_tick_s 算出，理论上恒定，但采样抖动会让末位差 1 ULP，
         * 精确相等会把缓存打穿、退化到每样本都重算。 */
        const bool period_changed =
            (_cached_period <= 0.0f) ||
            (std::fabs(dt - _cached_period) > (1.0e-4f * _cached_period));
        if (period_changed)
        {
            const double ratio = period / tau;
            const double decay = std::exp(-ratio);
            _decay_cos = decay * std::cos(ratio);
            _decay_sin = decay * std::sin(ratio);
            _cached_period = dt;
            ++coeff_recalc_count; /* 观测：稳态应几乎不增长 */
        }
        // y'' + (2/tau)y' + (2/tau²)y = (2/tau²)x。
        // 状态为 [y, tau*y']，dt 变化保持状态连续，长间隔也稳定。
        for (std::size_t i = 0; i < Channels; ++i)
        {
            const double x = input[i];
            const double offset = _state[i][0] - x;
            const double velocity = _state[i][1];
            _state[i][0] = x + (_decay_cos + _decay_sin) * offset + _decay_sin * velocity;
            _state[i][1] = -2.0 * _decay_sin * offset + (_decay_cos - _decay_sin) * velocity;
            output[i] = static_cast<float>(_state[i][0]);
        }
        return PYRO_OK;
    }

  private:
    std::array<std::array<double, 2>, Channels> _state{};
    std::array<double, Channels> _sum{};
    double _decay_cos = 0.0;
    double _decay_sin = 0.0;
    double _elapsed      = 0.0;
    float  _tau          = 0.0f;
    float  _cached_period = -1.0f;
    bool   _configured   = false;
    bool   _averaging    = true;
    bool   _primed       = false;
};

} // namespace pyro
#endif
