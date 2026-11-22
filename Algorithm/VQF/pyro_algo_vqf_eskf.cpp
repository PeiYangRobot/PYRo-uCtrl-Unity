// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
#include "pyro_algo_vqf_eskf.h"
#include "imu_base_drv.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace pyro
{
namespace
{
constexpr float deg_to_rad = 0.01745329251994329577f;
constexpr float gravity = 9.80665f;
constexpr float vibration_tau = 0.05f;
constexpr float min_acc_confidence = 0.2f;
constexpr float max_correction_angle = 0.1f;

float square(float value) { return value * value; }

float norm(const vector_t<3> &value)
{
    return sqrtf(square(value[0]) + square(value[1]) + square(value[2]));
}

vqf_eskf_t::quaternion_t multiply(const vqf_eskf_t::quaternion_t &a, const vqf_eskf_t::quaternion_t &b)
{
    vqf_eskf_t::quaternion_t result;
    arm_quaternion_product_single_f32(a.data(), b.data(), result.data());
    return result;
}

vqf_eskf_t::quaternion_t normalized(const vqf_eskf_t::quaternion_t &q)
{
    vqf_eskf_t::quaternion_t result;
    arm_quaternion_normalize_f32(q.data(), result.data(), 1);
    return result;
}

vector_t<3> rotate(const vqf_eskf_t::quaternion_t &q, const vector_t<3> &v)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float tx = 2.0f * (y * v[2] - z * v[1]);
    const float ty = 2.0f * (z * v[0] - x * v[2]);
    const float tz = 2.0f * (x * v[1] - y * v[0]);
    return {v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx};
}

template <std::size_t N> matrix_t<N, N> identity(float scale = 1.0f)
{
    matrix_t<N, N> result;
    for (std::size_t i = 0; i < N; ++i)
        result(i, i) = scale;
    return result;
}

/* 角增量 → 旋转四元数。可选输出半角 sin/cos 供 _compute_left_jacobian 复用，
 * 免去它在同一角度上再算一次 sinf/cosf（见 .h 里 _trig_* 的说明）。 */
vqf_eskf_t::quaternion_t rotation_step(const vector_t<3> &angle,
                                  float *out_sin_h = nullptr,
                                  float *out_cos_h = nullptr)
{
    const float magnitude = norm(angle);
    if (magnitude < 1.0e-6f)
    {
        if (out_sin_h != nullptr) { *out_sin_h = 0.5f * magnitude; }
        if (out_cos_h != nullptr) { *out_cos_h = 1.0f; }
        return normalized({1.0f, 0.5f * angle[0], 0.5f * angle[1], 0.5f * angle[2]});
    }
    const float half      = 0.5f * magnitude;
    const float sin_h     = sinf(half);
    const float cos_h     = cosf(half);
    if (out_sin_h != nullptr) { *out_sin_h = sin_h; }
    if (out_cos_h != nullptr) { *out_cos_h = cos_h; }
    const float scale = sin_h / magnitude;
    return {cos_h, scale * angle[0], scale * angle[1], scale * angle[2]};
}

// 将单位方向转向 +Z 的最短旋转向量，取水平两维作为倾斜新息。
vector_t<3> inclination_error(const vector_t<3> &direction)
{
    const float horizontal = sqrtf(square(direction[0]) + square(direction[1]));
    const float angle = atan2f(horizontal, direction[2]);
    if (horizontal < 1.0e-6f)
        return direction[2] < 0.0f ? vector_t<3>{3.14159265358979323846f, 0.0f, 0.0f}
                                   : vector_t<3>{direction[1], -direction[0], 0.0f};
    return {angle * direction[1] / horizontal, -angle * direction[0] / horizontal, 0.0f};
}

template <typename Sample> bool valid_stream(const sample_stream_t<Sample> &stream)
{
    /* SPSC 下行：索引只保证在 [0, capacity)，不变量由流自己维护。
     * capacity 必须是 >= 2 的 2 的幂：掩码用 capacity-1，且需留一个哨兵槽
     * 判别满，所以 capacity==1 会让掩码恒为 0、available() 永远读不到样本。 */
    return stream.available() == 0 ||
        (stream.sample != nullptr && stream.capacity >= 2U &&
         (stream.capacity & (stream.capacity - 1U)) == 0U);
}
} // namespace

status_t vqf_eskf_t::configure(const config_t &config)
{
    const float positive[] = {
        config.tau_acc, config.bias_sigma_init, config.bias_forgetting_time,
        config.bias_clip, config.bias_sigma_rest, config.rest_filter_tau,
        config.rest_th_gyr, config.rest_th_acc, config.attitude_sigma_init,
        config.gyro_noise_density, config.acc_noise, config.acc_norm_tolerance,
        config.acc_vibration_tolerance, config.acc_update_period, config.acc_nis_threshold};
    for (float value : positive)
        if (!matrix::is_finite(value) || value <= 0.0f ||
            !matrix::is_finite(square(value)) || square(value) == 0.0f)
            return PYRO_PARAM_ERROR;
    if (!matrix::is_finite(config.rest_min_t) || config.rest_min_t < 0.0f ||
        !matrix::is_finite(config.acc_recovery_t) || config.acc_recovery_t < 0.0f ||
        !matrix::is_finite(config.max_sample_gap_s) || config.max_sample_gap_s < 0.0f)
        return PYRO_PARAM_ERROR;
    const float derived[] = {
        square(config.bias_sigma_init * deg_to_rad),
        square(config.attitude_sigma_init * deg_to_rad),
        square(config.gyro_noise_density * deg_to_rad),
        square(config.bias_sigma_rest * deg_to_rad),
        square(0.1f * deg_to_rad) / config.bias_forgetting_time,
        square(config.acc_noise / gravity)};
    for (float value : derived)
        if (!matrix::is_finite(value) || value <= 0.0f)
            return PYRO_PARAM_ERROR;

    const bool first = !_error_kf.is_initialized();
    if (first)
    {
        error_kf_t::covariance_t P;
        for (std::size_t i = 0; i < 3; ++i)
        {
            P(i, i) = derived[1];
            P(i + 3, i + 3) = derived[0];
        }
        error_kf_t::options_t options;
        for (std::size_t i = 0; i < 3; ++i)
        {
            options.min_variance[i] = 1.0e-12f;
            options.min_variance[i + 3] = 1.0e-14f;
        }
        const status_t result = _error_kf.init({}, P, options);
        if (result != PYRO_OK)
            return result;
    }
    /* 参数"变化"检测只在已初始化后有意义：首次 configure 时 _config 是默认构造值，
     * 与调用方配置的差异并不是"热更新"，此时清空低通/可信历史毫无意义，
     * 还会让 reset() 之后本该保留的启动时序看起来像被参数改动打断。
     * （首次配置走下面的 reset()，历史本来就是空的。） */
    const bool rest_changed = !first && (config.rest_bias_est_enabled != _config.rest_bias_est_enabled ||
        config.rest_filter_tau != _config.rest_filter_tau || config.rest_min_t != _config.rest_min_t ||
        config.rest_th_gyr != _config.rest_th_gyr || config.rest_th_acc != _config.rest_th_acc ||
        config.bias_clip != _config.bias_clip);
    const bool acc_changed = !first && (config.tau_acc != _config.tau_acc ||
        config.acc_noise != _config.acc_noise || config.acc_norm_tolerance != _config.acc_norm_tolerance ||
        config.acc_vibration_tolerance != _config.acc_vibration_tolerance ||
        config.acc_recovery_t != _config.acc_recovery_t ||
        config.acc_nis_threshold != _config.acc_nis_threshold);
    if (rest_changed)
    {
        _rest_gyr_lp.reset();
        _rest_acc_lp.reset();
        _state.rest_gyr_lp = {};
        _state.rest_acc_lp = {};
        _state.rest_t = 0.0f;
        _state.rest_gyr_ok = _state.rest_detected = false;
    }
    if (acc_changed)
    {
        _state.trusted_acc_t = _state.observation_t = 0.0f;
        _acc_lp.reset();
    }
    (void)_acc_lp.configure(config.tau_acc);
    (void)_rest_gyr_lp.configure(config.rest_filter_tau);
    (void)_rest_acc_lp.configure(config.rest_filter_tau);
    _config = config;
    if (first)
        return reset();
    const float clip = _config.bias_clip * deg_to_rad;
    for (float &value : _state.bias)
        value = std::clamp(value, -clip, clip);
    return PYRO_OK;
}

status_t vqf_eskf_t::reset(const vector3_t &bias, float sigma)
{
    if (!_error_kf.is_initialized())
        return PYRO_NOT_FOUND;
    if (!matrix::is_finite(bias) || !matrix::is_finite(sigma) || sigma < 0.0f ||
        !matrix::is_finite(square(sigma)))
        return PYRO_PARAM_ERROR;
    const float clip = _config.bias_clip * deg_to_rad;
    for (float value : bias)
        if (std::fabs(value) > clip)
            return PYRO_PARAM_ERROR;
    error_kf_t::covariance_t P;
    for (std::size_t i = 0; i < 3; ++i)
    {
        P(i, i) = square(_config.attitude_sigma_init * deg_to_rad);
        P(i + 3, i + 3) = sigma > 0.0f ? square(sigma) : square(_config.bias_sigma_init * deg_to_rad);
    }
    const status_t result = _error_kf.reset({}, P);
    if (result != PYRO_OK)
        return result;
    _state = {};
    _state.bias = bias;
    _diagnostics = {};
    _acc_lp.reset();
    _rest_gyr_lp.reset();
    _rest_acc_lp.reset();
    err_stage = 0U;
    return PYRO_OK;
}

status_t vqf_eskf_t::update(const imu_data_t &data)
{
    err_stage = 0U;
    _state.consumed_gyr = 0;
    _state.consumed_acc = 0;
    _state.updated_gyr = 0;
    _state.updated_acc = 0;
    if (!_error_kf.is_initialized())
        return PYRO_NOT_FOUND;
    if (!valid_stream(data.gyro) || !valid_stream(data.acc))
    {
        err_stage = 1;
        return PYRO_PARAM_ERROR;
    }
    /* SPSC：本批要处理的样本数在批首取一次快照。
     * 不在循环里重读 available()——那样批次边界会随生产者推进而变长，
     * 也让 consumed_* 的语义（= 本批归还量）与"处理到哪"脱节。 */
    const uint16_t gyr_batch = data.gyro.available();
    const uint16_t acc_batch = data.acc.available();
    if (gyr_batch == 0U && acc_batch == 0U)
        return PYRO_OK;
    if (!std::isfinite(data.timestamp_tick_s) || data.timestamp_tick_s <= 0.0f)
    {
        err_stage = 2;
        return PYRO_PARAM_ERROR;
    }
    if (_state.timestamp_tick_s != 0.0f && _state.timestamp_tick_s != data.timestamp_tick_s)
    {
        err_stage = 3;
        return PYRO_PARAM_ERROR;
    }

    uint16_t &gyr_index = _state.consumed_gyr;
    uint16_t &acc_index = _state.consumed_acc;
    status_t batch_result = PYRO_OK;
    while (gyr_index < gyr_batch || acc_index < acc_batch)
    {
        // 先检查有效帧，再短路比较时间戳；差值为零时优先陀螺仪。
        // Check bounds before timestamp reads; zero delta selects the gyro.
        const bool gyro_first = gyr_index < gyr_batch &&
            (acc_index >= acc_batch ||
             uint32_t(data.acc.at(acc_index).timestamp -
                      data.gyro.at(gyr_index).timestamp) < (uint32_t{1} << 31));
        const uint32_t timestamp = gyro_first
            ? data.gyro.at(gyr_index).timestamp
            : data.acc.at(acc_index).timestamp;
        sample_clock_t &clock = gyro_first ? _state.gyr_clock : _state.acc_clock;
        const uint32_t ticks = timestamp - clock.timestamp;
        if (clock.initialized && ticks >= (uint32_t{1} << 31))
        {
            // 丢弃坏样本，但保留后续样本，避免队头永远阻塞。
            ++(gyro_first ? gyr_index : acc_index);
            err_stage = 4;
            return PYRO_PARAM_ERROR;
        }
        const float dt = clock.initialized ? static_cast<float>(ticks) * data.timestamp_tick_s : 0.0f;
        if (!std::isfinite(dt) || (clock.initialized && ticks > 0 && dt <= 0.0f))
        {
            err_stage = 5;
            return PYRO_PARAM_ERROR;
        }
        if (_config.max_sample_gap_s > 0.0f && dt > _config.max_sample_gap_s)
        {
            err_stage = 6;
            return PYRO_PARAM_ERROR;
        }
        const bool duplicate = clock.initialized && ticks == 0;
        status_t result = PYRO_WARNING;
        if (gyro_first)
        {
            const auto &gyr = data.gyro.at(gyr_index).Gyro;
            if (duplicate && (!std::isfinite(gyr[0]) || !std::isfinite(gyr[1]) || !std::isfinite(gyr[2])))
                return PYRO_PARAM_ERROR;
            if (!duplicate)
                result = _update_gyr({gyr[0], gyr[1], gyr[2]}, dt);
            if (result != PYRO_OK && result != PYRO_WARNING)
                return result;
            ++gyr_index;
        }
        else
        {
            const auto &sample = data.acc.at(acc_index).Accel;
            const vector3_t acc{sample[0], sample[1], sample[2]};
            if (!std::isfinite(norm(acc)))
                return PYRO_PARAM_ERROR;
            if (!duplicate)
            {
                result = _update_acc(acc, dt);
            }
            ++acc_index;
        }
        if (!duplicate && (result == PYRO_OK || result == PYRO_WARNING))
        {
            (gyro_first ? _state.updated_gyr : _state.updated_acc) =
                gyro_first ? gyr_index : acc_index;
        }
        clock = {timestamp, true};
        _state.timestamp_tick_s = data.timestamp_tick_s;
        if (result == PYRO_WARNING)
            batch_result = PYRO_WARNING;
        else if (result != PYRO_OK)
            return result;
    }
    return batch_result;
}

/* 6×6 → 3×3 子块读/写（纯拷贝，无浮点运算）。 */
void vqf_eskf_t::_get_block(const matrix_t<6, 6> &src, std::size_t row0, std::size_t col0,
                       matrix_t<3, 3> &dst)
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 3; ++col)
            dst(row, col) = src(row0 + row, col0 + col);
}

void vqf_eskf_t::_set_block(matrix_t<6, 6> &dst, std::size_t row0, std::size_t col0,
                       const matrix_t<3, 3> &src)
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 3; ++col)
            dst(row0 + row, col0 + col) = src(row, col);
}

/* 取当前 P 的四个 3×3 块。P 对称，因此 P10 = P01ᵀ（转置是纯拷贝，无舍入）。 */
status_t vqf_eskf_t::_load_p_blocks()
{
    const error_kf_t::covariance_t &P = _error_kf.get_covariance();
    _get_block(P, 0, 0, _p00);
    _get_block(P, 0, 3, _p01);
    _get_block(P, 3, 3, _p11);
    if (matrix::transpose(matrix::view(_p01), matrix::view(_p10)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    return PYRO_OK;
}

/* P⁻ = F·P·Fᵀ，误差状态转移 F = [[I, M], [0, I]]：
 *   F·P      = [[P00 + M·P10, P01 + M·P11], [P10, P11]]
 *   (F·P)·Fᵀ = [[X00 + X01·Mᵀ, X01], [X10 + X11·Mᵀ, X11]]
 * 又因 P 对称 ⇒ 结果对称 ⇒ X10 = X01ᵀ，于是只需 3 次 3×3 乘法：
 *   X01 = P01 + M·P11,  r00 = P00 + M·P10 + X01·Mᵀ,  r01 = X01,
 *   r11 = P11,          r10 = X01ᵀ
 * MAC 数 81，通用 6×6 路径是 432（2 次乘法）＋一次 6×6 转置。
 * 结果与通用路径只差浮点分组，后续 predict_covariance 会对称化并加 Q。 */
status_t vqf_eskf_t::_propagate_covariance()
{
    if (_load_p_blocks() != PYRO_OK)
        return PYRO_ERROR;
    if (matrix::transpose(matrix::view(_m), matrix::view(_m_t)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    if (matrix::multiply(matrix::view(_m), matrix::view(_p11), matrix::view(_t1)) !=
        ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    for (std::size_t i = 0; i < 9; ++i)
        _t2.data()[i] = _p01.data()[i] + _t1.data()[i]; /* X01 */
    if (matrix::multiply(matrix::view(_m), matrix::view(_p10), matrix::view(_t1)) !=
            ARM_MATH_SUCCESS ||
        matrix::multiply(matrix::view(_t2), matrix::view(_m_t), matrix::view(_t3)) !=
            ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    for (std::size_t i = 0; i < 9; ++i)
        _p00.data()[i] = (_p00.data()[i] + _t1.data()[i]) + _t3.data()[i]; /* r00 */
    if (matrix::transpose(matrix::view(_t2), matrix::view(_p10)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR; /* r10 */
    _set_block(_p_prop, 0, 0, _p00);
    _set_block(_p_prop, 0, 3, _t2);
    _set_block(_p_prop, 3, 0, _p10);
    _set_block(_p_prop, 3, 3, _p11);
    return PYRO_OK;
}

/* G·P·Gᵀ，误差注入 G = [[J, 0], [0, I]]：
 *   (0,0) = J·P00·Jᵀ, (0,1) = J·P01, (1,0) = (J·P01)ᵀ, (1,1) = P11
 * 3 次 3×3 乘法（通用路径是 2 次 6×6 + 一次转置）。 */
status_t vqf_eskf_t::_rotate_covariance()
{
    if (_load_p_blocks() != PYRO_OK)
        return PYRO_ERROR;
    if (matrix::transpose(matrix::view(_jacobian), matrix::view(_m_t)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    if (matrix::multiply(matrix::view(_jacobian), matrix::view(_p00), matrix::view(_t1)) !=
            ARM_MATH_SUCCESS ||
        matrix::multiply(matrix::view(_t1), matrix::view(_m_t), matrix::view(_t2)) !=
            ARM_MATH_SUCCESS ||
        matrix::multiply(matrix::view(_jacobian), matrix::view(_p01), matrix::view(_t3)) !=
            ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    if (matrix::transpose(matrix::view(_t3), matrix::view(_t1)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    _set_block(_p_prop, 0, 0, _t2);
    _set_block(_p_prop, 0, 3, _t3);
    _set_block(_p_prop, 3, 0, _t1);
    _set_block(_p_prop, 3, 3, _p11);
    return PYRO_OK;
}

/* SO(3) 左雅可比，结果写入成员 _jacobian：
 *   J_l(v) = I + (1-cos|v|)/|v|²·[v]x + (|v|-sin|v|)/|v|³·[v]x²
 * 逐元素形式与原来的表达式 `identity + skew*first + (skew*skew)*second`
 * 保持相同的乘加顺序，因此结果逐位不变。
 * 返回码：矩阵乘法失败时返回错误（原实现走 operator*，失败被静默吞掉）。
 *
 * trig 复用（2026-11-17）：_update_gyr 在同一个样本上先后调用 rotation_step()
 * （用半角 h=|v|/2 求 sin/cos）与本函数（用全角 |v| 求 sin/cos），
 * 两套超越函数共 4 次调用。现由 rotation_step 把 sin/cos(h) 算出来经
 * _trig 传进来，本函数用半角公式反推：
 *   sin|v| = 2·sh·ch,  cos|v| = ch² - sh²
 * 每样本省下 2 次 sinf/cosf（M4F 上各约 20~30 周期）。
 * 半角公式在 |v|→0 时无精度问题（sin|v|→0、cos|v|→1），且小角分支
 * 仍走下面的泰勒展开，不参与此路径。 */
status_t vqf_eskf_t::_compute_left_jacobian(const vector3_t &angle)
{
    const float squared = square(angle[0]) + square(angle[1]) + square(angle[2]);
    float first = 0.5f - squared / 24.0f;
    float second = 1.0f / 6.0f - squared / 120.0f;
    if (squared > 1.0e-4f)
    {
        const float magnitude = sqrtf(squared);
        /* 由调用方预先算好的半角三角函数反推全角值；
         * 若调用方未提供（_trig_valid == false）则退回直接计算。 */
        float sin_m;
        float cos_m;
        if (_trig_valid)
        {
            sin_m = 2.0f * _trig_sin * _trig_cos;
            cos_m = square(_trig_cos) - square(_trig_sin);
        }
        else
        {
            sin_m = sinf(magnitude);
            cos_m = cosf(magnitude);
        }
        first = (1.0f - cos_m) / squared;
        second = (magnitude - sin_m) / (squared * magnitude);
    }
    _skew(0, 0) = 0.0f;     _skew(0, 1) = -angle[2]; _skew(0, 2) = angle[1];
    _skew(1, 0) = angle[2]; _skew(1, 1) = 0.0f;      _skew(1, 2) = -angle[0];
    _skew(2, 0) = -angle[1]; _skew(2, 1) = angle[0]; _skew(2, 2) = 0.0f;
    if (matrix::multiply(matrix::view(_skew), matrix::view(_skew),
                         matrix::view(_skew_sq)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    for (std::size_t i = 0; i < 9; ++i)
    {
        const float identity_value = (i % 4U == 0U) ? 1.0f : 0.0f;
        _jacobian.data()[i] =
            (identity_value + _skew.data()[i] * first) + _skew_sq.data()[i] * second;
    }
    return PYRO_OK;
}

status_t vqf_eskf_t::_update_gyr(const vector3_t &gyr, float dt)
{
    const vector3_t unbiased = gyr - _state.bias;
    const vector3_t angle = unbiased * dt;
    if (!matrix::is_finite(norm(unbiased)) || !matrix::is_finite(norm(angle)))
        return PYRO_PARAM_ERROR;
    if (dt > 0.0f)
    {
        /* 一次算出半角 sin/cos，既用于本步旋转，也留给 _compute_left_jacobian
         * 用半角公式反推全角值（省 2 次超越函数）。 */
        _trig_valid = false;
        const quaternion_t next_gyr =
            normalized(multiply(_state.gyr_quat,
                                rotation_step(angle, &_trig_sin, &_trig_cos)));
        _trig_valid = true;
        // R_true=Exp(δθ_global)R_nom，δθ_dot=-R_nom δb-R_nom n_g。
        // 平均旋转 A=R_start J_l(ω dt)，对恒角速度积分偏置耦合。
        // 以下全部写入成员暂存（避免每样本构造/清零 matrix_t 临时量）。
        const status_t jacobian_result = _compute_left_jacobian(angle);
        if (jacobian_result != PYRO_OK)
            return jacobian_result;
        {
            const quaternion_t r_start = multiply(_state.acc_quat, _state.gyr_quat);
            arm_quaternion2rotation_f32(r_start.data(), _rot.data(), 1);
        }
        /* 显式检查返回码：原实现走 matrix_t::operator*，失败会被静默吞掉，
         * 导致本拍用残缺的 A/Aᵀ 继续传播协方差。 */
        if (matrix::multiply(matrix::view(_rot), matrix::view(_jacobian),
                             matrix::view(_avg_rot)) != ARM_MATH_SUCCESS ||
            matrix::transpose(matrix::view(_avg_rot), matrix::view(_avg_rot_t)) !=
                ARM_MATH_SUCCESS ||
            matrix::multiply(matrix::view(_avg_rot), matrix::view(_avg_rot_t),
                             matrix::view(_avg_rot_at)) != ARM_MATH_SUCCESS)
            return PYRO_ERROR;

        error_kf_t::covariance_t &Q = _q;
        /* 耦合块 M = -A·dt，即误差状态转移 F 的右上块。块传播只用到这一块，
         * 不再构造 6×6 的 F。Q 的右下 3×3 非对角块本拍不参与计算，必须显式清零
         * （原来靠默认构造清零）；其余元素由下面的循环完整覆盖。 */
        for (std::size_t row = 0; row < 3; ++row)
            for (std::size_t col = 0; col < 3; ++col)
                _m(row, col) = -_avg_rot(row, col) * dt;
        Q(3, 4) = Q(3, 5) = Q(4, 3) = Q(4, 5) = Q(5, 3) = Q(5, 4) = 0.0f;
        const float bias_rate = square(0.1f * deg_to_rad) / _config.bias_forgetting_time;
        const float gyro_variance = square(_config.gyro_noise_density * deg_to_rad) * dt;
        const float bias_variance = bias_rate * dt;
        for (std::size_t row = 0; row < 3; ++row)
        {
            Q(row, row) = gyro_variance;
            Q(row + 3, row + 3) = bias_variance;
            for (std::size_t col = 0; col < 3; ++col)
            {
                /* 原式为 Q(row,col) += …（Q 初值 0）：对角项在上一行刚被赋成
                 * gyro_variance，非对角项初值恒为 0，故等价于显式赋值。 */
                const float previous = (row == col) ? Q(row, row) : 0.0f;
                Q(row, col) =
                    previous + _avg_rot_at(row, col) * bias_variance * square(dt) / 3.0f;
                Q(row, col + 3) = Q(col + 3, row) =
                    -0.5f * _avg_rot(row, col) * bias_variance * dt;
            }
        }
        /* P⁻ = F·P·Fᵀ 用块结构算（3 次 3×3 代替 2 次 6×6），再交给 KF 补 Q 并提交。 */
        const status_t propagated = _propagate_covariance();
        if (propagated != PYRO_OK)
            return propagated;
        const status_t result = _error_kf.predict_covariance({}, _p_prop, Q);
        _diagnostics = _error_kf.get_diagnostics();
        if (result != PYRO_OK)
            return result;
        _state.gyr_quat = next_gyr;
    }
    if (_config.rest_bias_est_enabled)
    {
        (void)_rest_gyr_lp.process(gyr.data(), _state.rest_gyr_lp.data(), dt);
        const vector3_t deviation = gyr - _state.rest_gyr_lp;
        const float clip = _config.bias_clip * deg_to_rad;
        _state.rest_gyr_ok = norm(deviation) < _config.rest_th_gyr * deg_to_rad &&
            std::fabs(_state.rest_gyr_lp[0]) <= clip &&
            std::fabs(_state.rest_gyr_lp[1]) <= clip &&
            std::fabs(_state.rest_gyr_lp[2]) <= clip;
        if (!_state.rest_gyr_ok)
        {
            _state.rest_t = 0.0f;
            _state.rest_detected = false;
        }
    }
    return PYRO_OK;
}

void vqf_eskf_t::_reject_acc()
{
    _state.acc_rejected = true;
    ++_state.acc_rejections;
    _state.trusted_acc_t = _state.observation_t = 0.0f;
    _state.rest_t = 0.0f;
    _state.rest_detected = false;
    // 恢复时重新积累可信历史，冲击样本的低通尾迹在此清除。
    _acc_lp.reset();
}

status_t vqf_eskf_t::_update_acc(const vector3_t &acc, float dt)
{
    const float magnitude = norm(acc);
    _state.acc_confidence = 0.0f;
    if (magnitude <= std::numeric_limits<float>::min())
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    const float norm_error = std::fabs(magnitude - gravity);
    if (norm_error > 3.0f * _config.acc_norm_tolerance)
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    const vector3_t acc_inertial = rotate(_state.gyr_quat, acc);
    if (!_state.reference_initialized)
    {
        _state.acc_reference = acc_inertial;
        _state.reference_initialized = true;
    }
    // 50 ms 能量窗口在近惯性系评估，真实转动导致的重力投影变化已补偿。
    const float gain = dt / (vibration_tau + dt);
    const vector3_t highpass = acc_inertial - _state.acc_reference;
    const float energy = square(highpass[0]) + square(highpass[1]) + square(highpass[2]);
    const float next_variance = _state.vibration_variance + gain * (energy - _state.vibration_variance);
    if (!matrix::is_finite(energy) || !matrix::is_finite(next_variance))
    {
        _state.reference_initialized = false;
        _state.vibration_variance = 0.0f;
        _reject_acc();
        return PYRO_WARNING;
    }
    _state.acc_reference += highpass * gain;
    _state.vibration_variance = next_variance;
    float confidence = 1.0f / (1.0f + square(norm_error / _config.acc_norm_tolerance) +
        _state.vibration_variance / square(_config.acc_vibration_tolerance));
    bool direction_rejected = false;
    if (_state.tilt_initialized)
    {
        const auto raw_direction = rotate(_state.acc_quat, acc_inertial / magnitude);
        const auto raw_error = inclination_error(raw_direction);
        const auto &P = _error_kf.get_covariance();
        const float variance = P(0, 0) + P(1, 1) + 2.0f * square(_config.acc_noise / magnitude);
        const float normalized_error = (square(raw_error[0]) + square(raw_error[1])) / variance;
        confidence /= 1.0f + normalized_error / _config.acc_nis_threshold;
        direction_rejected = normalized_error > _config.acc_nis_threshold;
    }
    _state.acc_confidence = std::clamp(confidence, 0.0f, 1.0f);
    if (!matrix::is_finite(confidence) ||
        confidence < min_acc_confidence || direction_rejected)
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    _state.acc_rejected = false;
    if (_config.rest_bias_est_enabled)
    {
        (void)_rest_acc_lp.process(acc.data(), _state.rest_acc_lp.data(), dt);
        const auto deviation = acc - _state.rest_acc_lp;
        if (_state.rest_gyr_ok && norm(deviation) < _config.rest_th_acc &&
            norm_error < _config.rest_th_acc)
        {
            _state.rest_t += dt;
            _state.rest_detected = _state.rest_t >= _config.rest_min_t;
        }
        else
        {
            _state.rest_t = 0.0f;
            _state.rest_detected = false;
        }
    }
    (void)_acc_lp.process(acc_inertial.data(), _state.last_acc_lp.data(), dt);
    _state.trusted_acc_t += dt;
    _state.observation_t += dt;
    if (_state.trusted_acc_t < _config.acc_recovery_t)
    {
        _state.acc_rejected = true;
        ++_state.acc_rejections;
        return PYRO_WARNING;
    }
    if (!_state.tilt_initialized)
        return _initialize_tilt();
    if (_state.observation_t < _config.acc_update_period)
        return PYRO_OK;
    const float interval = _state.observation_t;
    _state.observation_t = 0.0f;
    const status_t result = _correct_acc(interval);
    if (result == PYRO_WARNING)
        _reject_acc();
    if (result != PYRO_OK)
        return result;
    if (_state.rest_detected && _config.rest_bias_est_enabled)
        return _correct_rest(interval);
    return PYRO_OK;
}

status_t vqf_eskf_t::_initialize_tilt()
{
    const float magnitude = norm(_state.last_acc_lp);
    if (!matrix::is_finite(magnitude) || magnitude <= std::numeric_limits<float>::min())
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    const auto direction = rotate(_state.acc_quat, _state.last_acc_lp / magnitude);
    const auto next_acc = normalized(multiply(rotation_step(inclination_error(direction)), _state.acc_quat));
    auto P = _error_kf.get_covariance();
    // 初始重力对齐确定两个倾斜自由度，全局航向保留初始不确定度。
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 6; ++col)
            P(row, col) = P(col, row) = 0.0f;
    P(0, 0) = P(1, 1) = square(_config.acc_noise / magnitude) / square(_state.acc_confidence);
    P(2, 2) = square(_config.attitude_sigma_init * deg_to_rad);
    const status_t result = _error_kf.reset({}, P);
    if (result != PYRO_OK)
        return result;
    _state.acc_quat = next_acc;
    _state.tilt_initialized = true;
    _state.observation_t = 0.0f;
    return PYRO_OK;
}

status_t vqf_eskf_t::_correct_acc(float interval)
{
    const float magnitude = norm(_state.last_acc_lp);
    if (!matrix::is_finite(magnitude) || magnitude <= std::numeric_limits<float>::min())
        return PYRO_WARNING;
    const auto direction = rotate(_state.acc_quat, _state.last_acc_lp / magnitude);
    const auto residual = inclination_error(direction);
    error_kf_t::observation_t H;
    H(0, 0) = H(1, 1) = 1.0f;
    error_kf_t::measurement_noise_t R;
    // 低通观测相关：按 tau/实际观测间隔放大 R，限制重复使用相邻滤波样本的信息量。
    // 这是保守的相关时间近似，硬件噪声标定及回放用于进一步调参。
    const float correlation_scale = std::max(1.0f, _config.tau_acc / interval);
    R(0, 0) = R(1, 1) = square(_config.acc_noise / magnitude) * correlation_scale /
        square(_state.acc_confidence);
    const status_t result = _correct(residual, H, R, true);
    if (result == PYRO_OK)
        ++_state.acc_corrections;
    return result;
}

status_t vqf_eskf_t::_correct_rest(float interval)
{
    error_kf_t::observation_t H;
    for (std::size_t i = 0; i < 3; ++i)
        H(i, i + 3) = 1.0f;
    const float variance = square(_config.bias_sigma_rest * deg_to_rad) *
        std::max(1.0f, 2.0f * _config.rest_filter_tau / interval);
    return _correct(_state.rest_gyr_lp - _state.bias, H, identity<3>(variance), false);
}

status_t vqf_eskf_t::_gate(error_kf_t::correction_t &correction, bool acceleration)
{
    if (acceleration && correction.diagnostics.nis > _config.acc_nis_threshold)
        return PYRO_WARNING;
    if (acceleration)
    {
        // 六轴重力观测修正全局水平倾斜，航向误差维持陀螺传播。
        for (std::size_t col = 0; col < 3; ++col)
            correction.K(2, col) = 0.0f;
        if (!_config.motion_bias_est_enabled || _state.rest_detected)
            for (std::size_t row = 3; row < 6; ++row)
                for (std::size_t col = 0; col < 3; ++col)
                    correction.K(row, col) = 0.0f;
    }
    error_kf_t::state_t delta;
    if (matrix::multiply(matrix::view(correction.K), matrix::view(correction.y),
                         matrix::view(delta)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    const float angle = norm({delta[0], delta[1], delta[2]});
    if (!matrix::is_finite(delta) || !matrix::is_finite(angle))
        return PYRO_ERROR;
    const float scale = angle > max_correction_angle ? max_correction_angle / angle : 1.0f;
    correction.K *= scale;
    const float clip = _config.bias_clip * deg_to_rad;
    for (std::size_t row = 3; row < 6; ++row)
    {
        const float change = delta[row] * scale;
        const float candidate = _state.bias[row - 3] + change;
        if (change == 0.0f || std::fabs(candidate) <= clip)
            continue;
        const float bounded = std::clamp(candidate, -clip, clip);
        const float row_scale = std::clamp((bounded - _state.bias[row - 3]) / change, 0.0f, 1.0f);
        for (std::size_t col = 0; col < 3; ++col)
            correction.K(row, col) *= row_scale;
    }
    // 核心根据调整后的 K 同时执行状态及 Joseph 协方差更新。
    return PYRO_OK;
}

status_t vqf_eskf_t::_correct(const vector3_t &residual, const error_kf_t::observation_t &H,
                         const error_kf_t::measurement_noise_t &R, bool acceleration)
{
    const auto prior = _error_kf.get_covariance();
    error_kf_t::measurement_mask_t mask{true, true, !acceleration};
    /* eskf_t::correct 与 kf_t::correct 的唯一区别是多一个 dx 出参：把本次
     * 实际施加的误差增量交给 _inject_error()。协方差仍由核心按最终 K 更新，
     * 因此「更新协方差用的增量」与「注入名义状态的增量」是同一份。 */
    error_kf_t::state_t delta{};
    const status_t result = _error_kf.correct(residual, H, R, delta, &mask,
        [this, acceleration](error_kf_t::correction_t &correction)
        { return _gate(correction, acceleration); });
    _diagnostics = _error_kf.get_diagnostics();
    if (acceleration)
        _state.acc_nis = _diagnostics.nis;
    if (result != PYRO_OK)
        return result;
    const status_t injected = _inject_error(delta);
    if (injected != PYRO_OK)
    {
        // 注入失败时恢复修正前协方差，名义姿态和零偏仍保持原值。
        (void)_error_kf.reset({}, prior);
        _diagnostics.status = injected;
    }
    return injected;
}

status_t vqf_eskf_t::_inject_error(const error_kf_t::state_t &delta)
{
    const vector3_t angle{delta[0], delta[1], delta[2]};
    /* 与 _update_gyr 同样复用半角三角函数：rotation_step 输出、雅可比反推。
     * 这里必须显式置 _trig_valid，否则会沿用上一次 _update_gyr 的残值，
     * 用错误的角度反推出错误的雅可比——那是静默的数值污染。 */
    _trig_valid = false;
    const auto next_acc =
        normalized(multiply(rotation_step(angle, &_trig_sin, &_trig_cos), _state.acc_quat));
    _trig_valid = true;
    const auto next_bias = _state.bias + vector3_t{delta[3], delta[4], delta[5]};
    if (!matrix::is_finite(next_acc) || !matrix::is_finite(next_bias))
        return PYRO_ERROR;
    /* 误差注入：G = [[J_l, 0], [0, I]]，用块结构计算新的 P。 */
    const status_t jacobian_result = _compute_left_jacobian(angle);
    if (jacobian_result != PYRO_OK)
        return jacobian_result;
    /* P ← G·P·Gᵀ，G = [[J_l, 0], [0, I]]：块结构 3 次 3×3 乘法。 */
    const status_t rotated = _rotate_covariance();
    if (rotated != PYRO_OK)
        return rotated;
    const status_t result = _error_kf.reset({}, _p_prop);
    if (result != PYRO_OK)
        return result;
    _state.acc_quat = next_acc;
    _state.bias = next_bias;
    return PYRO_OK;
}

void vqf_eskf_t::get_state(state_t &result) const
{
    result.quat = multiply(_state.acc_quat, _state.gyr_quat);
    result.bias = _state.bias;
    result.error_covariance = _error_kf.get_covariance();
    float bound = 0.0f;
    for (std::size_t row = 0; row < 3; ++row)
    {
        float sum = 0.0f;
        for (std::size_t col = 0; col < 3; ++col)
        {
            result.bias_covariance(row, col) = result.error_covariance(row + 3, col + 3);
            sum += std::fabs(result.bias_covariance(row, col));
        }
        bound = std::max(bound, sum);
    }
    result.bias_sigma = sqrtf(bound);
    result.rest_detected = _state.rest_detected;
    result.rest_t = _state.rest_t;
    result.rest_gyr_lp = _state.rest_gyr_lp;
    result.rest_acc_lp = _state.rest_acc_lp;
    result.initialized = _state.tilt_initialized;
    result.consumed_gyr = _state.consumed_gyr;
    result.consumed_acc = _state.consumed_acc;
    result.updated_gyr = _state.updated_gyr;
    result.updated_acc = _state.updated_acc;
    result.acc_confidence = _state.acc_confidence;
    result.acc_nis = _state.acc_nis;
    result.acc_rejected = _state.acc_rejected;
    result.acc_corrections = _state.acc_corrections;
    result.acc_rejections = _state.acc_rejections;
    result.bias_diagnostics = _diagnostics;
}

vqf_eskf_t::state_t vqf_eskf_t::get_state() const
{
    state_t result;
    get_state(result);
    return result;
}

} // namespace pyro
