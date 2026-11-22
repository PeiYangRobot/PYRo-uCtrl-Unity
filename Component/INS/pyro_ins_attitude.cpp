#include "pyro_ins_attitude.h"

#include "dsp/quaternion_math_functions.h"
#include "imu_base_drv.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace pyro
{
namespace
{
using axes_t = std::array<int8_t, 3>;
using vec3_t = std::array<float, 3>;
using quat_t = std::array<float, 4>;

// 每个轴使用带符号的一基序号，表示从器件系取哪个分量。
constexpr axes_t mounting_axes[] = {
    {1, 2, 3}, {2, -1, 3}, {-1, -2, 3}, {-2, 1, 3},
    {1, -2, -3}, {-2, -1, -3}, {-1, 2, -3}, {2, 1, -3},
};

uint8_t axis_index(int8_t axis)
{
    return static_cast<uint8_t>((axis > 0 ? axis : -axis) - 1);
}

vec3_t to_body(const axes_t &axes, const vec3_t &sensor)
{
    vec3_t body{};
    for (uint8_t i = 0U; i < body.size(); ++i)
        body[i] = (axes[i] > 0 ? 1.0f : -1.0f) * sensor[axis_index(axes[i])];
    return body;
}

quat_t mount_inverse(const axes_t &axes)
{
    // 装配矩阵的转置将机体系转换回器件系。
    float rotation[9]{};
    for (uint8_t i = 0U; i < axes.size(); ++i)
        rotation[3U * axis_index(axes[i]) + i] = axes[i] > 0 ? 1.0f : -1.0f;
    quat_t mounting{};
    arm_rotation2quaternion_f32(rotation, mounting.data(), 1U);
    return mounting;
}

float yaw_angle(const quat_t &q)
{
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

vec3_t euler_angles(const quat_t &q)
{
    const float roll = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                                  1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    const float pitch = std::asin(std::clamp(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f));
    return {roll, pitch, yaw_angle(q)};
}

vec3_t rotate(const float (&rotation)[9], const vec3_t &body)
{
    return {
        rotation[0] * body[0] + rotation[1] * body[1] + rotation[2] * body[2],
        rotation[3] * body[0] + rotation[4] * body[1] + rotation[5] * body[2],
        rotation[6] * body[0] + rotation[7] * body[1] + rotation[8] * body[2],
    };
}

vqf_eskf_t::config_t filter_config(float max_sample_gap_s)
{
    vqf_eskf_t::config_t config;
    config.tau_acc = 0.4f;
    config.rest_th_gyr = 5.0f;
    config.rest_th_acc = 1.0f;
    config.bias_clip = 5.0f;
    config.rest_filter_tau = 0.15f;
    config.motion_bias_est_enabled = false;
    config.max_sample_gap_s = max_sample_gap_s;
    return config;
}
} // namespace

bool ins_attitude_t::valid_direction(direction_t direction)
{
    const auto index = static_cast<uint8_t>(direction);
    return index != 0U && index <= std::size(mounting_axes);
}

status_t ins_attitude_t::init(direction_t direction, float gravity, float max_sample_gap_s)
{
    if (!valid_direction(direction) || !std::isfinite(gravity) || gravity <= 0.0f)
        return PYRO_PARAM_ERROR;

    const auto status = _filter.configure(filter_config(max_sample_gap_s));
    if (status != PYRO_OK)
        return status;

    _mount_axes = mounting_axes[static_cast<uint8_t>(direction) - 1U];
    _mount_inverse = mount_inverse(_mount_axes);
    _gravity = gravity;
    _last_yaw = 0.0f;
    return reset();
}

status_t ins_attitude_t::reset()
{
    const auto status = _filter.reset();
    _acc_seen = _gyro_seen = _heading_set = false;
    return status;
}

status_t ins_attitude_t::update(const imu_data_t &data, output_t &output, update_t &step)
{
    step = {};
    const status_t status = _filter.update(data);
    vqf_eskf_t::state_t state;
    _filter.get_state(state);
    step.consumed_acc = state.consumed_acc;
    step.consumed_gyro = state.consumed_gyr;
    if (status != PYRO_OK && status != PYRO_WARNING)
    {
        // 时间断裂时保留尚未处理的帧，供重置后的滤波器建立时间基准。
        if (vqf_eskf_t::err_stage != 4U && vqf_eskf_t::err_stage != 6U)
        {
            step.consumed_acc = data.acc.available();
            step.consumed_gyro = data.gyro.available();
        }
        return status;
    }

    step.updated_acc = state.updated_acc != 0U;
    step.updated_gyro = state.updated_gyr != 0U;
    if (step.updated_acc)
    {
        const auto &a = data.acc.at(state.updated_acc - 1U).Accel;
        _acc = {a[0], a[1], a[2]};
        _acc_seen = true;
    }
    if (step.updated_gyro)
    {
        const auto &g = data.gyro.at(state.updated_gyr - 1U).Gyro;
        _gyro = {g[0], g[1], g[2]};
        _gyro_seen = true;
    }
    if (!state.initialized || !_acc_seen || !_gyro_seen || (!step.updated_acc && !step.updated_gyro))
        return status;

    quat_t attitude{};
    arm_quaternion_product_single_f32(state.quat.data(), _mount_inverse.data(), attitude.data());
    if (!_heading_set)
    {
        // 首次对准以当前航向为零，传感器恢复时接续最近一次航向。
        const float half = 0.5f * (_last_yaw - yaw_angle(attitude));
        _heading_correction = {std::cos(half), 0.0f, 0.0f, std::sin(half)};
        _heading_set = true;
    }
    arm_quaternion_product_single_f32(_heading_correction.data(), attitude.data(), output.quat.data());
    output.angles_rad = euler_angles(output.quat);
    _last_yaw = output.angles_rad[2];

    output.accel_b = to_body(_mount_axes, _acc);
    output.gyro_b = to_body(_mount_axes, {
        _gyro[0] - state.bias[0],
        _gyro[1] - state.bias[1],
        _gyro[2] - state.bias[2],
    });

    float rotation[9];
    arm_quaternion2rotation_f32(output.quat.data(), rotation, 1U);
    output.accel_n = rotate(rotation, output.accel_b);
    output.gyro_n = rotate(rotation, output.gyro_b);
    output.linear_n = output.accel_n;
    output.linear_n[2] -= _gravity;
    for (uint8_t i = 0U; i < 3U; ++i)
        output.linear_b[i] = output.accel_b[i] - rotation[6U + i] * _gravity;

    step.output_ready = true;
    return status;
}

} // namespace pyro
