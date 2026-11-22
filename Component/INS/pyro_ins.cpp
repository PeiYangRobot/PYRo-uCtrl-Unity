#include "pyro_ins.h"

#include "imu_base_drv.h"

#include <cmath>

namespace pyro
{
namespace
{
constexpr uint32_t stale_ms = 20U;
constexpr float rad_to_deg = 180.0f / PI;

bool expired(uint32_t now, uint32_t last)
{
    return now - last >= pdMS_TO_TICKS(stale_ms);
}

} // namespace

ins_drv_t ins_drv_t::_instance;

ins_drv_t *ins_drv_t::get_instance()
{
    return &_instance;
}

status_t ins_drv_t::init(const ins_config_t &config)
{
    return _imu != nullptr ? init(*_imu, config) : PYRO_NOT_FOUND;
}

status_t ins_drv_t::init(imu_base_drv &imu, const ins_config_t &config)
{
    if (_task.active())
        return PYRO_ALREADY_INIT;

    if (!ins_attitude_t::valid_direction(config.direct) ||
        !std::isfinite(config.g_norm) || config.g_norm <= 0.0f ||
        !std::isfinite(config.gx_offset) || !std::isfinite(config.gy_offset) || !std::isfinite(config.gz_offset))
    {
        return PYRO_PARAM_ERROR;
    }

    invalidate();
    _imu = &imu;
    _config = config;
    const status_t status = _task.start();
    // task_base_t::start() 在 xTaskCreate 失败时也可能返回 PYRO_OK。
    return status == PYRO_OK && !_task.active() ? PYRO_ERROR : status;
}

void ins_drv_t::run_task()
{
    auto &data = _imu->read_imu_data();
    data.cfg.calibrate_flag = _config.calibrate;
    data.cali = {_config.gx_offset, _config.gy_offset, _config.gz_offset, _config.g_norm};
    if (!_imu->init() ||
        _attitude.init(_config.direct, data.cali.g_norm, static_cast<float>(stale_ms) * 0.001f) != PYRO_OK)
        return;

    _acc_time = {};
    _gyro_time = {};

    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        update();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1U));
    }
}

void ins_drv_t::update()
{
    (void)_imu->read();
    const auto &data = _imu->read_imu_data();
    const uint32_t now = xTaskGetTickCount();
    if ((_acc_time.seen && expired(now, _acc_time.tick)) ||
        (_gyro_time.seen && expired(now, _gyro_time.tick)))
        reset_attitude();
    if (data.acc.available() == 0U && data.gyro.available() == 0U)
        return;

    snapshot_t next{};
    ins_attitude_t::update_t step;
    const status_t status = _attitude.update(data, next.attitude, step);
    _imu->assume_acc_point(step.consumed_acc);
    _imu->assume_gyro_point(step.consumed_gyro);
    if (status != PYRO_OK && status != PYRO_WARNING)
    {
        reset_attitude();
        return;
    }

    if (step.updated_acc)
        _acc_time = {now, true};
    if (step.updated_gyro)
        _gyro_time = {now, true};
    if (!step.output_ready)
        return;

    next.acc_tick = _acc_time.tick;
    next.gyro_tick = _gyro_time.tick;
    next.valid = true;
    taskENTER_CRITICAL();
    _snapshot = next;
    taskEXIT_CRITICAL();
}

void ins_drv_t::invalidate()
{
    taskENTER_CRITICAL();
    _snapshot.valid = false;
    taskEXIT_CRITICAL();
}

void ins_drv_t::reset_attitude()
{
    invalidate();
    (void)_attitude.reset();
    _acc_time = {};
    _gyro_time = {};
}

bool ins_drv_t::copy_snapshot(snapshot_t &result) const
{
    taskENTER_CRITICAL();
    result = _snapshot;
    taskEXIT_CRITICAL();

    const uint32_t now = xTaskGetTickCount();
    return result.valid && !expired(now, result.acc_tick) && !expired(now, result.gyro_tick);
}

status_t ins_drv_t::get_vector(vector_t output_t::*field, float *x, float *y, float *z,
                               float scale, bool reverse) const
{
    if (x == nullptr || y == nullptr || z == nullptr)
        return PYRO_ERROR;
    snapshot_t data;
    if (!copy_snapshot(data))
        return PYRO_NOT_FOUND;

    const auto &vector = data.attitude.*field;
    *x = vector[reverse ? 2U : 0U] * scale;
    *y = vector[1] * scale;
    *z = vector[reverse ? 0U : 2U] * scale;
    return PYRO_OK;
}

status_t ins_drv_t::get_angles_b(float *yaw, float *pitch, float *roll)
{
    return get_angles_n(yaw, pitch, roll);
}

status_t ins_drv_t::get_angles_n(float *yaw, float *pitch, float *roll)
{
    return get_vector(&output_t::angles_rad, yaw, pitch, roll, rad_to_deg, true);
}

status_t ins_drv_t::get_rads_b(float *yaw, float *pitch, float *roll)
{
    return get_rads_n(yaw, pitch, roll);
}

status_t ins_drv_t::get_rads_n(float *yaw, float *pitch, float *roll)
{
    return get_vector(&output_t::angles_rad, yaw, pitch, roll, 1.0f, true);
}

status_t ins_drv_t::get_gyro_b(float *yaw, float *pitch, float *roll)
{
    return get_vector(&output_t::gyro_b, yaw, pitch, roll, 1.0f, true);
}

status_t ins_drv_t::get_gyro_n(float *yaw, float *pitch, float *roll)
{
    return get_vector(&output_t::gyro_n, yaw, pitch, roll, 1.0f, true);
}

status_t ins_drv_t::get_accel_b(float *x, float *y, float *z)
{
    return get_vector(&output_t::accel_b, x, y, z);
}

status_t ins_drv_t::get_accel_n(float *x, float *y, float *z)
{
    return get_vector(&output_t::accel_n, x, y, z);
}

status_t ins_drv_t::get_accel_without_g_b(float *x, float *y, float *z)
{
    return get_vector(&output_t::linear_b, x, y, z);
}

status_t ins_drv_t::get_accel_without_g_n(float *x, float *y, float *z)
{
    return get_vector(&output_t::linear_n, x, y, z);
}

status_t ins_drv_t::get_quaternion(float *q0, float *q1, float *q2, float *q3)
{
    if (q0 == nullptr || q1 == nullptr || q2 == nullptr || q3 == nullptr)
        return PYRO_ERROR;
    snapshot_t data;
    if (!copy_snapshot(data))
        return PYRO_NOT_FOUND;

    *q0 = data.attitude.quat[0];
    *q1 = data.attitude.quat[1];
    *q2 = data.attitude.quat[2];
    *q3 = data.attitude.quat[3];
    return PYRO_OK;
}
} // namespace pyro
