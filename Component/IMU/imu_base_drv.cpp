#include "imu_base_drv.h"
#include "main.h" /* HAL_GetTick()：标定超时判据 */

#include <cmath>

imu_base_drv::~imu_base_drv() = default;

/**
 * @brief 陀螺零偏 + 重力模长标定
 * @note  依赖派生类 read() 的语义："取回一批样本，陀螺写入 gyro_sample、
 *        加速度写入 acc 流，两者未被消费的数量体现在 acc.available() / gyro.available()"。
 *        本函数不关心 read() 的返回值，只按两路计数判断本批有没有新数据。
 *        标定期间器件需保持静止。两路独立计数，各自攒够 target 个样本为止
 *        （陀螺/加速度同速率时同步完成；1920 Hz 下 500 个约 260 ms）。
 *        本函数会**消费**样本（读后调 assume_*），避免下一轮重复累加同一批。
 *        单位：Gyro 为 rad/s；Accel 为 m/s²，故静止时 g_norm ≈ 9.80665。
 * @note  超时判据基于真实时间（HAL_GetTick），不是循环次数：见下方常量说明。
 */
bool imu_base_drv::calibrate() {
    const uint16_t target = (_data.cfg.calibrate_times > 0U) ? _data.cfg.calibrate_times : 2000U;
    const imu_cali_t previous_cali = _data.cali;

    /* 偏置存储在器件原坐标系。驱动的输出契约即为器件坐标系，标定天然读到的
     * 就是未扣偏置的新样本，无需临时更改任何配置。 */
    _data.cali.gx_offset = 0.0f;
    _data.cali.gy_offset = 0.0f;
    _data.cali.gz_offset = 0.0f;
    assume_gyro_point(_data.gyro.available());
    assume_acc_point(_data.acc.available());

    double   sum_gyro[3] = {0.0, 0.0, 0.0};
    double   sum_norm    = 0.0;
    uint32_t got_gyro    = 0U;
    uint32_t got_acc     = 0U;

    /* 停滞与总时长保护（旧实现用"连续无进展的循环次数 5e6"，
     * 而每次循环至少一次 read()（≥1 ms），实际要几十分钟才触发，等于没保护）。
     *   ① 连续 k_calib_stall_ms 没有任何新样本 → 器件停摆 / 总线不通；
     *   ② 总时长超过标称耗时的 3 倍 + 500 ms → 器件实际 ODR 与配置不符。 */
    constexpr uint32_t k_calib_stall_ms = 200U;
    const uint32_t min_freq_hz = (_data.cfg.acc_freq < _data.cfg.gyro_freq)
                                     ? _data.cfg.acc_freq
                                     : _data.cfg.gyro_freq;
    const uint32_t freq_hz    = (min_freq_hz > 0U) ? min_freq_hz : 1000U;
    const uint32_t nominal_ms = 1000U * static_cast<uint32_t>(target) / freq_hz;
    const uint32_t budget_ms  = nominal_ms * 3U + 500U;
    const uint32_t start_ms   = HAL_GetTick();
    uint32_t       last_progress_ms = start_ms;

    while (got_gyro < static_cast<uint32_t>(target) || got_acc < static_cast<uint32_t>(target)) {
        const uint32_t previous_gyro_count = got_gyro;
        const uint32_t previous_acc_count  = got_acc;
        (void)read(); /* 标定不关心返回值，只看两路可用样本数 */

        const uint16_t ng = _data.gyro.available();
        const uint16_t na = _data.acc.available();
        for (uint16_t i = 0U; i < ng && got_gyro < static_cast<uint32_t>(target); ++i) {
            const gyro_sample_t &s = _data.gyro.at(i); /* 逻辑下标，内部处理回绕 */
            sum_gyro[0] += static_cast<double>(s.Gyro[0]);
            sum_gyro[1] += static_cast<double>(s.Gyro[1]);
            sum_gyro[2] += static_cast<double>(s.Gyro[2]);
            ++got_gyro;
        }
        for (uint16_t i = 0U; i < na && got_acc < static_cast<uint32_t>(target); ++i) {
            const acc_sample_t &s = _data.acc.at(i);
            const double ax = s.Accel[0];
            const double ay = s.Accel[1];
            const double az = s.Accel[2];
            sum_norm += std::sqrt(ax * ax + ay * ay + az * az);
            ++got_acc;
        }

        /* 消费掉本批，下一轮才不会被重复累加 */
        assume_gyro_point(ng);
        assume_acc_point(na);

        const uint32_t now_ms = HAL_GetTick();
        if (got_gyro != previous_gyro_count || got_acc != previous_acc_count) {
            last_progress_ms = now_ms;
        }
        if ((now_ms - last_progress_ms) > k_calib_stall_ms ||
            (now_ms - start_ms) > budget_ms) {
            /* 以实际标定进度检测停滞，覆盖仅一路有数据的情况。 */
            _data.cali = previous_cali;
            return false;
        }
    }

    const double inv_g = 1.0 / static_cast<double>(got_gyro);
    const double inv_a = 1.0 / static_cast<double>(got_acc);
    _data.cali.gx_offset = static_cast<float>(sum_gyro[0] * inv_g);
    _data.cali.gy_offset = static_cast<float>(sum_gyro[1] * inv_g);
    _data.cali.gz_offset = static_cast<float>(sum_gyro[2] * inv_g);
    _data.cali.g_norm    = static_cast<float>(sum_norm * inv_a);

    return true;
}
