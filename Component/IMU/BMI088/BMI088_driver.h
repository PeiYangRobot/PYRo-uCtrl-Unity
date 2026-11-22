#ifndef BMI088_DRIVER_H
#define BMI088_DRIVER_H

#include "imu_base_drv.h"

class bmi088_t;

class bmi088_drv_t final : public imu_base_drv
{
  public:
    // 使用默认采样配置。自定义参数可在 init() 前通过 read_imu_data().cfg 设置。
    explicit bmi088_drv_t(bmi088_t &device);

    bool init() override;
    // FIFO 模式独立交付两路批次，直读模式成对交付。
    bool read() override;

  private:
    static constexpr uint16_t stream_capacity = 64U;

    struct sample_clock_t
    {
        uint32_t period_ticks = 0U;
        uint32_t last_timestamp = 0U;
        bool valid = false;

        uint32_t first_timestamp(uint32_t snapshot, uint16_t frames) const;
    };

    bmi088_t &_device;
    acc_sample_t _acc[stream_capacity]{};
    gyro_sample_t _gyro[stream_capacity]{};
    sample_clock_t _acc_clock{};
    sample_clock_t _gyro_clock{};
    bool _fifo_recovery_pending = false;
    bool _fifo_enabled = false;
    float _acc_mps2_per_lsb = 0.0f;
    float _gyro_rps_per_lsb = 0.0f;

    bool _ready = false;

    bool read_direct();
    bool read_fifo();
    bool configure_fifo();
    bool reset_fifo();
    void decode_accel(acc_sample_t &sample, const int16_t (&axes)[3], uint32_t timestamp) const;
    void decode_gyro(gyro_sample_t &sample, const int16_t *axes, uint32_t timestamp) const;
};

#endif
