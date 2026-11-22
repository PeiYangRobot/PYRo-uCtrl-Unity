#ifndef PYRO_INS_ATTITUDE_H
#define PYRO_INS_ATTITUDE_H

#include <array>
#include <cstdint>

#include "pyro_algo_vqf_eskf.h"

namespace pyro
{

// 融合 IMU 样本，并将器件姿态转换为机体及导航系输出。
class ins_attitude_t
{
  public:
    enum class direction_t : uint8_t
    {
        DIRECT_1 = 1,
        DIRECT_2 = 2,
        DIRECT_3 = 3,
        DIRECT_4 = 4,
        DIRECT_5 = 5,
        DIRECT_6 = 6,
        DIRECT_7 = 7,
        DIRECT_8 = 8
    };

    using vector_t = std::array<float, 3>;
    using quaternion_t = std::array<float, 4>;

    struct output_t
    {
        quaternion_t quat{};   // [w, x, y, z]，机体系到导航系。
        vector_t angles_rad{}; // [横滚, 俯仰, 航向]。
        vector_t gyro_b{}, gyro_n{};
        vector_t accel_b{}, accel_n{};
        vector_t linear_b{}, linear_n{};
    };

    struct update_t
    {
        uint16_t consumed_acc = 0U;
        uint16_t consumed_gyro = 0U;
        bool updated_acc = false;
        bool updated_gyro = false;
        bool output_ready = false;
    };

    static bool valid_direction(direction_t direction);
    status_t init(direction_t direction, float gravity, float max_sample_gap_s);
    status_t reset();
    // 驱动按消费量释放样本；output_ready 时输出本拍结果。
    status_t update(const imu_data_t &data, output_t &output, update_t &step);

  private:
    vqf_eskf_t _filter;
    std::array<int8_t, 3> _mount_axes{};
    quaternion_t _mount_inverse{};
    quaternion_t _heading_correction{};
    vector_t _acc{}, _gyro{};
    float _gravity = 9.80665f;
    float _last_yaw = 0.0f;
    bool _heading_set = false;
    bool _acc_seen = false;
    bool _gyro_seen = false;
};

} // namespace pyro
#endif // PYRO_INS_ATTITUDE_H
