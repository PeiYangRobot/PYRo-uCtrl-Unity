#ifndef PYRO_INS_H
#define PYRO_INS_H

#include "pyro_ins_attitude.h"
#include "pyro_task.h"

struct ins_config_t
{
    using imu_direct_t = pyro::ins_attitude_t::direction_t;

    imu_direct_t direct = imu_direct_t::DIRECT_1;
    bool calibrate      = false;
    float gx_offset     = 0.0f;
    float gy_offset     = 0.0f;
    float gz_offset     = 0.0f;
    float g_norm        = 9.80665f;
};

class imu_base_drv;

namespace pyro
{

// 持有采集与融合任务，其他任务通过 getter 读取最近一次融合结果。
class ins_drv_t
{
  public:
    static ins_drv_t *get_instance();
    // 在板级启动阶段调用。初始化返回成功表示任务已创建，姿态就绪后 getter 才有数据。
    status_t init(const ins_config_t &config);
    // imu 的生命周期须覆盖 INS 任务。
    status_t init(imu_base_drv &imu, const ins_config_t &config);

    // 数据未就绪或过期时返回 PYRO_NOT_FOUND，输出参数保持原值。
    // 姿态角按航向、俯仰、横滚输出；角速度分别对应 Z、Y、X。
    status_t get_angles_b(float *yaw, float *pitch, float *roll);
    status_t get_angles_n(float *yaw, float *pitch, float *roll);
    status_t get_rads_b(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_rads_n(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_gyro_b(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_gyro_n(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_accel_b(float* accel_x, float* accel_y, float* accel_z);
    status_t get_accel_n(float* accel_x, float* accel_y, float* accel_z);
    status_t get_accel_without_g_b(float* a_x, float* a_y, float* a_z);
    status_t get_accel_without_g_n(float* a_x, float* a_y, float* a_z);
    status_t get_quaternion(float *q0, float *q1, float *q2, float *q3);

  private:
    using output_t = ins_attitude_t::output_t;
    using vector_t = ins_attitude_t::vector_t;

    class ins_task_t final : public task_base_t
    {
      public:
        explicit ins_task_t(ins_drv_t &owner)
            : task_base_t("ins_task", 0, 1024, priority_t::BELOW_NORMAL), _owner(owner)
        {
        }

        bool active() const { return _loop_task_handle != nullptr; }

      private:
        // task_base_t::start() 同步执行此回调，硬件初始化留在循环任务内。
        status_t init() override { return PYRO_OK; }
        void run_loop() override { _owner.run_task(); }

        ins_drv_t &_owner;
    };

    struct stream_time_t
    {
        uint32_t tick = 0U;
        bool seen = false;
    };

    struct snapshot_t
    {
        output_t attitude{};
        uint32_t acc_tick = 0U;
        uint32_t gyro_tick = 0U;
        bool valid = false;
    };

    ins_drv_t() = default;
    ins_drv_t(const ins_drv_t &) = delete;
    ins_drv_t &operator=(const ins_drv_t &) = delete;
    static ins_drv_t _instance;
    ins_task_t _task{*this};
    imu_base_drv *_imu = nullptr;
    ins_config_t _config{};
    ins_attitude_t _attitude;
    stream_time_t _acc_time, _gyro_time;
    snapshot_t _snapshot{};

    void run_task();
    void update();
    void invalidate();
    void reset_attitude();
    bool copy_snapshot(snapshot_t &result) const;
    status_t get_vector(vector_t output_t::*field, float *x, float *y, float *z,
                        float scale = 1.0f, bool reverse = false) const;
};
}
#endif
