#ifndef __IMU_BASE_DRV_H__
#define __IMU_BASE_DRV_H__
#include <cstdint>

/* 物理量一律为【器件坐标系】（LSM6DSV16X 的 X/Y/Z，与输出原码一一对应）。
 * 板级装配方向映射不属于器件契约，由产品装配层在输出端完成，见
 * Module/Config/imu_mount.h。 */
struct acc_sample_t {
    uint32_t timestamp;   /* 器件时间戳计数器原始值（tick），见上方说明 */
    float Accel[3];       /* 器件坐标系，单位 m/s² */
};

struct gyro_sample_t {
    uint32_t timestamp;
    float Gyro[3];        /* 器件坐标系，单位 rad/s */
};

struct imu_cfg_t {
    bool calibrate_flag{};
    uint16_t calibrate_times = 60000;
    uint16_t acc_range{};   // 单位 g（配置用 g,输出换算成 m/s²）
    uint16_t acc_freq{};    // 单位 Hz
    uint16_t gyro_range{};  // 单位 dps
    uint16_t gyro_freq{};   // 单位 Hz
    bool fifo_en{};

    /* 单次回读的最大 FIFO 字数（硬上限，超过则整批丢弃并排空器件 FIFO）。
     *
     * 为什么需要它：正常 1 ms 一拍只产生 2~3 个字（gyro + acc + timestamp）。
     * 若调度被长时间阻塞（或总线异常），器件 FIFO 会累积到上百字，一次全部
     * 涌入会同时压垮软件环形缓冲（64 帧）与算法（每样本跑一次协方差传播），
     * 表现为 err_backpressure 暴涨 + 融合耗时尖峰，随后还要多拍才能追平。
     * 这种情况下最新数据比"完整但陈旧的一大批"更有价值，因此整批抛弃。
     *
     * 取值依据：1920 Hz 下 1 ms ≈ 4 个字（两路 + 时间戳），
     * 16 字 ≈ 8 倍余量，既能容忍偶发抖动，又能在真正积压时及时止损。
     * 设为 0 表示不限制（退回旧的"最多读 MAX_WORDS_PER_XFER 字"行为）。 */
    uint16_t fifo_max_words_per_read = 16U;
};

struct imu_cali_t {
    /* 陀螺静态零偏，器件坐标系，rad/s。标定在器件系进行（见 calibrate()）。 */
    float gx_offset = 0.0f;
    float gy_offset = 0.0f;
    float gz_offset = 0.0f;
    /* 静止加速度模长，m/s²。目前只有 calibrate() 写入、尚无读者
     * （本产品不输出"去重力线性加速度"）。
     * 默认取标准重力而不是 0：将来按下式取线性加速度时
     *     linear = acc - (g_norm != 0 ? g_norm : 9.80665) * 单位重力方向
     * 默认 0 会让分母为 0、方向无定义，且这种错误是静默的。 */
    float g_norm = 9.80665f;
};

/* ============================ 顺序屏障 ============================
 * SPSC 的正确性只依赖"索引与数据的可见顺序"，不依赖互斥。
 * 单核 Cortex-M4 上生产者与消费者同核，观察自己的写入天然按程序序可见，
 * 因此真正要防的是【编译器重排】；ARM 目标额外发一条 dmb sy 作为架构级
 * 保证（将来若移植到多核必须保留它）。主机侧（g++/clang，无 __arm__）
 * 退化为纯编译器屏障，保证主机回归可跑。 */
#if defined(__arm__) || defined(__ARM_ARCH)
#define PYRO_STREAM_FENCE() __asm__ __volatile__("dmb sy" ::: "memory")
#elif defined(__GNUC__) || defined(__clang__)
#define PYRO_STREAM_FENCE() __asm__ __volatile__("" ::: "memory")
#else
#define PYRO_STREAM_FENCE() ((void)0)
#endif

/**
     * 单路样本流：单生产者/单消费者（SPSC）回绕环形缓冲。
     *
     * ============================ 所有权 ============================
     * 【每个索引只有一个写者】，这是无需任何临界区的前提：
     *   tail —— 只由生产者（驱动 read()）写；下一个可写槽
     *   head —— 只由消费者（上层 assume_*_point）写；最旧未消费样本
     * 未消费数由 (tail - head) 算出，不再单独存储 —— 旧实现里的 count 被双方
     * 读改写，是数据竞争的来源（丢失更新 + 半写样本被看见）。
     *
     * ============================ 发布顺序 ============================
     * 生产者：reserve() 拿槽 → 原地写满 payload → publish() 才移动 tail。
     * 消费者：available() 取一批快照 → at()/cur() 读 payload → release()。
     * 屏障保证"数据先于索引可见"，因此消费者绝不会看到半写的样本；
     * 最坏情况只是暂时看不到最新样本（available 偏小），安全性单调。
     *
     * ============================ 满时策略 ============================
     * 满则【丢弃新样本】：reserve() 返回 nullptr，由调用方计入 err_data。
     * 不采用"覆盖最旧"——覆盖必须移动 head，而 head 属于消费者；
     * 让生产者写 head 就等于退回需要加锁的设计。
     * 副作用：err_data 成为纯粹的"背压丢样"计数，不再与器件 FIFO 溢出混同。
     *
     * 约束：capacity 必须是 2 的幂（用掩码代替取模）；需留一个哨兵槽判别满，
     *       故实际可用深度 = capacity - 1。
     */
template <typename T>
struct sample_stream_t {
    T*       sample{};          /* 物理基址 */
    uint16_t capacity{};        /* 物理缓冲元素数，必须是 2 的幂 */
    volatile uint16_t head{};   /* 最旧未消费样本的物理下标 —— 只由消费者写 */
    volatile uint16_t tail{};   /* 下一个可写槽的物理下标   —— 只由生产者写 */

    /* 未消费样本数（消费者视角的一批快照；必须在本批处理前取一次）。 */
    uint16_t available() const {
        PYRO_STREAM_FENCE();
        return static_cast<uint16_t>(
            (static_cast<uint16_t>(tail) - static_cast<uint16_t>(head)) &
            static_cast<uint16_t>(capacity - 1U));
    }

    /* 逻辑下标 i（0 = 最旧）→ 实际元素。i 必须 < available()。 */
    const T& at(const uint16_t i) const {
        return sample[(static_cast<uint16_t>(head) + i) &
                      static_cast<uint16_t>(capacity - 1U)];
    }
    /* 最新样本（available() 必须 > 0） */
    const T& cur() const {
        return at(static_cast<uint16_t>(available() - 1U));
    }

    /* ---- 生产者侧 ---- */
    /* 取一个待写槽；返回 nullptr 表示已满（A1：丢弃新样本）。 */
    T* reserve() {
        if (capacity == 0U || sample == nullptr) {
            return nullptr;
        }
        const uint16_t t = static_cast<uint16_t>(tail);
        if (static_cast<uint16_t>((t + 1U) & static_cast<uint16_t>(capacity - 1U)) ==
            static_cast<uint16_t>(head)) {
            return nullptr; /* 满 */
        }
        return &sample[t];
    }
    /* 槽内容写完后调用，才让消费者可见。 */
    void publish() {
        PYRO_STREAM_FENCE();
        tail = static_cast<uint16_t>(
            (static_cast<uint16_t>(tail) + 1U) & static_cast<uint16_t>(capacity - 1U));
    }

    /* ---- 消费者侧 ---- */
    /* 归还 n 个已消费样本（超出未消费数时按实际数量处理）。 */
    void release(uint16_t n) {
        const uint16_t avail = available();
        const uint16_t drop  = (n >= avail) ? avail : n;
        PYRO_STREAM_FENCE();
        head = static_cast<uint16_t>(
            (static_cast<uint16_t>(head) + drop) & static_cast<uint16_t>(capacity - 1U));
    }
};

struct imu_data_t {
    /* 全部物理量为【器件坐标系】。驱动只做单位换算与器件零偏扣除，
     * 板级装配方向映射由产品装配层在输出端完成。 */
    /**
     * 两路样本流（陀螺 / 加速度）。
     * 取样本：acc.at(i)/acc.cur()、gyro.at(i)/gyro.cur()；
     * 数量：acc.available() / gyro.available()（0 表示没有新数据）；
     * 消费后归还：assume_acc_point()/assume_gyro_point()。
     */
    sample_stream_t<acc_sample_t>  acc{};
    sample_stream_t<gyro_sample_t> gyro{};

    float temperature{};
    imu_cali_t cali{};
    imu_cfg_t cfg = imu_cfg_t{};
    float timestamp_tick_s = 1.0e-6f; /* timestamp 每 tick 的秒数，由具体器件驱动填写 */
};

class imu_base_drv {
    public:
    explicit imu_base_drv(const imu_cfg_t& imu_cfg) {
        _data.cfg = imu_cfg;
    };
    imu_data_t& read_imu_data() {
        return _data;
    }

    /* 归还已消费的样本数（非虚：环形状态统一在 sample_stream_t 里，
     * 与具体 IMU 型号无关，派生类不需要也无法改变这个语义）。
     * 只前移 head 指针，O(1)，不搬数据；超出未消费数量时按实际数量处理。 */
    void assume_acc_point(uint16_t count) {
        _data.acc.release(count);
    }
    void assume_gyro_point(uint16_t count) {
        _data.gyro.release(count);
    }

    bool calibrate();
    virtual ~imu_base_drv();
    virtual bool init() = 0;

    /**
     * @brief 读取一拍数据并解包（仅限任务上下文）。
     *
     * 单一读取入口：内部按配置自行决定数据来源（器件 FIFO 批量 / 输出寄存器直读），
     * 调用方不需要知道走了哪条路径，也不需要感知"状态查询"这一步。
     * 典型实现为两段式：先发起状态事务、经完成中断唤醒后判读，再发起数据事务、
     * 再次唤醒后收取并解包。等待期间让出 CPU，不做轮询。
     *
     * @retval true  本拍解出了新样本（见 acc.available() / gyro.available()）
     * @retval false 本拍没有新数据，或超时 / 总线失败（区分方式见 diag()）
     *
     * @note  无数据时不得改变已有样本；缓冲满时【丢弃新样本】并计入 err_data。
     */
    virtual bool read() = 0;

    protected:
    imu_data_t _data;
};

#endif
