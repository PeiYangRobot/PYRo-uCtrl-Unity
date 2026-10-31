#ifndef PYRO_BUZZER_DRV_H
#define PYRO_BUZZER_DRV_H

#include <stdint.h>

/**
 * @brief 蜂鸣器驱动类（单例）
 * @note  驱动层假设用户已通过 CubeMX 完成 TIM12 和 GPIO 的初始化
 *        驱动只做增量控制（不覆盖任何已有配置）
 */
class BuzzerDriver {
public:
    static BuzzerDriver& getInstance();

    // 初始化（仅标记状态，不修改任何寄存器）
    void init();

    // 反初始化（停止输出，释放资源）
    void deinit();

    // 设置频率（Hz），0 表示停止输出
    void setFreq(uint32_t freq);

    // 设置音量（0~100），内部映射为占空比（0~50%）
    void setVolume(uint8_t volume);

    // 同时设置频率和音量
    void set(uint32_t freq, uint8_t volume);

    // 状态查询
    uint32_t getFreq() const;
    uint8_t  getVolume() const;
    bool     isPlaying() const;

private:
    BuzzerDriver();
    ~BuzzerDriver() = default;
    BuzzerDriver(const BuzzerDriver&) = delete;
    BuzzerDriver& operator=(const BuzzerDriver&) = delete;

    void updateHardware();

    uint32_t freq_;
    uint8_t  volume_;
    bool     initialized_;
};

#endif