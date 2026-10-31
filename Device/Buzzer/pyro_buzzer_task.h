#ifndef PYRO_BUZZER_TASK_H
#define PYRO_BUZZER_TASK_H

#include <stdint.h>
#include <vector>
#include <cstddef>   // for size_t

// 节奏型相关宏
#define BUZZER_RHYTHM_UNIT_MS       100   // 一个“哔”的基本时长（毫秒）
#define BUZZER_RHYTHM_FREQ          4000  // 默认频率（Hz）
#define BUZZER_RHYTHM_SHORT_GAP_MS  50    // 音符间短间隔（毫秒）

struct BuzzerEvent {
    uint32_t time_ms;
    uint32_t freq;
    uint8_t volume;
};

class BuzzerTask {
public:
    BuzzerTask();

    void add_event(uint32_t time_ms, uint32_t freq, uint8_t volume);
    void set_duration(uint32_t duration_ms);
    void reset();
    bool get_state_at(uint32_t current_time_ms, uint32_t& out_freq, uint8_t& out_volume) const;
    uint32_t get_duration() const;
    size_t get_event_count() const;

    // ===== RTTTL 解析（工厂方法） =====
    static BuzzerTask* from_rtttl(const char* rtttl_string);

    // ===== 从数组构建（工厂方法） =====
    // freqs[]: 频率数组 (Hz)，0 表示静音
    // durations_ms[]: 每个音符的持续时间 (毫秒)
    // count: 数组元素个数
    // 每个音符后会自动添加间隙（音符时长的 10%，最小 10ms）
    static BuzzerTask* from_arrays(const uint32_t freqs[], const uint32_t durations_ms[], size_t count);
    // ===== 从节奏模式构建（工厂方法） =====
    // pattern[]: 每个元素表示连续“哔”的次数
    // length: 数组长度
    // gap_ms: 每段之间的间隔（毫秒）
    // freq: 发声频率，默认为 BUZZER_RHYTHM_FREQ
    static BuzzerTask* from_rhythm(const uint8_t pattern[], size_t length, uint32_t gap_ms, uint32_t freq = BUZZER_RHYTHM_FREQ);

private:
    std::vector<BuzzerEvent> events_;
    uint32_t duration_;
    mutable size_t last_index_;
};

#endif // PYRO_BUZZER_TASK_H

//这里不太好,如果有不同的类型,无法用阶跃来描述那种,这个结构就失效了
//但是考虑到如果使用不同的实例,那调用时需要反复申请和释放,这在嵌入式上不太好,所以暂时还是用这个结构吧,如果有更好的方法再改
//暂时用不同的生成方法来实现吧
//ds魔了,工厂在哪里