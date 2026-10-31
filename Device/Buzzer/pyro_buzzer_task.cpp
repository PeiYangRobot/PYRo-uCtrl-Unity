#include "pyro_buzzer_task.h"
#include "pyro_buzzer_notes.h"
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <string>

// ========== 原有方法实现 ==========
BuzzerTask::BuzzerTask()
    : duration_(0)
    , last_index_(0)
{
}

void BuzzerTask::add_event(uint32_t time_ms, uint32_t freq, uint8_t volume)
{
    if (!events_.empty()) {
        if (time_ms == events_.back().time_ms) {
            events_.back().freq = freq;
            events_.back().volume = volume;
            return;
        }
    }
    events_.push_back({time_ms, freq, volume});
    duration_ = time_ms;
}

void BuzzerTask::set_duration(uint32_t duration_ms)
{
    duration_ = duration_ms;
}

void BuzzerTask::reset()
{
    last_index_ = 0;
}

bool BuzzerTask::get_state_at(uint32_t current_time_ms, uint32_t& out_freq, uint8_t& out_volume) const
{
    if (current_time_ms >= duration_) {
        out_freq = 0;
        out_volume = 0;
        return false;
    }

    if (events_.empty()) {
        out_freq = 0;
        out_volume = 0;
        return true;
    }

    size_t i = last_index_;
    if (i >= events_.size()) {
        i = events_.size() - 1;
    }

    if (events_[i].time_ms > current_time_ms) {
        i = 0;
    }

    while (i + 1 < events_.size() && events_[i + 1].time_ms <= current_time_ms) {
        ++i;
    }

    last_index_ = i;

    out_freq = events_[i].freq;
    out_volume = events_[i].volume;
    return true;
}

uint32_t BuzzerTask::get_duration() const
{
    return duration_;
}

size_t BuzzerTask::get_event_count() const
{
    return events_.size();
}

// ========== RTTTL 解析 ==========
static float note_name_to_freq(char note_letter, bool sharp, int octave) {
    const float base_freq_A4 = 440.0f;
    static const int semitone_offsets[7] = {-9, -7, -5, -4, -2, 0, 2};
    int idx = -1;
    switch (note_letter) {
        case 'c': idx = 0; break;
        case 'd': idx = 1; break;
        case 'e': idx = 2; break;
        case 'f': idx = 3; break;
        case 'g': idx = 4; break;
        case 'a': idx = 5; break;
        case 'b': idx = 6; break;
        default: return 0.0f;
    }
    int semitone = semitone_offsets[idx];
    if (sharp) semitone += 1;
    int octave_diff = octave - 4;
    float freq = base_freq_A4 * powf(2.0f, (float)(semitone + 12 * octave_diff) / 12.0f);
    return freq;
}

BuzzerTask* BuzzerTask::from_rtttl(const char* rtttl_string) {
    if (!rtttl_string) return nullptr;

    std::string str(rtttl_string);
    str.erase(0, str.find_first_not_of(" \t\n\r"));
    str.erase(str.find_last_not_of(" \t\n\r") + 1);

    size_t pos1 = str.find(':');
    if (pos1 == std::string::npos) return nullptr;
    size_t pos2 = str.find(':', pos1 + 1);
    if (pos2 == std::string::npos) return nullptr;

    std::string settings = str.substr(pos1 + 1, pos2 - pos1 - 1);
    std::string notes = str.substr(pos2 + 1);
    notes.erase(0, notes.find_first_not_of(" \t\n\r"));
    notes.erase(notes.find_last_not_of(" \t\n\r") + 1);

    // 默认值
    int default_duration = 4;   // 四分音符
    int default_octave = 5;
    int bpm = 120;

    // 解析设置：d=4, o=5, b=140
    size_t start = 0;
    while (start < settings.length()) {
        size_t comma = settings.find(',', start);
        std::string item = settings.substr(start, comma - start);
        item.erase(0, item.find_first_not_of(" \t"));
        item.erase(item.find_last_not_of(" \t") + 1);
        if (item.length() >= 3 && item[1] == '=') {
            char key = item[0];
            std::string val = item.substr(2);
            if (key == 'd') {
                default_duration = std::atoi(val.c_str());
            } else if (key == 'o') {
                default_octave = std::atoi(val.c_str());
            } else if (key == 'b') {
                bpm = std::atoi(val.c_str());
            }
        }
        if (comma == std::string::npos) break;
        start = comma + 1;
    }

    // 计算四分音符时长（毫秒）
    uint32_t quarter_duration_ms = (bpm > 0) ? (60000 / bpm) : 500;

    BuzzerTask* task = new BuzzerTask();

    uint32_t current_time = 0;
    const float gap_ratio = 0.1f;   // 音符间隙比例
    const uint32_t min_gap_ms = 10;

    // 解析音符序列
    start = 0;
    while (start < notes.length()) {
        size_t comma = notes.find(',', start);
        std::string note_str = notes.substr(start, comma - start);
        note_str.erase(0, note_str.find_first_not_of(" \t"));
        note_str.erase(note_str.find_last_not_of(" \t") + 1);

        if (note_str.empty()) {
            if (comma == std::string::npos) break;
            start = comma + 1;
            continue;
        }

        // 解析单个音符
        // 格式：[时长][音名][#][八度][.]
        // 例如：8g5, c, 8f., 16c#6
        int note_duration = default_duration;   // 当前音符时长分母
        int octave = default_octave;
        char note_letter = '\0';
        bool sharp = false;
        bool dotted = false;

        size_t idx = 0;

        // 1. 解析可选的时长数字（1, 2, 4, 8, 16, 32）
        int num = 0;
        while (idx < note_str.length() && std::isdigit(note_str[idx])) {
            num = num * 10 + (note_str[idx] - '0');
            idx++;
        }
        if (num > 0) {
            // 数字表示时长分母（如 8 表示八分音符）
            note_duration = num;
        }

        // 2. 解析音名（c, d, e, f, g, a, b, p）
        if (idx < note_str.length()) {
            char ch = std::tolower(note_str[idx]);
            if (ch >= 'a' && ch <= 'g') {
                note_letter = ch;
                idx++;
            } else if (ch == 'p') {
                // p 表示停顿（静音）
                note_letter = 'p';
                idx++;
            } else {
                // 无效音符，跳过
                if (comma == std::string::npos) break;
                start = comma + 1;
                continue;
            }
        } else {
            // 没有音名，跳过
            if (comma == std::string::npos) break;
            start = comma + 1;
            continue;
        }

        // 3. 解析可选的升号 #
        if (idx < note_str.length() && note_str[idx] == '#') {
            sharp = true;
            idx++;
        }

        // 4. 解析可选的八度数字（0-8）
        if (idx < note_str.length() && std::isdigit(note_str[idx])) {
            octave = note_str[idx] - '0';
            idx++;
        }

        // 5. 解析可选的附点 .
        if (idx < note_str.length() && note_str[idx] == '.') {
            dotted = true;
            idx++;
        }

        // 计算音符时长（毫秒）
        // 时长分母：2=二分音符, 4=四分音符, 8=八分音符, 16=十六分音符
        uint32_t note_len_ms = (quarter_duration_ms * 4) / note_duration;
        float duration_mult = dotted ? 1.5f : 1.0f;
        uint32_t note_duration_ms = (uint32_t)(note_len_ms * duration_mult);

        // 计算频率
        uint32_t freq = 0;
        if (note_letter != 'p') {
            // 音名转频率
            float freq_float = note_name_to_freq(note_letter, sharp, octave);
            if (freq_float >= 1.0f) {
                freq = (uint32_t)(freq_float + 0.5f);
            }
        }
        // 停顿（p）或频率为0时，直接静音

        // 1. 添加音符开始事件
        task->add_event(current_time, freq, 100);

        // 2. 计算音符结束时间，插入静音
        uint32_t note_end_time = current_time + note_duration_ms;
        task->add_event(note_end_time, 0, 0);

        // 3. 计算间隙（仅对非静音音符）
        uint32_t gap_ms = (freq > 0) ? (uint32_t)(note_duration_ms * gap_ratio) : 0;
        if (gap_ms < min_gap_ms) gap_ms = min_gap_ms;

        // 4. 推进到下一个音符的开始时间
        current_time = note_end_time + gap_ms;

        if (comma == std::string::npos) break;
        start = comma + 1;
    }

    // 结束静音
    task->add_event(current_time, 0, 0);
    task->set_duration(current_time);

    return task;
}

// ========== 从数组构建 ==========
BuzzerTask* BuzzerTask::from_arrays(const uint32_t freqs[], const uint32_t durations_ms[], size_t count) {
    if (!freqs || !durations_ms || count == 0) return nullptr;

    BuzzerTask* task = new BuzzerTask();
    if (!task) return nullptr;

    uint32_t current_time = 0;
    const float gap_ratio = 0.1f;
    const uint32_t min_gap_ms = 10;

    for (size_t i = 0; i < count; ++i) {
        uint32_t freq = freqs[i];
        uint32_t duration = durations_ms[i];

        if (duration == 0) continue;   // 跳过零时长音符

        // 1. 添加音符开始事件（音量固定为 100）
        task->add_event(current_time, freq, 100);

        // 2. 音符结束时间（插入静音）
        uint32_t note_end = current_time + duration;
        task->add_event(note_end, 0, 0);

        // 3. 计算间隙（仅对非静音音符）
        uint32_t gap = (freq > 0) ? (uint32_t)(duration * gap_ratio) : 0;
        if (gap < min_gap_ms) gap = min_gap_ms;

        // 4. 推进到下一个音符开始时间
        current_time = note_end + gap;
    }

    // 末尾再添加一个静音事件，确保任务结束
    task->add_event(current_time, 0, 0);
    task->set_duration(current_time);

    return task;
}


// ========== 从节奏模式构建 ==========
BuzzerTask* BuzzerTask::from_rhythm(const uint8_t pattern[], size_t length, uint32_t gap_ms, uint32_t freq) {
    if (!pattern || length == 0) return nullptr;

    BuzzerTask* task = new BuzzerTask();
    if (!task) return nullptr;

    const uint32_t unit_ms = BUZZER_RHYTHM_UNIT_MS;
    const uint32_t short_gap = BUZZER_RHYTHM_SHORT_GAP_MS;
    uint32_t current_time = 0;

    for (size_t i = 0; i < length; ++i) {
        uint8_t count = pattern[i];
        // 生成 count 个连续的“哔”
        for (uint8_t j = 0; j < count; ++j) {
            // 1. 添加音符开始事件（音量固定为 100）
            task->add_event(current_time, freq, 100);

            // 2. 音符结束时间（插入静音）
            uint32_t note_end = current_time + unit_ms;
            task->add_event(note_end, 0, 0);

            // 3. 推进到下一个音符（加上短间隔）
            current_time = note_end + short_gap;
        }

        // 4. 段结束后，如果还有下一段，插入段间间隔
        if (i < length - 1) {
            // 段间间隔直接推进时间（当前已经是静音状态）
            current_time += gap_ms;
        }
    }

    // 末尾添加静音事件，确保任务结束
    task->add_event(current_time, 0, 0);
    task->set_duration(current_time);

    return task;
}