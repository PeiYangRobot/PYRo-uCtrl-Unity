# 蜂鸣器模块快速使用指南(by deepseek)

## 1. 前置条件（CubeMX 配置）

在使用蜂鸣器模块之前，**用户必须**在 CubeMX 中完成以下配置：

| 配置项 | 设置 |
|--------|------|
| 定时器 | **TIM12** 启用，时钟源为内部时钟 |
| 通道 | **TIM12_CH2** 启用，PWM 模式 1 |
| GPIO | **PB15** 复用为 TIM12_CH2，推挽输出 |
| 预分频器 (Prescaler) | **159** |
| 自动重载预装载 (ARPE) | **Enable**（关键！） |
| 初始脉冲 (Pulse) | **0**（上电静音） |

> 其他 TIM12 参数（如周期、计数器模式等）可使用默认值，模块运行时会动态修改。

---

## 2. 初始化

在系统初始化位置（如 `pyro_init_thread` 或 `main`）调用：

```cpp
#include "pyro_buzzer_player.h"

void system_init() {
    BuzzerPlayer::getInstance().init();
}
```

`init()` 会自动完成：
- 驱动层初始化（假设硬件已由 CubeMX 配置）
- 创建 mainloop 任务（后台调度）

---

## 3. 基本用法

### 3.1 播放提示音（非阻塞）

```cpp
// 播放 440Hz，音量 100%，持续 300ms（立即返回，后台播放）
BuzzerPlayer::getInstance().beep(440, 100, 300);

// 使用默认频率 4000Hz
BuzzerPlayer::getInstance().beep(80, 200);  // 80% 音量，200ms
```

**特点**：
- 调用后立即返回，不阻塞调用者
- 支持队列（最多 16 个请求），按顺序播放
- 优先级最高，会打断音乐任务

### 3.2 播放旋律（从 RTTTL 字符串）

```cpp
BuzzerTask* task = BuzzerTask::from_rtttl(
    "Twinkle:d=4,o=5,b=120:8c,8c,8g,8g,8a,8a,4g"
);
if (task) {
    BuzzerPlayer::getInstance().loadTask(task);
}
```

### 3.3 播放旋律（从频率数组）

```cpp
uint32_t freqs[] = {262, 294, 330, 349, 392, 440, 494, 523};
uint32_t durations[] = {300, 300, 300, 300, 300, 300, 300, 300};
size_t count = sizeof(freqs) / sizeof(freqs[0]);

BuzzerTask* task = BuzzerTask::from_arrays(freqs, durations, count);
if (task) {
    BuzzerPlayer::getInstance().loadTask(task);
}
```

### 3.4 使用预定义音符

```cpp
#include "pyro_buzzer_notes.h"

// 播放 A4 (440Hz)
BuzzerPlayer::getInstance().beep(BUZZER_GET_FREQ(BUZZER_NOTE_A4), 100, 500);
```

---

## 4. 播放控制

| 方法 | 功能 |
|------|------|
| `pause()` | 暂停当前音乐播放 |
| `resume()` | 恢复音乐播放 |
| `stop()` | 停止音乐并清空任务 |

```cpp
auto& player = BuzzerPlayer::getInstance();
player.pause();   // 暂停
player.resume();  // 恢复
player.stop();    // 停止
```

---

## 5. 持续音（手动控制开关）

```cpp
auto& player = BuzzerPlayer::getInstance();

// 设置参数（不播放）
player.tone_set(440, 100);

// 开始播放（立即发声）
player.tone_start();

// 改变频率（先设置，再启动即可无缝切换）
player.tone_set(880, 80);
player.tone_start();

// 停止
player.tone_stop();

// 查询状态
if (player.is_tone_playing()) {
    // ...
}
```

---

## 6. 状态查询

```cpp
auto& player = BuzzerPlayer::getInstance();

// 当前状态：IDLE / PLAYING / PAUSED
PlayerState state = player.getState();

// 当前播放进度（毫秒）
uint32_t progress = player.getProgressMs();
```

---

## 7. 完整示例

```cpp
#include "pyro_buzzer_player.h"
#include "pyro_buzzer_task.h"
#include "pyro_buzzer_notes.h"

void test_usage() {
    auto& player = BuzzerPlayer::getInstance();

    // 1. 初始化
    player.init();

    // 2. 播放提示音（非阻塞）
    player.beep(440, 100, 300);

    // 3. 播放旋律（RTTTL）
    const char* rtttl = "Scale:d=4,o=5,b=120:8c,8d,8e,8f,8g,8a,8b,8c6";
    BuzzerTask* task = BuzzerTask::from_rtttl(rtttl);
    if (task) {
        player.loadTask(task);
    }

    // 4. 等待 5 秒后暂停
    vTaskDelay(pdMS_TO_TICKS(5000));
    player.pause();

    // 5. 等待 2 秒后恢复
    vTaskDelay(pdMS_TO_TICKS(2000));
    player.resume();

    // 6. 停止
    player.stop();

    // 7. 持续音
    player.tone_set(880, 100);
    player.tone_start();
    vTaskDelay(pdMS_TO_TICKS(1000));
    player.tone_stop();
}
```

---

## 8. 注意事项

| 要点 | 说明 |
|------|------|
| **硬件配置** | CubeMX 必须预先配置 TIM12 和 PB15，否则模块无效 |
| **非阻塞 beep** | `beep()` 调用后立即返回，声音由后台 mainloop 任务播放 |
| **任务生命周期** | `BuzzerTask` 由调用方管理，播放完成后需自行 `delete` |
| **队列长度** | beep 队列最大 16 个，超出会丢弃新请求 |
| **优先级** | Beep > Tone > 音乐任务 |
| **暂停影响** | 暂停仅影响音乐任务，tone 和 beep 不受影响 |

---

## 9. API 快速参考

| 模块 | 方法 | 功能 |
|------|------|------|
| **播放器** | `init()` | 初始化 |
| | `deinit()` | 反初始化 |
| | `beep(freq, vol, ms)` | 非阻塞提示音 |
| | `beep(vol, ms)` | 默认频率 4000Hz |
| | `loadTask(task)` | 加载旋律任务 |
| | `pause()` / `resume()` | 暂停/继续 |
| | `stop()` | 停止播放 |
| | `tone_set(freq, vol)` | 设置持续音参数 |
| | `tone_start()` | 开始持续音 |
| | `tone_stop()` | 停止持续音 |
| | `is_tone_playing()` | 查询持续音状态 |
| | `getState()` | 获取播放状态 |
| | `getProgressMs()` | 获取进度 |
| **任务** | `add_event(ms, freq, vol)` | 添加突变点 |
| | `set_duration(ms)` | 设置总时长 |
| | `from_rtttl(str)` | 解析 RTTTL 字符串 |
| | `from_arrays(freqs, durs, n)` | 从数组构建 |
| **音符** | `BUZZER_NOTE_XXX` | 88 键枚举 |
| | `BUZZER_GET_FREQ(note)` | 获取频率 |

---

现在你可以从零开始使用蜂鸣器模块了！ 
</BuzzerPlayer>