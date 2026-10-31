#ifndef PYRO_BUZZER_PLAYER_H
#define PYRO_BUZZER_PLAYER_H

#include <stdint.h>
#include "pyro_buzzer_task.h"

// 状态位宏定义（用于 getState() 返回值）
#define BUZZER_STATE_TOTAL_BIT          (0x01 << 0)  // 总状态：是否有音频输出（任意音源）
#define BUZZER_STATE_BEEP_BIT           (0x01 << 1)  // Beep 队列非空
#define BUZZER_STATE_TONE_BIT           (0x01 << 2)  // Tone 持续音激活
#define BUZZER_STATE_MUSIC_BIT          (0x01 << 3)  // 音乐逻辑播放中（任务存在且 state == PLAYING）
#define BUZZER_STATE_MUSIC_PLAYING_BIT  (0x01 << 4)  // 音乐未暂停（state == PLAYING），0 表示暂停或空闲

enum class PlayerState {
    IDLE,       // 无任务或已结束
    PLAYING,    // 播放中
    PAUSED      // 暂停
};

/**
 * @brief 蜂鸣器播放器（软件层）
 * @note  管理任务播放、暂停/继续、进度控制；支持非阻塞 beep 队列和持续音
 */
class BuzzerPlayer {
public:
    static BuzzerPlayer& getInstance();

    void init();
    void deinit();

    // ===== 播放控制 =====
    void loadTask(BuzzerTask* task);
    void pause();
    void resume();
    void stop();

    // ===== 状态查询 =====
    uint8_t getState() const;   // 返回位掩码，使用上述宏提取
    uint32_t getProgressMs() const;

    // ===== 非阻塞 Beep（入队） =====
    bool beep(uint32_t freq, uint8_t volume, uint32_t duration_ms);
    bool beep(uint8_t volume, uint32_t duration_ms);  // 默认频率 4000Hz

    // ===== 持续音（Tone）控制 =====
    // 设置持续音参数（不立即播放）
    void tone_set(uint32_t freq, uint8_t volume = 100);
    // 开始播放持续音（使用 tone_set 设置的参数）
    void tone_start();
    // 停止持续音
    void tone_stop();
    // 查询是否正在播放持续音
    bool is_tone_playing() const;

    // 增加一个重置方法,重置整个播放器,释放掉所有它持有的资源,包括任务和beep队列,持续音等
    void reset();

    // mainloop
    void mainloop();

private:
    BuzzerPlayer();
    ~BuzzerPlayer();
    BuzzerPlayer(const BuzzerPlayer&) = delete;
    BuzzerPlayer& operator=(const BuzzerPlayer&) = delete;

    // ---- 内部队列操作 ----
    struct BeepNode {
        uint32_t freq;
        uint8_t volume;
        uint32_t remaining_ms;
        BeepNode* next;
    };

    bool pushBeep(uint32_t freq, uint8_t volume, uint32_t duration_ms);
    bool popBeep(BeepNode& out);
    BeepNode* peekBeep();// 获取队列头（不出队）//好吧你赢了
    void updateBeepFrontRemaining(uint32_t new_remaining);
    void clearBeepQueue();

    void updateHardware();   // 根据当前状态更新驱动

    // 队列头尾
    BeepNode* beep_head_;
    BeepNode* beep_tail_;
    size_t beep_count_;

    // 持续音状态
    bool tone_active_;
    uint32_t tone_freq_;
    uint8_t tone_volume_;

    BuzzerTask* current_task_;
    PlayerState state_;
    uint32_t current_time_ms_;
    bool initialized_;

    static const size_t MAX_BEEP_QUEUE_SIZE = 16;
};

#endif // PYRO_BUZZER_PLAYER_H

//软件层在哪里持有任务的实例?
//软件层的任务实例是由用户创建并传入的,播放器只持有指针,不会管理内存
//行号?
//行号是指代码中的行号吗?
//是的
//行号是指代码中的行号吗?
//你怎么也魔了,直接打印行号就行,我只是懒得找了
//算了算了我自己找