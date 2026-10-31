#include "pyro_buzzer_player.h"
#include "pyro_buzzer_drv.h"
#include "cmsis_os.h"
#include "task.h"

#define PLAYER_TICK_MS 10
#define BUZZER_DEFAULT_FREQ 4000UL

// ==================== 构造与析构 ====================
BuzzerPlayer::BuzzerPlayer()
    : beep_head_(nullptr)
    , beep_tail_(nullptr)
    , beep_count_(0)
    , tone_active_(false)
    , tone_freq_(0)
    , tone_volume_(100)
    , current_task_(nullptr)
    , state_(PlayerState::IDLE)
    , current_time_ms_(0)
    , initialized_(false)
{}

BuzzerPlayer::~BuzzerPlayer() {
    clearBeepQueue();
}

BuzzerPlayer& BuzzerPlayer::getInstance() {
    static BuzzerPlayer instance;
    return instance;
}

// ==================== 队列操作（私有成员） ====================
void BuzzerPlayer::clearBeepQueue() {
    BeepNode* node = beep_head_;
    while (node) {
        BeepNode* next = node->next;
        delete node;
        node = next;
    }
    beep_head_ = beep_tail_ = nullptr;
    beep_count_ = 0;
}

bool BuzzerPlayer::pushBeep(uint32_t freq, uint8_t volume, uint32_t duration_ms) {
    if (beep_count_ >= MAX_BEEP_QUEUE_SIZE) return false;

    BeepNode* node = new BeepNode{freq, volume, duration_ms, nullptr};
    if (!node) return false;

    taskENTER_CRITICAL();
    if (beep_tail_) {
        beep_tail_->next = node;
        beep_tail_ = node;
    } else {
        beep_head_ = beep_tail_ = node;
    }
    beep_count_++;
    taskEXIT_CRITICAL();
    return true;
}

bool BuzzerPlayer::popBeep(BeepNode& out) {
    taskENTER_CRITICAL();
    if (!beep_head_) {
        taskEXIT_CRITICAL();
        return false;
    }
    out = *beep_head_;
    BeepNode* old = beep_head_;
    beep_head_ = beep_head_->next;
    if (!beep_head_) beep_tail_ = nullptr;
    beep_count_--;
    taskEXIT_CRITICAL();
    delete old;
    return true;
}

BuzzerPlayer::BeepNode* BuzzerPlayer::peekBeep() {
    taskENTER_CRITICAL();
    BeepNode* front = beep_head_;
    taskEXIT_CRITICAL();
    return front;
}

void BuzzerPlayer::updateBeepFrontRemaining(uint32_t new_remaining) {
    taskENTER_CRITICAL();
    if (beep_head_) {
        beep_head_->remaining_ms = new_remaining;
    }
    taskEXIT_CRITICAL();
}

// ==================== init / deinit ====================
void BuzzerPlayer::init() {
    if (initialized_) return;
    BuzzerDriver::getInstance().init();
    clearBeepQueue();
    tone_active_ = false;
    tone_freq_ = 0;
    tone_volume_ = 100;
    current_task_ = nullptr;
    state_ = PlayerState::IDLE;
    current_time_ms_ = 0;
    initialized_ = true;

    xTaskCreate([](void*) {
        BuzzerPlayer::getInstance().mainloop();
        vTaskDelete(nullptr);
    }, "buzzer_player", 256, nullptr, configMAX_PRIORITIES - 2, nullptr);
}

void BuzzerPlayer::deinit() {
    if (!initialized_) return;
    stop();
    tone_stop();
    clearBeepQueue();
    BuzzerDriver::getInstance().deinit();
    initialized_ = false;
}

// ==================== 播放控制 ====================
void BuzzerPlayer::loadTask(BuzzerTask* task) {
    if (!initialized_) return;
    stop();
    current_task_ = task;
    if (task) {
        current_time_ms_ = 0;
        state_ = PlayerState::PLAYING;
        task->reset();
        // 只有在无 tone 且无 beep 时才立即更新硬件
        if (!tone_active_ && beep_head_ == nullptr) {
            updateHardware();
        }
    }
}

void BuzzerPlayer::pause() {
    if (state_ != PlayerState::PLAYING) return;
    state_ = PlayerState::PAUSED;
    BuzzerDriver::getInstance().set(0, 0);
}

void BuzzerPlayer::resume() {
    if (state_ != PlayerState::PAUSED) return;
    state_ = PlayerState::PLAYING;
}

void BuzzerPlayer::stop() {
    current_task_ = nullptr;
    state_ = PlayerState::IDLE;
    current_time_ms_ = 0;
    BuzzerDriver::getInstance().set(0, 0);
    // beep 队列保留，tone 不受影响
}

// ==================== 状态查询 ====================
// ==================== 状态查询 ====================
uint8_t BuzzerPlayer::getState() const {
    uint8_t status = 0;

    // ---- bit3: 音乐逻辑播放且任务存在 ----
    if (current_task_ != nullptr && state_ == PlayerState::PLAYING) {
        status |= BUZZER_STATE_MUSIC_BIT;
    }

    // ---- bit2: Tone 激活 ----
    if (tone_active_) {
        status |= BUZZER_STATE_TONE_BIT;
    }

    // ---- bit1: Beep 队列非空 ----
    if (beep_head_ != nullptr) {
        status |= BUZZER_STATE_BEEP_BIT;
    }

    // ---- bit0: 总状态（是否有实际音频输出） ----
    // 只要任一音源有效（音乐逻辑播放 / tone / beep），即认为有输出
    if ((status & BUZZER_STATE_BEEP_BIT) ||
        (status & BUZZER_STATE_TONE_BIT) ||
        (status & BUZZER_STATE_MUSIC_BIT)) {
        status |= BUZZER_STATE_TOTAL_BIT;
    }

    // ---- bit4: 音乐未暂停（state == PLAYING），0 表示暂停或空闲 ----
    if (state_ == PlayerState::PLAYING) {
        status |= BUZZER_STATE_MUSIC_PLAYING_BIT;
    }

    return status;
}
uint32_t BuzzerPlayer::getProgressMs() const { return current_time_ms_; }

// ==================== beep（非阻塞入队） ====================
bool BuzzerPlayer::beep(uint32_t freq, uint8_t volume, uint32_t duration_ms) {
    if (!initialized_) init();
    if (duration_ms == 0) return true;
    return pushBeep(freq, volume, duration_ms);
}

bool BuzzerPlayer::beep(uint8_t volume, uint32_t duration_ms) {
    return beep(BUZZER_DEFAULT_FREQ, volume, duration_ms);
}

// ==================== 持续音（Tone）控制 ====================
void BuzzerPlayer::tone_set(uint32_t freq, uint8_t volume) {
    tone_freq_ = freq;
    tone_volume_ = (volume > 100) ? 100 : volume;
}

void BuzzerPlayer::tone_start() {
    if (!initialized_) init();
    if (tone_freq_ == 0) return;
    tone_active_ = true;
    // 立即更新硬件（无需等待 mainloop）
    BuzzerDriver::getInstance().set(tone_freq_, tone_volume_);
}

void BuzzerPlayer::tone_stop() {
    tone_active_ = false;
    tone_freq_ = 0;
    tone_volume_ = 100;
    BuzzerDriver::getInstance().set(0, 0);
}

bool BuzzerPlayer::is_tone_playing() const {
    return tone_active_;
}

// ==================== updateHardware ====================
void BuzzerPlayer::updateHardware() {
    auto& driver = BuzzerDriver::getInstance();

    if (state_ != PlayerState::PLAYING || current_task_ == nullptr) {
        driver.set(0, 0);
        return;
    }

    uint32_t freq;
    uint8_t volume;
    bool still_playing = current_task_->get_state_at(current_time_ms_, freq, volume);
    if (!still_playing) {
        stop();
        return;
    }

    driver.set(freq, volume);
}

// ==================== mainloop ====================
void BuzzerPlayer::mainloop() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(PLAYER_TICK_MS));
        if (!initialized_) continue;

        auto& driver = BuzzerDriver::getInstance();

        // ---- 1. 音乐时间推进（无论是否有更高优先级音源） ----
        if (state_ == PlayerState::PLAYING && current_task_ != nullptr) {
            current_time_ms_ += PLAYER_TICK_MS;
            if (current_time_ms_ > current_task_->get_duration()) {
                current_time_ms_ = current_task_->get_duration();
            }
        }

        // ---- 2. 最高优先级：Beep 队列 ----
        BeepNode* front = peekBeep();
        if (front) {
            if (front->remaining_ms > PLAYER_TICK_MS) {
                driver.set(front->freq, front->volume);
                uint32_t new_remaining = front->remaining_ms - PLAYER_TICK_MS;
                updateBeepFrontRemaining(new_remaining);
            } else {
                driver.set(0, 0);
                BeepNode dummy;
                popBeep(dummy);
                // 如果队列空且无 tone，恢复音乐（或保持静音）
                if (beep_head_ == nullptr && !tone_active_ && state_ == PlayerState::PLAYING && current_task_ != nullptr) {
                    updateHardware();
                }
            }
            continue;  // 本 tick 结束，beep 优先级最高
        }

        // ---- 3. 次优先级：持续音（Tone） ----
        if (tone_active_) {
            driver.set(tone_freq_, tone_volume_);
            continue;  // 本 tick 结束
        }

        // ---- 4. 最低优先级：音乐任务 ----
        if (state_ == PlayerState::PLAYING && current_task_ != nullptr) {
            updateHardware();
        } else if (state_ == PlayerState::IDLE) {
            driver.set(0, 0);
        }
        // PAUSED 状态已在 pause() 中处理静音
    }
}
// ==================== reset ====================
void BuzzerPlayer::reset() {
    stop();
    clearBeepQueue();
    tone_stop();
    current_task_ = nullptr;
    state_ = PlayerState::IDLE;
    current_time_ms_ = 0;
}
//如此...组合吗...