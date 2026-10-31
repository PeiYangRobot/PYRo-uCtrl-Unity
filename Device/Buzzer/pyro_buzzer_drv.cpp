#include "pyro_buzzer_drv.h"
#include "stm32h7xx.h"
#define CNT_CLOCK 1500000UL  // 计数时钟频率（与 CubeMX 配置一致，PSC=159，计数时钟=1.5MHz）
// ==================== 构造与单例 ====================
BuzzerDriver::BuzzerDriver() : freq_(0), volume_(0), initialized_(false) {}

BuzzerDriver& BuzzerDriver::getInstance() {
    static BuzzerDriver instance;
    return instance;
}

// ==================== 初始化（仅标记状态，不做任何硬件修改） ====================
void BuzzerDriver::init() {
    if (initialized_) return;
    // 假设用户已通过 CubeMX 完成 TIM12 和 GPIO 的初始化
    // 我们只记录状态，不修改任何寄存器
    freq_ = 0;
    volume_ = 0;
    initialized_ = true;
}

// ==================== 反初始化（停止输出，释放资源） ====================
void BuzzerDriver::deinit() {
    if (!initialized_) return;
    // 停止输出（只清空我们控制的部分，不影响其他通道）
    TIM12->CCER &= ~TIM_CCER_CC2E;   // 关闭输出
    TIM12->CCR2 = 0;                 // 清除比较值
    freq_ = 0;
    volume_ = 0;
    initialized_ = false;
}

// ==================== 更新硬件（增量控制，不覆盖已有配置） ====================
void BuzzerDriver::updateHardware() {
    if (!initialized_) return;

    // 如果频率或音量为0，直接静音（不关闭通道，只清CCR）
    if (freq_ == 0 || volume_ == 0) {
        TIM12->CCR2 = 0;
        // 确保输出使能保持（如果被意外关闭，重新打开）
        TIM12->CCER |= TIM_CCER_CC2E;
        TIM12->CR1 |= TIM_CR1_CEN;
        return;
    }
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
    // 计数时钟（假设与 CubeMX 配置一致：PSC=159，计数时钟=1.5MHz）
    // 如果用户修改了 PSC，需同步调整此值
    const uint32_t cnt_clock = CNT_CLOCK;
/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//怕你找不到多加点标记,我知道这不优雅,但是...暂时这样吧,除了加def没想到什么好方法
//参数,预分频159,剩下的都可以现场配置

    // 计算 ARR
    uint32_t arr = (cnt_clock / freq_) - 1;
    if (arr > 65535) arr = 65535;
    if (arr < 1) arr = 1;

    // 音量 → 占空比（0~50% 线性映射）
    uint8_t duty = (volume_ >= 100) ? 50 : (volume_ * 50) / 100;
    uint32_t ccr = (arr * duty) / 100;
    if (ccr > arr) ccr = arr;

    // 增量写入寄存器（只修改我们需要的位）
    TIM12->ARR = arr;
    TIM12->CCR2 = ccr;
    TIM12->EGR |= TIM_EGR_UG;      // 关键：立即更新
    TIM12->CR1 |= TIM_CR1_CEN;      // 确保计数器运行（置位，不覆盖）
    TIM12->CCER |= TIM_CCER_CC2E;   // 确保输出使能（置位，不覆盖）
}

// ==================== 设置接口 ====================
void BuzzerDriver::setFreq(uint32_t freq) {
    if (!initialized_) return;
    freq_ = freq;
    updateHardware();
}

void BuzzerDriver::setVolume(uint8_t volume) {
    if (!initialized_) return;
    if (volume > 100) volume = 100;
    volume_ = volume;
    updateHardware();
}

void BuzzerDriver::set(uint32_t freq, uint8_t volume) {
    if (!initialized_) return;
    freq_ = freq;
    if (volume > 100) volume = 100;
    volume_ = volume;
    updateHardware();
}

// ==================== 状态查询 ====================
uint32_t BuzzerDriver::getFreq() const { return freq_; }
uint8_t  BuzzerDriver::getVolume() const { return volume_; }

bool BuzzerDriver::isPlaying() const {
    if (!initialized_) return false;
    // 读取实际硬件状态
    return (TIM12->CCER & TIM_CCER_CC2E) && (TIM12->CCR2 > 0);
}