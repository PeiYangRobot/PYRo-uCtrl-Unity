#ifndef __PYRO_MS53L0M_DRV_H__
#define __PYRO_MS53L0M_DRV_H__

#include "pyro_uart_drv.h"

namespace pyro {

/**
 * @brief ATK-MS53L0M 激光测距模块驱动 (Normal 工作模式)
 * @note 采用单例模式，通过 get_instance() 获取实例
 */
class ms53l0m_drv_t {
public:
    enum class range_state_t : uint8_t {
        VALID          = 0,   // 测量范围内
        SIGMA_FAIL     = 1,   // Sigma 错误
        SIGNAL_FAIL    = 2,   // 信号错误
        MIN_RANGE_FAIL = 3,   // 超出最小测量范围
        PHASE_FAIL     = 4,   // 超出最大测量范围
        HW_FAIL        = 5,   // 硬件错误
        RANGE_ERROR    = 6,   // 测量错误
        NO_UPDATE      = 255, // 无数据更新
        UNKNOWN        = 254  // 初始化默认状态
    };

    static ms53l0m_drv_t* get_instance();

    ~ms53l0m_drv_t();

    // 禁用拷贝与赋值
    ms53l0m_drv_t(const ms53l0m_drv_t&) = delete;
    ms53l0m_drv_t& operator=(const ms53l0m_drv_t&) = delete;

    status_t init();
    void get_data(uint32_t& distance_mm, range_state_t& state) const;
    [[nodiscard]] uint32_t get_distance() const;

private:
    // 构造函数私有化，禁止外部直接实例化
    explicit ms53l0m_drv_t(uart_drv_t& uart_instance);

    /**
     * @brief 内部接收回调解析函数，用于处理底层 DMA 抛出的数据
     */
    bool rx_callback(const uint8_t * p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken);

    uart_drv_t* _uart;

    volatile uint32_t _distance_mm;
    volatile range_state_t _state;
};

} // namespace pyro

#endif // __PYRO_MS53L0M_DRV_H__