#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "pyro_core_def.h"
#include "ws2812_protocol.h"
#include "ws2812_middleware.h"

namespace pyro {

/* ============================================================
 *  WS2812 LED驱动 (多实例, 支持多路PWM同时驱动)
 *
 *  用法:
 *    ws2812_t led1(WS2812_HW_TIM1_CH1);  // 第1路
 *    ws2812_t led2(WS2812_HW_TIM3_CH2);  // 第2路
 *    led1.init(8);   led1.set_all(255,0,0); led1.update();
 *    led2.init(16);  led2.set_all(0,255,0); led2.update();
 *
 *  注意: CubeMX中对应TIM通道的DMA数据宽度必须配置为 Half Word
 * ============================================================ */
class ws2812_t
{
public:
    /* 构造: 传入硬件配置 (定时器句柄+通道+DMA ID) */
    explicit ws2812_t(const ws2812_hw_t& hw_config);
    ~ws2812_t(void);

    /* 禁止拷贝 (每个实例独占硬件资源) */
    ws2812_t(const ws2812_t&)            = delete;
    ws2812_t& operator=(const ws2812_t&) = delete;

    /* ---------- 生命周期 ---------- */
    status_t init(uint16_t led_num);
    status_t reset_led(uint16_t led_num);   /* 重新初始化,改变LED数量 */
    status_t reset(void);                   /* 重新初始化硬件,LED数量不变 */
    void     deinit(void);

    /* ---------- 单颗LED设置 ---------- */
    status_t set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    status_t set_led(uint16_t index, const ws2812_color_t& color);
    [[nodiscard]] ws2812_color_t get_led(uint16_t index) const;

    /* ---------- 批量设置 ---------- */
    status_t set_all(uint8_t r, uint8_t g, uint8_t b);
    status_t set_range(uint16_t start, uint16_t end, uint8_t r, uint8_t g, uint8_t b);
    status_t set_buffer(const ws2812_color_t* colors, uint16_t count);
    status_t clear(void);

    /* ---------- 发送显示 ---------- */
    status_t update(void);           /* 阻塞发送 */
    status_t update_async(void);     /* 非阻塞发送 */
    [[nodiscard]] bool is_busy(void) const;

    /* ---------- 亮度 ---------- */
    void    set_brightness(uint8_t brightness);
    [[nodiscard]] uint8_t get_brightness(void) const;

    /* ---------- HSV支持 ---------- */
    static ws2812_color_t hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v);
    status_t set_led_hsv(uint16_t index, uint16_t h, uint8_t s, uint8_t v);
    status_t set_all_hsv(uint16_t h, uint8_t s, uint8_t v);

    /* ---------- 动态效果 ---------- */
    status_t gradient(uint16_t start, uint16_t end,
                      const ws2812_color_t& color1, const ws2812_color_t& color2);
    status_t rainbow(uint16_t start_hue, uint8_t density);
    status_t breathing(const ws2812_color_t& color, uint8_t step);
    status_t chase(const ws2812_color_t& color, uint16_t position, uint8_t length);
    status_t twinkle(const ws2812_color_t& color, uint8_t probability);
    status_t color_wipe(const ws2812_color_t& color, uint16_t position);

    /* ---------- 查询 ---------- */
    [[nodiscard]] uint16_t get_num_leds(void) const  { return _led_num; }
    [[nodiscard]] bool     is_initialized(void) const { return _initialized; }

    /* ---------- 测试 ---------- */
    void test_pattern(void);
    void self_test(void);

    /* ---------- 内部回调 (由中断调用, 不要手动调用) ---------- */
    void _on_dma_complete(void);

    /* 静态: 根据定时器句柄查找正在传输的实例 (用于回调分发) */
    static ws2812_t* find_busy_instance(TIM_HandleTypeDef* htim);

private:
    /* 硬件配置 */
    ws2812_hw_t _hw;

    /* 状态 */
    uint16_t  _led_num;
    bool      _initialized;
    volatile bool _busy;
    uint8_t   _brightness;

    /* 定时器参数 */
    uint32_t _tim_clk;
    uint32_t _psc;
    uint32_t _arr;
    uint32_t _pwm_freq;
    float    _pwm_period_us;
    uint8_t  _code0_ccr;
    uint8_t  _code1_ccr;

    /* 呼吸灯状态 */
    uint8_t  _breath_brightness;
    bool     _breath_direction;

    /* 缓冲区 (从DMA堆分配, H7的DTCM不能被DMA访问) */
    uint8_t*  _color_buf;    /* GRB颜色缓冲区, 大小 = led_num × 3 */
    uint16_t* _pulse_buf;    /* PWM脉冲缓冲区, uint16_t匹配DMA Half Word */

    /* ---------- 内部方法 ---------- */
    uint32_t _calc_tim_clock(void);
    void     _calc_pwm_params(void);
    void     _convert_color_to_pulse(void);
    bool     _allocate_buffers(uint16_t led_num);
    void     _free_buffers(void);

    /* ---------- 静态实例注册表 ---------- */
    static constexpr uint8_t MAX_INSTANCES = 4;  /* 最多支持4路 */
    static ws2812_t* _instances[MAX_INSTANCES];
    static uint8_t    _instance_count;

    void _register_instance(void);
    void _unregister_instance(void);
};

} // namespace pyro

#endif // WS2812_DRIVER_H
