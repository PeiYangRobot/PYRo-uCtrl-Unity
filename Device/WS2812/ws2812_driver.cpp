#include "ws2812_driver.h"
#include "pyro_core_def.h"
#include "pyro_core_dma_heap.h"   /* PYRo DMA堆: 脉冲缓冲区必须从DMA堆分配 */
#include "pyro_dwt_drv.h"         /* PYRo DWT驱动: 随机数种子 */
#include <string.h>
#include <stdlib.h>

namespace pyro {

/* ============================================================
 *  静态成员初始化
 * ============================================================ */
ws2812_t* ws2812_t::_instances[ws2812_t::MAX_INSTANCES] = {nullptr};
uint8_t   ws2812_t::_instance_count = 0;

/* ============================================================
 *  构造 / 析构
 * ============================================================ */
ws2812_t::ws2812_t(const ws2812_hw_t& hw_config)
    : _hw(hw_config)
    , _led_num(0)
    , _initialized(false)
    , _busy(false)
    , _brightness(255)
    , _tim_clk(0)
    , _psc(0)
    , _arr(0)
    , _pwm_freq(0)
    , _pwm_period_us(0.0f)
    , _code0_ccr(0)
    , _code1_ccr(0)
    , _breath_brightness(0)
    , _breath_direction(true)
    , _color_buf(nullptr)
    , _pulse_buf(nullptr)
{
    _register_instance();
}

ws2812_t::~ws2812_t(void)
{
    deinit();
    _unregister_instance();
}

/* ============================================================
 *  实例注册 / 注销
 * ============================================================ */
void ws2812_t::_register_instance(void)
{
    if (_instance_count < MAX_INSTANCES) {
        _instances[_instance_count++] = this;
    }
}

void ws2812_t::_unregister_instance(void)
{
    for (uint8_t i = 0; i < _instance_count; i++) {
        if (_instances[i] == this) {
            /* 前移填补空位 */
            for (uint8_t j = i; j < _instance_count - 1; j++) {
                _instances[j] = _instances[j + 1];
            }
            _instances[_instance_count - 1] = nullptr;
            _instance_count--;
            break;
        }
    }
}

/* 查找指定定时器上正在传输的实例 (用于DMA回调分发) */
ws2812_t* ws2812_t::find_busy_instance(TIM_HandleTypeDef* htim)
{
    for (uint8_t i = 0; i < _instance_count; i++) {
        if (_instances[i]->_hw.htim == htim && _instances[i]->_busy) {
            return _instances[i];
        }
    }
    return nullptr;
}

/* ============================================================
 *  生命周期
 * ============================================================ */
status_t ws2812_t::init(uint16_t led_num)
{
    if (led_num == 0) {
        return PYRO_PARAM_ERROR;
    }
    if (_initialized) {
        deinit();
    }

    _tim_clk = _calc_tim_clock();
    _calc_pwm_params();

    /* PWM频率校验: WS2812要求800kHz左右, 容差±25% */
    if (_pwm_freq < 600000 || _pwm_freq > 1000000) {
        return PYRO_ERROR;
    }

    if (!_allocate_buffers(led_num)) {
        return PYRO_NO_MEMORY;
    }
    memset(_color_buf, 0, WS2812_COLOR_BUF_SIZE(led_num));

    _led_num = led_num;
    _initialized = true;
    _busy = false;

    /* 用DWT周期计数器作为随机数种子 */
    srand(pyro::dwt_drv_t::get_current_ticks());

    ws2812_hw_init(&_hw);

    return PYRO_OK;
}

status_t ws2812_t::reset_led(uint16_t led_num)
{
    deinit();
    return init(led_num);
}

status_t ws2812_t::reset(void)
{
    if (!_initialized) {
        return PYRO_ERROR;
    }

    ws2812_hw_deinit(&_hw);
    _busy = false;

    ws2812_hw_init(&_hw);

    uint32_t tim_clk, psc, arr;
    ws2812_hw_get_tim_config(&_hw, &tim_clk, &psc, &arr);
    _tim_clk = tim_clk;
    _psc = psc;
    _arr = arr;
    _calc_pwm_params();

    return PYRO_OK;
}

void ws2812_t::deinit(void)
{
    if (!_initialized) {
        return;
    }

    ws2812_hw_deinit(&_hw);
    _busy = false;
    _free_buffers();

    _led_num = 0;
    _initialized = false;
}

/* ============================================================
 *  单颗LED设置
 * ============================================================ */
status_t ws2812_t::set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (!_initialized) return PYRO_ERROR;
    if (index >= _led_num) return PYRO_PARAM_ERROR;

    /* 颜色缓冲区是GRB顺序 */
    uint32_t offset = (uint32_t)index * WS2812_BYTES_PER_LED;
    _color_buf[offset + WS2812_OFFSET_G] = g;
    _color_buf[offset + WS2812_OFFSET_R] = r;
    _color_buf[offset + WS2812_OFFSET_B] = b;

    return PYRO_OK;
}

status_t ws2812_t::set_led(uint16_t index, const ws2812_color_t& color)
{
    return set_led(index, color.r, color.g, color.b);
}

ws2812_color_t ws2812_t::get_led(uint16_t index) const
{
    ws2812_color_t color = WS2812_COLOR_BLACK;
    if (!_initialized || index >= _led_num) return color;

    uint32_t offset = (uint32_t)index * WS2812_BYTES_PER_LED;
    color.g = _color_buf[offset + WS2812_OFFSET_G];
    color.r = _color_buf[offset + WS2812_OFFSET_R];
    color.b = _color_buf[offset + WS2812_OFFSET_B];
    return color;
}

/* ============================================================
 *  批量设置
 * ============================================================ */
status_t ws2812_t::set_all(uint8_t r, uint8_t g, uint8_t b)
{
    if (!_initialized) return PYRO_ERROR;
    for (uint16_t i = 0; i < _led_num; i++) {
        set_led(i, r, g, b);
    }
    return PYRO_OK;
}

status_t ws2812_t::set_range(uint16_t start, uint16_t end,
                              uint8_t r, uint8_t g, uint8_t b)
{
    if (!_initialized) return PYRO_ERROR;
    if (start >= _led_num || end >= _led_num || start > end) return PYRO_PARAM_ERROR;
    for (uint16_t i = start; i <= end; i++) {
        set_led(i, r, g, b);
    }
    return PYRO_OK;
}

status_t ws2812_t::set_buffer(const ws2812_color_t* colors, uint16_t count)
{
    if (!_initialized) return PYRO_ERROR;
    if (colors == nullptr) return PYRO_PARAM_ERROR;
    uint16_t actual = (count < _led_num) ? count : _led_num;
    for (uint16_t i = 0; i < actual; i++) {
        set_led(i, colors[i].r, colors[i].g, colors[i].b);
    }
    return PYRO_OK;
}

status_t ws2812_t::clear(void)
{
    return set_all(0, 0, 0);
}

/* ============================================================
 *  发送显示
 * ============================================================ */
status_t ws2812_t::update(void)
{
    if (!_initialized) return PYRO_ERROR;
    if (_busy) return PYRO_BUSY;

    _convert_color_to_pulse();

    _busy = true;
    uint32_t total_pulses = WS2812_PULSE_BUF_SIZE(_led_num) + WS2812_RESET_PULSE_COUNT;
    ws2812_hw_start_dma(&_hw, (const uint8_t*)_pulse_buf, (uint16_t)total_pulses);

    /* 阻塞等待DMA传输完成 (超时保护) */
    uint32_t timeout = 0;
    while (_busy) {
        ws2812_hw_delay_us(10);
        if (++timeout > 2000) {
            _busy = false;
            ws2812_hw_stop_dma(&_hw);
            return PYRO_TIMEOUT;
        }
    }

    ws2812_hw_delay_us(WS2812_RESET_DELAY_US);
    return PYRO_OK;
}

status_t ws2812_t::update_async(void)
{
    if (!_initialized) return PYRO_ERROR;
    if (_busy) return PYRO_BUSY;

    _convert_color_to_pulse();

    _busy = true;
    uint32_t total_pulses = WS2812_PULSE_BUF_SIZE(_led_num) + WS2812_RESET_PULSE_COUNT;
    ws2812_hw_start_dma(&_hw, (const uint8_t*)_pulse_buf, (uint16_t)total_pulses);
    return PYRO_OK;
}

bool ws2812_t::is_busy(void) const
{
    return _busy;
}

void ws2812_t::_on_dma_complete(void)
{
    ws2812_hw_stop_dma(&_hw);
    _busy = false;
}

/* ============================================================
 *  亮度
 * ============================================================ */
void ws2812_t::set_brightness(uint8_t brightness) { _brightness = brightness; }
uint8_t ws2812_t::get_brightness(void) const { return _brightness; }

/* ============================================================
 *  HSV支持
 * ============================================================ */
ws2812_color_t ws2812_t::hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v)
{
    ws2812_color_t rgb = WS2812_COLOR_BLACK;
    if (s == 0) { rgb.r = rgb.g = rgb.b = v; return rgb; }

    float h1 = (float)(h % 360) / 60.0f;
    float s1 = (float)s / 255.0f;
    float v1 = (float)v / 255.0f;
    uint8_t i = (uint8_t)h1;
    float f = h1 - (float)i;
    float p = v1 * (1.0f - s1);
    float q = v1 * (1.0f - s1 * f);
    float t = v1 * (1.0f - s1 * (1.0f - f));

    float r, g, b;
    switch (i) {
        case 0: r = v1; g = t;   b = p;   break;
        case 1: r = q;   g = v1; b = p;   break;
        case 2: r = p;   g = v1; b = t;   break;
        case 3: r = p;   g = q;   b = v1; break;
        case 4: r = t;   g = p;   b = v1; break;
        default: r = v1; g = p;   b = q;   break;
    }
    rgb.r = (uint8_t)(r * 255.0f);
    rgb.g = (uint8_t)(g * 255.0f);
    rgb.b = (uint8_t)(b * 255.0f);
    return rgb;
}

status_t ws2812_t::set_led_hsv(uint16_t index, uint16_t h, uint8_t s, uint8_t v)
{
    return set_led(index, hsv_to_rgb(h, s, v));
}

status_t ws2812_t::set_all_hsv(uint16_t h, uint8_t s, uint8_t v)
{
    ws2812_color_t rgb = hsv_to_rgb(h, s, v);
    return set_all(rgb.r, rgb.g, rgb.b);
}

/* ============================================================
 *  动态效果
 * ============================================================ */
status_t ws2812_t::gradient(uint16_t start, uint16_t end,
                             const ws2812_color_t& color1, const ws2812_color_t& color2)
{
    if (!_initialized) return PYRO_ERROR;
    if (start >= _led_num || end >= _led_num || start > end) return PYRO_PARAM_ERROR;
    uint16_t range = end - start;
    if (range == 0) return set_led(start, color1);

    for (uint16_t i = start; i <= end; i++) {
        float ratio = (float)(i - start) / (float)range;
        ws2812_color_t c;
        c.r = (uint8_t)((float)color1.r + ((float)color2.r - (float)color1.r) * ratio);
        c.g = (uint8_t)((float)color1.g + ((float)color2.g - (float)color1.g) * ratio);
        c.b = (uint8_t)((float)color1.b + ((float)color2.b - (float)color1.b) * ratio);
        set_led(i, c);
    }
    return update();
}

status_t ws2812_t::rainbow(uint16_t start_hue, uint8_t density)
{
    if (!_initialized) return PYRO_ERROR;
    if (density == 0) return PYRO_PARAM_ERROR;
    for (uint16_t i = 0; i < _led_num; i++) {
        uint16_t hue = (start_hue + (uint32_t)i * density) % 360;
        set_led(i, hsv_to_rgb(hue, 255, 255));
    }
    return update();
}

status_t ws2812_t::breathing(const ws2812_color_t& color, uint8_t step)
{
    if (!_initialized) return PYRO_ERROR;
    if (step == 0) return PYRO_PARAM_ERROR;

    if (_breath_direction) {
        if (_breath_brightness + step >= 255) {
            _breath_brightness = 255;
            _breath_direction = false;
        } else {
            _breath_brightness += step;
        }
    } else {
        if (_breath_brightness <= step) {
            _breath_brightness = 0;
            _breath_direction = true;
        } else {
            _breath_brightness -= step;
        }
    }

    ws2812_color_t scaled;
    scaled.r = (uint8_t)((uint16_t)color.r * _breath_brightness / 255);
    scaled.g = (uint8_t)((uint16_t)color.g * _breath_brightness / 255);
    scaled.b = (uint8_t)((uint16_t)color.b * _breath_brightness / 255);
    set_all(scaled.r, scaled.g, scaled.b);
    return update();
}

status_t ws2812_t::chase(const ws2812_color_t& color, uint16_t position, uint8_t length)
{
    if (!_initialized) return PYRO_ERROR;
    if (length == 0 || length > _led_num) return PYRO_PARAM_ERROR;
    clear();
    for (uint8_t i = 0; i < length; i++) {
        uint16_t idx = (position + i) % _led_num;
        set_led(idx, color);
    }
    return update();
}

status_t ws2812_t::twinkle(const ws2812_color_t& color, uint8_t probability)
{
    if (!_initialized) return PYRO_ERROR;
    if (probability > 100) probability = 100;
    clear();
    for (uint16_t i = 0; i < _led_num; i++) {
        if ((rand() % 100) < probability) {
            set_led(i, color);
        }
    }
    return update();
}

status_t ws2812_t::color_wipe(const ws2812_color_t& color, uint16_t position)
{
    if (!_initialized) return PYRO_ERROR;
    if (position >= _led_num) position = _led_num - 1;
    clear();
    for (uint16_t i = 0; i <= position; i++) {
        set_led(i, color);
    }
    return update();
}

/* ============================================================
 *  测试
 * ============================================================ */
void ws2812_t::test_pattern(void)
{
    if (!_initialized) return;
    set_all(255, 0, 0);   update(); ws2812_hw_delay_ms(500);
    set_all(0, 255, 0);   update(); ws2812_hw_delay_ms(500);
    set_all(0, 0, 255);   update(); ws2812_hw_delay_ms(500);
    set_all(255, 255, 255); update(); ws2812_hw_delay_ms(500);
    clear(); update();
}

void ws2812_t::self_test(void)
{
    if (!_initialized) return;
    for (uint16_t i = 0; i < _led_num; i++) {
        clear();
        set_led(i, WS2812_COLOR_WHITE);
        update();
        ws2812_hw_delay_ms(100);
    }
    clear(); update();
}

/* ============================================================
 *  内部方法
 * ============================================================ */
uint32_t ws2812_t::_calc_tim_clock(void)
{
    uint32_t tim_clk, psc, arr;
    ws2812_hw_get_tim_config(&_hw, &tim_clk, &psc, &arr);
    _tim_clk = tim_clk;
    _psc = psc;
    _arr = arr;
    return tim_clk;
}

void ws2812_t::_calc_pwm_params(void)
{
    _pwm_freq = _tim_clk / ((_psc + 1) * (_arr + 1));
    if (_pwm_freq > 0) {
        _pwm_period_us = 1000000.0f / (float)_pwm_freq;
    } else {
        _pwm_period_us = 0.0f;
    }

    float ccr0 = (WS2812_CODE0_HIGH_US / _pwm_period_us) * (float)(_arr + 1);
    _code0_ccr = (uint8_t)(ccr0 + 0.5f);
    float ccr1 = (WS2812_CODE1_HIGH_US / _pwm_period_us) * (float)(_arr + 1);
    _code1_ccr = (uint8_t)(ccr1 + 0.5f);

    if (_code0_ccr > _arr) _code0_ccr = (uint8_t)_arr;
    if (_code1_ccr > _arr) _code1_ccr = (uint8_t)_arr;
}

void ws2812_t::_convert_color_to_pulse(void)
{
    uint32_t pulse_idx = 0;
    for (uint16_t led = 0; led < _led_num; led++) {
        uint32_t color_offset = (uint32_t)led * WS2812_BYTES_PER_LED;
        for (uint8_t byte = 0; byte < WS2812_BYTES_PER_LED; byte++) {
            uint16_t scaled = (uint16_t)_color_buf[color_offset + byte] * _brightness / 255;
            uint8_t val = (uint8_t)scaled;
            for (int8_t bit = 7; bit >= 0; bit--) {
                if (val & (1 << bit)) {
                    _pulse_buf[pulse_idx++] = _code1_ccr;
                } else {
                    _pulse_buf[pulse_idx++] = _code0_ccr;
                }
            }
        }
    }
    /* 复位码: 末尾填0 (低电平) */
    for (uint32_t i = 0; i < WS2812_RESET_PULSE_COUNT; i++) {
        _pulse_buf[pulse_idx++] = 0;
    }
}

bool ws2812_t::_allocate_buffers(uint16_t led_num)
{
    uint32_t color_size = WS2812_COLOR_BUF_SIZE(led_num);
    uint32_t pulse_count = WS2812_PULSE_BUF_SIZE(led_num) + WS2812_RESET_PULSE_COUNT;
    uint32_t pulse_size = pulse_count * sizeof(uint16_t);

    _color_buf = (uint8_t*)pvPortDmaMalloc(color_size);
    if (_color_buf == nullptr) return false;

    _pulse_buf = (uint16_t*)pvPortDmaMalloc(pulse_size);
    if (_pulse_buf == nullptr) {
        vPortDmaFree(_color_buf);
        _color_buf = nullptr;
        return false;
    }
    return true;
}

void ws2812_t::_free_buffers(void)
{
    if (_color_buf != nullptr) {
        vPortDmaFree(_color_buf);
        _color_buf = nullptr;
    }
    if (_pulse_buf != nullptr) {
        vPortDmaFree(_pulse_buf);
        _pulse_buf = nullptr;
    }
}

} // namespace pyro

/* ============================================================
 *  HAL DMA完成回调 (多路分发)
 *  同一TIM的所有通道共用同一个PWM Pulse Finished中断,
 *  通过遍历实例注册表, 找到匹配定时器且正在传输的实例。
 *  放在ITCM中加速中断响应。
 * ============================================================ */
extern "C" {

__attribute__((section(".itcm_text")))
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    pyro::ws2812_t* inst = pyro::ws2812_t::find_busy_instance(htim);
    if (inst != nullptr) {
        inst->_on_dma_complete();
    }
}

} // extern "C"
