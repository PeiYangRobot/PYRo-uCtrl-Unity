#ifndef WS2812_MIDDLEWARE_H
#define WS2812_MIDDLEWARE_H

#include "main.h"
#include <stdint.h>
#include "tim.h"
/* ============================================================
 *  WS2812 硬件配置结构体
 *  每一路PWM输出对应一个实例，包含定时器句柄、通道、DMA ID
 * ============================================================ */
typedef struct {
    TIM_HandleTypeDef* htim;      /* 定时器句柄, 如 &htim1 */
    uint32_t          channel;     /* PWM通道, TIM_CHANNEL_1/2/3/4 */
    uint32_t          dma_id;      /* DMA标识, TIM_DMA_ID_CC1/CC2/CC3/CC4 */
} ws2812_hw_t;

/* ============================================================
 *  便捷宏: 预定义常用硬件配置 (STM32H723)
 *  CubeMX已配置的PWM通道可直接使用
 *  注意: DMA数据宽度必须配置为 Half Word (16位)
 * ============================================================ */

#define WS2812_HW_TIM1_CH1   ((ws2812_hw_t){ &htim1, TIM_CHANNEL_1, TIM_DMA_ID_CC1 })
#define WS2812_HW_TIM1_CH2   ((ws2812_hw_t){ &htim1, TIM_CHANNEL_2, TIM_DMA_ID_CC2 })
#define WS2812_HW_TIM1_CH3   ((ws2812_hw_t){ &htim1, TIM_CHANNEL_3, TIM_DMA_ID_CC3 })
#define WS2812_HW_TIM1_CH4   ((ws2812_hw_t){ &htim1, TIM_CHANNEL_4, TIM_DMA_ID_CC4 })
#define WS2812_HW_TIM2_CH1   ((ws2812_hw_t){ &htim2, TIM_CHANNEL_1, TIM_DMA_ID_CC1 })
#define WS2812_HW_TIM2_CH2   ((ws2812_hw_t){ &htim2, TIM_CHANNEL_2, TIM_DMA_ID_CC2 })
#define WS2812_HW_TIM2_CH3   ((ws2812_hw_t){ &htim2, TIM_CHANNEL_3, TIM_DMA_ID_CC3 })
#define WS2812_HW_TIM2_CH4   ((ws2812_hw_t){ &htim2, TIM_CHANNEL_4, TIM_DMA_ID_CC4 })
#define WS2812_HW_TIM3_CH1   ((ws2812_hw_t){ &htim3, TIM_CHANNEL_1, TIM_DMA_ID_CC1 })
#define WS2812_HW_TIM3_CH2   ((ws2812_hw_t){ &htim3, TIM_CHANNEL_2, TIM_DMA_ID_CC2 })
#define WS2812_HW_TIM3_CH3   ((ws2812_hw_t){ &htim3, TIM_CHANNEL_3, TIM_DMA_ID_CC3 })
#define WS2812_HW_TIM3_CH4   ((ws2812_hw_t){ &htim3, TIM_CHANNEL_4, TIM_DMA_ID_CC4 })

/* 初始化 (硬件由CubeMX配置, 此处留空) */
extern void ws2812_hw_init(ws2812_hw_t* hw);
/* 反初始化 */
extern void ws2812_hw_deinit(ws2812_hw_t* hw);
/* 启动DMA传输 */
extern void ws2812_hw_start_dma(ws2812_hw_t* hw, const uint8_t* pulse_buf, uint16_t length);
/* 停止DMA传输 */
extern void ws2812_hw_stop_dma(ws2812_hw_t* hw);
/* 查询DMA是否正在传输 */
extern uint8_t ws2812_hw_is_dma_busy(ws2812_hw_t* hw);
/* 获取定时器配置 (时钟、预分频、自动重装载) */
extern void ws2812_hw_get_tim_config(ws2812_hw_t* hw, uint32_t* tim_clk, uint32_t* psc, uint32_t* arr);
/* 微秒延时 */
extern void ws2812_hw_delay_us(uint16_t us);
/* 毫秒延时 */
extern void ws2812_hw_delay_ms(uint16_t ms);

#endif // WS2812_MIDDLEWARE_H
