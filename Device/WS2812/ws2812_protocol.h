#ifndef WS2812_PROTOCOL_H
#define WS2812_PROTOCOL_H

#include <stdint.h>

/*数据速率*/
#define WS2812_DATA_RATE_HZ        800000U     // 800kHz
#define WS2812_BIT_PERIOD_US       1.25f        // 1.25us/bit

/*0码: 高电平短, 低电平长*/
/* 典型值: 高0.4us + 低0.85us = 1.25us */
/* 容差范围: 高0.35~0.5us, 低0.65~0.8us */
#define WS2812_CODE0_HIGH_US       0.4f
#define WS2812_CODE0_LOW_US        0.85f

/*1码: 高电平长, 低电平短*/
/* 典型值: 高0.8us + 低0.45us = 1.25us */
/* 容差范围: 高0.65~0.8us, 低0.35~0.5us */
#define WS2812_CODE1_HIGH_US       0.8f
#define WS2812_CODE1_LOW_US        0.45f

/*复位码: 数据线拉低, 锁存显示*/
/* 最低要求 >= 50us, 留余量用60us */
#define WS2812_RESET_LOW_US_MIN    50U
#define WS2812_RESET_DELAY_US      60U


#define WS2812_BITS_PER_LED        24U     // 每颗LED 24bit
#define WS2812_BYTES_PER_LED       3U      // 每颗LED 3字节 (G,R,B)

/*颜色顺序索引 (用于颜色缓冲区访问)*/
#define WS2812_OFFSET_G             0U      // 绿色在第0字节
#define WS2812_OFFSET_R             1U      // 红色在第1字节
#define WS2812_OFFSET_B             2U      // 蓝色在第2字节


#define WS2812_COLOR_BUF_SIZE(led_num)  ((led_num) * WS2812_BYTES_PER_LED)
#define WS2812_PULSE_BUF_SIZE(led_num)  ((led_num) * WS2812_BITS_PER_LED)


#define WS2812_RESET_PULSE_COUNT        40U


/* RGB 颜色 */
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_color_t;


typedef struct
{
    uint16_t h;  
    uint8_t  s;   
    uint8_t  v;   
} ws2812_color_hsv_t;
//常用颜色 
constexpr ws2812_color_t WS2812_COLOR_BLACK = {0,   0,   0  };
constexpr ws2812_color_t WS2812_COLOR_RED   = {255, 0,   0  };
constexpr ws2812_color_t WS2812_COLOR_GREEN = {0,   255, 0  };
constexpr ws2812_color_t WS2812_COLOR_BLUE  = {0,   0,   255};
constexpr ws2812_color_t WS2812_COLOR_WHITE = {255, 255, 255};


#endif // WS2812_PROTOCOL_H
