/*
 * 音符频率定义（钢琴 88 键，A0 ~ C8，A4=440Hz）
 * 该文件只保存频率值，时钟频率由驱动层决定。
 * 使用时通过 BUZZER_CALC_ARR(note) 计算当前时钟下的 ARR 值。
 *
 * 注意：BUZZER_CNT_CLOCK 需在使用前定义，或由驱动层提供。
 */

#ifndef PYRO_BUZZER_NOTES_H
#define PYRO_BUZZER_NOTES_H

#include <stdint.h>

// 音符枚举 (A0 ~ C8)
typedef enum {
    BUZZER_NOTE_A0 = 0,
    BUZZER_NOTE_AS0,
    BUZZER_NOTE_B0,
    BUZZER_NOTE_C1,
    BUZZER_NOTE_CS1,
    BUZZER_NOTE_D1,
    BUZZER_NOTE_DS1,
    BUZZER_NOTE_E1,
    BUZZER_NOTE_F1,
    BUZZER_NOTE_FS1,
    BUZZER_NOTE_G1,
    BUZZER_NOTE_GS1,
    BUZZER_NOTE_A1,
    BUZZER_NOTE_AS1,
    BUZZER_NOTE_B1,
    BUZZER_NOTE_C2,
    BUZZER_NOTE_CS2,
    BUZZER_NOTE_D2,
    BUZZER_NOTE_DS2,
    BUZZER_NOTE_E2,
    BUZZER_NOTE_F2,
    BUZZER_NOTE_FS2,
    BUZZER_NOTE_G2,
    BUZZER_NOTE_GS2,
    BUZZER_NOTE_A2,
    BUZZER_NOTE_AS2,
    BUZZER_NOTE_B2,
    BUZZER_NOTE_C3,
    BUZZER_NOTE_CS3,
    BUZZER_NOTE_D3,
    BUZZER_NOTE_DS3,
    BUZZER_NOTE_E3,
    BUZZER_NOTE_F3,
    BUZZER_NOTE_FS3,
    BUZZER_NOTE_G3,
    BUZZER_NOTE_GS3,
    BUZZER_NOTE_A3,
    BUZZER_NOTE_AS3,
    BUZZER_NOTE_B3,
    BUZZER_NOTE_C4,
    BUZZER_NOTE_CS4,
    BUZZER_NOTE_D4,
    BUZZER_NOTE_DS4,
    BUZZER_NOTE_E4,
    BUZZER_NOTE_F4,
    BUZZER_NOTE_FS4,
    BUZZER_NOTE_G4,
    BUZZER_NOTE_GS4,
    BUZZER_NOTE_A4,
    BUZZER_NOTE_AS4,
    BUZZER_NOTE_B4,
    BUZZER_NOTE_C5,
    BUZZER_NOTE_CS5,
    BUZZER_NOTE_D5,
    BUZZER_NOTE_DS5,
    BUZZER_NOTE_E5,
    BUZZER_NOTE_F5,
    BUZZER_NOTE_FS5,
    BUZZER_NOTE_G5,
    BUZZER_NOTE_GS5,
    BUZZER_NOTE_A5,
    BUZZER_NOTE_AS5,
    BUZZER_NOTE_B5,
    BUZZER_NOTE_C6,
    BUZZER_NOTE_CS6,
    BUZZER_NOTE_D6,
    BUZZER_NOTE_DS6,
    BUZZER_NOTE_E6,
    BUZZER_NOTE_F6,
    BUZZER_NOTE_FS6,
    BUZZER_NOTE_G6,
    BUZZER_NOTE_GS6,
    BUZZER_NOTE_A6,
    BUZZER_NOTE_AS6,
    BUZZER_NOTE_B6,
    BUZZER_NOTE_C7,
    BUZZER_NOTE_CS7,
    BUZZER_NOTE_D7,
    BUZZER_NOTE_DS7,
    BUZZER_NOTE_E7,
    BUZZER_NOTE_F7,
    BUZZER_NOTE_FS7,
    BUZZER_NOTE_G7,
    BUZZER_NOTE_GS7,
    BUZZER_NOTE_A7,
    BUZZER_NOTE_AS7,
    BUZZER_NOTE_B7,
    BUZZER_NOTE_C8,
    BUZZER_NOTE_MAX
} buzzer_note_t;

// 音符频率表 (单位: Hz，A4=440)
static const float buzzer_note_freq[] = {
    27.50f,   // A0
    29.14f,   // AS0
    30.87f,   // B0
    32.70f,   // C1
    34.65f,   // CS1
    36.71f,   // D1
    38.89f,   // DS1
    41.20f,   // E1
    43.65f,   // F1
    46.25f,   // FS1
    49.00f,   // G1
    51.91f,   // GS1
    55.00f,   // A1
    58.27f,   // AS1
    61.74f,   // B1
    65.41f,   // C2
    69.30f,   // CS2
    73.42f,   // D2
    77.78f,   // DS2
    82.41f,   // E2
    87.31f,   // F2
    92.50f,   // FS2
    98.00f,   // G2
    103.83f,  // GS2
    110.00f,  // A2
    116.54f,  // AS2
    123.47f,  // B2
    130.81f,  // C3
    138.59f,  // CS3
    146.83f,  // D3
    155.56f,  // DS3
    164.81f,  // E3
    174.61f,  // F3
    185.00f,  // FS3
    196.00f,  // G3
    207.65f,  // GS3
    220.00f,  // A3
    233.08f,  // AS3
    246.94f,  // B3
    261.63f,  // C4
    277.18f,  // CS4
    293.66f,  // D4
    311.13f,  // DS4
    329.63f,  // E4
    349.23f,  // F4
    369.99f,  // FS4
    392.00f,  // G4
    415.30f,  // GS4
    440.00f,  // A4
    466.16f,  // AS4
    493.88f,  // B4
    523.25f,  // C5
    554.37f,  // CS5
    587.33f,  // D5
    622.25f,  // DS5
    659.26f,  // E5
    698.46f,  // F5
    739.99f,  // FS5
    783.99f,  // G5
    830.61f,  // GS5
    880.00f,  // A5
    932.33f,  // AS5
    987.77f,  // B5
    1046.50f, // C6
    1108.73f, // CS6
    1174.66f, // D6
    1244.51f, // DS6
    1318.51f, // E6
    1396.91f, // F6
    1479.98f, // FS6
    1567.98f, // G6
    1661.22f, // GS6
    1760.00f, // A6
    1864.66f, // AS6
    1975.53f, // B6
    2093.00f, // C7
    2217.46f, // CS7
    2349.32f, // D7
    2489.02f, // DS7
    2637.02f, // E7
    2793.83f, // F7
    2959.96f, // FS7
    3135.96f, // G7
    3322.44f, // GS7
    3520.00f, // A7
    3729.31f, // AS7
    3951.07f, // B7
    4186.01f  // C8
};

// 宏：获取音符频率
#define BUZZER_GET_FREQ(note) (buzzer_note_freq[note])

// 内联函数：根据音符和当前时钟计算 ARR 值
// 使用时需保证 BUZZER_CNT_CLOCK 已在调用前定义（由驱动层提供）
static inline uint32_t buzzer_calc_arr(buzzer_note_t note, uint32_t cnt_clock)
{
    float freq = buzzer_note_freq[note];
    uint32_t arr = (uint32_t)((cnt_clock / freq) - 1.0f);
    if (arr < 1) arr = 1;
    if (arr > 65535) arr = 65535;
    return arr;
}

// 便捷宏：使用默认时钟（需在驱动层定义 BUZZER_CNT_CLOCK）
#ifdef BUZZER_CNT_CLOCK
    #define BUZZER_GET_ARR(note) buzzer_calc_arr(note, BUZZER_CNT_CLOCK)
#else
    // 如果未定义默认时钟，则要求显式传入
    #define BUZZER_GET_ARR(note) buzzer_calc_arr(note, BUZZER_CNT_CLOCK)
#endif

#endif // PYRO_BUZZER_NOTES_H