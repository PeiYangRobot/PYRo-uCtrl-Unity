/**
 * @file pyro_dwt_drv.cpp
 * @brief Implementation file for the PYRO C++ DWT Driver class.
 *
 * This file implements the `pyro::dwt_drv_t` static methods.
 *
 * @author Wang Hongxi (Original C)
 * @author Lucky (C++ Refactor)
 * @version 1.1.0
 * @date 2025-11-13
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_dwt_drv.h"
#include "main.h" // For CoreDebug, DWT registers

namespace pyro
{
/**
 * @brief Initializes the DWT peripheral.
 */
void dwt_drv_t::init(const uint32_t cpu_freq_mhz)
{
    // Enable DWT peripheral
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Reset DWT CYCCNT register
    DWT->CYCCNT = 0u;

    // Enable Cortex-M DWT CYCCNT register
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Store frequency in different units
    _cpu_freq_hz    = cpu_freq_mhz * 1000000;
    _cpu_freq_hz_ms = _cpu_freq_hz / 1000;
    _cpu_freq_hz_us = _cpu_freq_hz / 1000000;

    // Reset all counter states
    _cyccnt_round_count = 0;
    _cyccnt_last        = 0;
    _cyccnt_64          = 0;
}

/**
 * @brief Gets the time delta (float, seconds).
 */
float dwt_drv_t::get_delta_t(uint32_t *cnt_last)
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    // Calculate delta, (uint32_t) cast handles 32-bit wrap-around
    const float dt =
        static_cast<float>(static_cast<uint32_t>(cnt_now - *cnt_last)) /
        static_cast<float>(_cpu_freq_hz);
    *cnt_last = cnt_now;

    return dt;
}

/**
 * @brief Gets the time delta (double, seconds).
 */
double dwt_drv_t::get_delta_t_64(uint32_t *cnt_last)
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    const double dt =
        static_cast<double>(static_cast<uint32_t>(cnt_now - *cnt_last)) /
        static_cast<double>(_cpu_freq_hz);
    *cnt_last = cnt_now;

    return dt;
}

/**
 * @brief Updates the internal _sys_time structure.
 */
void dwt_drv_t::update_sys_time()
{
    // Must update overflow counter first
    update_cycle_count();

    const volatile uint32_t cnt_now = DWT->CYCCNT;

    // Calculate 64-bit total cycle count
    _cyccnt_64 = static_cast<uint64_t>(_cyccnt_round_count) * 0x100000000ULL +
                 static_cast<uint64_t>(cnt_now);

    // Decompose into s, ms, us
    const uint64_t cnt_temp_1 = _cyccnt_64 / _cpu_freq_hz;
    const uint64_t cnt_temp_2 = _cyccnt_64 - cnt_temp_1 * _cpu_freq_hz;
    _sys_time.s               = static_cast<uint32_t>(cnt_temp_1);
    _sys_time.ms = static_cast<uint16_t>(cnt_temp_2 / _cpu_freq_hz_ms);
    const uint64_t cnt_temp_3 =
        cnt_temp_2 - static_cast<uint64_t>(_sys_time.ms) * _cpu_freq_hz_ms;
    _sys_time.us = static_cast<uint16_t>(cnt_temp_3 / _cpu_freq_hz_us);
}

/**
 * @brief Gets the total time elapsed (float, seconds).
 */
float dwt_drv_t::get_timeline_s()
{
    update_sys_time();
    return static_cast<float>(_sys_time.s) +
           static_cast<float>(_sys_time.ms) * 0.001f +
           static_cast<float>(_sys_time.us) * 0.000001f;
}

/**
 * @brief Gets the total time elapsed (float, milliseconds).
 */
float dwt_drv_t::get_timeline_ms()
{
    update_sys_time();
    return static_cast<float>(_sys_time.s) * 1000.0f +
           static_cast<float>(_sys_time.ms) +
           static_cast<float>(_sys_time.us) * 0.001f;
}

/**
 * @brief Gets the total time elapsed (uint64_t, microseconds).
 */
uint64_t dwt_drv_t::get_timeline_us()
{
    update_sys_time();
    return static_cast<uint64_t>(_sys_time.s) * 1000000 +
           static_cast<uint64_t>(_sys_time.ms) * 1000 +
           static_cast<uint64_t>(_sys_time.us);
}

/**
 * @brief Gets the total formatted time (time_t struct).
 */
dwt_drv_t::time_t dwt_drv_t::get_timeline()
{
    update_sys_time();
    return _sys_time;
}

/**
 * @brief Updates the 32-bit counter overflow.
 */
void dwt_drv_t::update_cycle_count()
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < _cyccnt_last)
    {
        _cyccnt_round_count++;
    }
    _cyccnt_last = cnt_now;
}

/**
 * @brief Blocking delay (float, seconds).
 */
void dwt_drv_t::delay_s(const float seconds)
{
    const uint32_t start_tick = DWT->CYCCNT;
    const auto delay_ticks =
        static_cast<uint32_t>(seconds * static_cast<float>(_cpu_freq_hz));

    while ((DWT->CYCCNT - start_tick) < delay_ticks)
    {
        // Busy wait
    }
}

/**
 * @brief Blocking delay (uint32_t, microseconds).
 */
void dwt_drv_t::delay_us(const uint32_t microseconds)
{
    const uint32_t start_tick = DWT->CYCCNT;
    const auto delay_ticks    = static_cast<uint32_t>(
        static_cast<float>(microseconds) * static_cast<float>(_cpu_freq_hz_us));

    while ((DWT->CYCCNT - start_tick) < delay_ticks)
    {
        // Busy wait
    }
}

/**
 * @brief Gets the current raw 32-bit DWT->CYCCNT counter value.
 */
uint32_t dwt_drv_t::get_current_ticks()
{
    return DWT->CYCCNT;
}

} // namespace pyro