/**
 * @file pyro_dwt_drv.h
 * @brief Header file for the PYRO C++ DWT (Data Watchpoint and Trace) Driver.
 *
 * This file defines the `pyro::dwt_drv_t` static class, which provides
 * a high-resolution timer interface using the ARM Cortex-M CYCCNT register.
 *
 * @author Wang Hongxi (Original C)
 * @author Lucky (C++ Refactor)
 * @version 1.1.0
 * @date 2025-11-13
 */

#ifndef __PYRO_DWT_DRV_H__
#define __PYRO_DWT_DRV_H__

#include "stdint.h"

namespace pyro
{

/**
 * @brief C++ DWT (Data Watchpoint and Trace) high-resolution timer driver.
 *
 * This is a static class providing a singleton interface to the
 * ARM Cortex-M DWT CYCCNT register for high-precision timing.
 */
class dwt_drv_t
{
  public:
    // --- Constructor/Destructor ---
    // Deleted constructors to make this a pure static class.
    dwt_drv_t()                              = delete;
    dwt_drv_t(const dwt_drv_t &)            = delete;
    dwt_drv_t &operator=(const dwt_drv_t &) = delete;

    /**
     * @brief Replaces the C-style DWT_Time_t struct.
     */
    struct time_t
    {
        uint32_t s;  // Seconds
        uint16_t ms; // Milliseconds
        uint16_t us; // Microseconds
    };

    /**
     * @brief Initializes the DWT peripheral. Must be called once at startup.
     * @param cpu_freq_mhz The system CPU frequency in MHz.
     */
    static void init(uint32_t cpu_freq_mhz);

    /**
     * @brief Gets the elapsed time delta (float, seconds) since 'cnt_last'.
     * @param cnt_last Pointer to the variable storing the last count.
     * This variable will be updated.
     * @return The time delta in seconds (float).
     */
    static float get_delta_t(uint32_t *cnt_last);

    /**
     * @brief Gets the elapsed time delta (double, seconds) since 'cnt_last'.
     * @param cnt_last Pointer to the variable storing the last count.
     * This variable will be updated.
     * @return The time delta in seconds (double).
     */
    static double get_delta_t_64(uint32_t *cnt_last);

    /**
     * @brief Gets the total time elapsed since init() (float, seconds).
     */
    static float get_timeline_s();

    /**
     * @brief Gets the total time elapsed since init() (float, milliseconds).
     */
    static float get_timeline_ms();

    /**
     * @brief Gets the total time elapsed since init() (uint64_t, microseconds).
     */
    static uint64_t get_timeline_us();

    /**
     * @brief Gets the total time elapsed since init() (time_t struct).
     */
    static time_t get_timeline();

    /**
     * @brief Blocking delay (float, seconds).
     */
    static void delay_s(float seconds);

    /**
     * @brief Blocking delay (uint32_t, microseconds).
     */
    static void delay_us(uint32_t microseconds);

    /**
     * @brief Gets the current raw 32-bit DWT->CYCCNT counter value.
     */
    static uint32_t get_current_ticks();

  private:
    /**
     * @brief Updates the 64-bit cycle counter to handle 32-bit overflow.
     */
    static void update_cycle_count();

    /**
     * @brief Updates the internal _sys_time structure.
     */
    static void update_sys_time();

    // --- Private Static Members (replaces C globals) ---
    inline static uint32_t _cpu_freq_hz{};
    inline static uint32_t _cpu_freq_hz_ms{};
    inline static uint32_t _cpu_freq_hz_us{};
    inline static uint32_t _cyccnt_round_count{}; // 32-bit counter overflow count
    inline static uint32_t _cyccnt_last{}; // Last count, for overflow detection
    inline static uint64_t _cyccnt_64{};   // 64-bit total cycle count
    inline static time_t _sys_time{};      // Formatted system time
};

} // namespace pyro

#endif // __PYRO_DWT_DRV_H__