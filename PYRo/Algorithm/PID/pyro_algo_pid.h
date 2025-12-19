/**
 * @file pyro_algo_pid.h
 * @brief Header file for the PYRO C++ PID Controller class.
 *
 * This file defines the `pyro::pid_t` class, which encapsulates a
 * PID controller with various improvement options (e.g., filters, OLS).
 * It is a C++ refactor of the original C controller library.
 *
 * @author Wang Hongxi (Original C), Lucky (C++ Refactor)
 * @version 1.1.4
 * @date 2025-11-16
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_ALGO_PID_H__
#define __PYRO_ALGO_PID_H__

#include "pyro_algo_ols.h" // For pyro::ols_t
#include <cstdint>

namespace pyro
{

/**
 * @brief PID Controller C++ Class.
 *
 * Encapsulates PID logic, state, and optional improvements.
 * Automatically uses `dwt_drv_t` for time delta and `ols_t` for
 * derivative calculation if specified.
 */
class pid_t
{
  public:
    /**
     * @brief PID Improvement bitmask options.
     */
    enum improvement_t : uint8_t
    {
        NONE                      = 0x00, ///< No improvements
        INTEGRAL_LIMIT            = 0x01, ///< Enable integral limit
        DERIVATIVE_ON_MEASUREMENT = 0x02, ///< Enable Derivative on Measurement
        TRAPEZOID_INTEGRAL        = 0x04, ///< Enable Trapezoid Integral
        PROPORTIONAL_ON_MEASUREMENT = 0x08, ///< (Unused in current logic)
        OUTPUT_FILTER               = 0x10, ///< Enable Output LPF
        CHANGING_INTEGRATION_RATE = 0x20, ///< Enable Changing Integration Rate
        DERIVATIVE_FILTER         = 0x40, ///< Enable Derivative LPF
        ERROR_HANDLE              = 0x80, ///< Enable Error Handling
    };

    /**
     * @brief Error types for the PID handler.
     */
    enum class error_type_t : uint8_t
    {
        NONE          = 0x00U,
        MOTOR_BLOCKED = 0x01U
    };

    /**
     * @brief Structure for the PID error handler state.
     */
    struct error_handler_t
    {
        uint64_t error_count;
        error_type_t error_type;
    };

    /**
     * @brief Function pointer type for user callbacks.
     * @param pid A pointer to the current pid_t instance.
     */
    using user_func_t = void (*)(pid_t *pid);

    // --- Constructors (Overloaded) ---

    /**
     * @brief Constructor 1: Basic PID.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param integral_limit Max absolute value of the integral term.
     * @param max_out Max absolute value of the final output.
     * @param improve Bitmask of improvement_t flags.
     */
    pid_t(float kp, float ki, float kd, float integral_limit, float max_out,
          uint8_t improve = INTEGRAL_LIMIT);

    /**
     * @brief Constructor 2: PID with filters and OLS.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param integral_limit Max absolute value of the integral term.
     * @param max_out Max absolute value of the final output.
     * @param output_cutoff_hz Cutoff frequency (Hz) for the output LPF.
     * (Set to 0 to disable).
     * @param derivative_cutoff_hz Cutoff frequency (Hz) for the derivative LPF.
     * (Set to 0 to disable).
     * @param ols_order Order (sample count) for the OLS derivative.
     * @param improve Bitmask of improvement_t flags.
     */
    pid_t(float kp, float ki, float kd, float integral_limit, float max_out,
          float output_cutoff_hz, float derivative_cutoff_hz,
          uint16_t ols_order,
          uint8_t improve = INTEGRAL_LIMIT | OUTPUT_FILTER | DERIVATIVE_FILTER);

    /**
     * @brief Constructor 3: Full-featured (Main Constructor).
     * @param max_out Max absolute value of the final output.
     * @param integral_limit Max absolute value of the integral term.
     * @param deadband Error deadband; PID calculation is skipped if
     * |error| < deadband.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param A CoefA for ChangingIntegrationRate.
     * @param B CoefB for ChangingIntegrationRate.
     * @param output_cutoff_hz Cutoff frequency (Hz) for the output LPF.
     * (Set to 0 to disable).
     * @param derivative_cutoff_hz Cutoff frequency (Hz) for the derivative LPF.
     * (Set to 0 to disable).
     * @param ols_order Order (sample count) for the OLS derivative.
     * @param improve Bitmask of improvement_t flags.
     */
    pid_t(float max_out, float integral_limit, float deadband, float kp,
          float ki, float kd,
          float A, // CoefA for ChangingIntegrationRate
          float B, // CoefB for ChangingIntegrationRate
          float output_cutoff_hz, float derivative_cutoff_hz,
          uint16_t ols_order, uint8_t improve);

    /**
     * @brief Calculates the PID output.
     * @param ref The desired reference (setpoint) value.
     * @param measure The current measured value.
     * @return The calculated PID output.
     */
    float calculate(float ref, float measure);

    /**
     * @brief Clears the internal PID state (I-term, D-term, error, etc.).
     */
    void clear();

    /**
     * @brief Sets new PID gains (Kp, Ki, Kd).
     */
    void set_gains(float kp, float ki, float kd);

    /**
     * @brief Registers User Function 1.
     * (Called after error calculation, before P/I/D term calculation).
     */
    void set_user_func1(user_func_t func);

    /**
     * @brief Registers User Function 2.
     * (Called after P/I/D term calculation, before improvements).
     */
    void set_user_func2(user_func_t func);

    // --- Getters ---
    [[nodiscard]] float get_output() const
    {
        return _output;
    }
    [[nodiscard]] float get_p_out() const
    {
        return _p_out;
    }
    [[nodiscard]] float get_i_out() const
    {
        return _i_out;
    }
    [[nodiscard]] float get_d_out() const
    {
        return _d_out;
    }
    [[nodiscard]] float get_error() const
    {
        return _err;
    }

  private:
    // --- Private Helper Functions (PID Improvements) ---
    void trapezoid_integral();
    void limit_integral();
    void changing_integration_rate();
    void filter_output();
    void filter_derivative();
    void limit_output();
    void limit_proportion();
    void handle_error();

    // --- Private Member Variables ---

    // Configuration
    float _kp, _ki, _kd;               ///< PID Gains
    float _max_out, _integral_limit;   ///< Output and Integral limits
    float _deadband;                   ///< Error deadband
    float _coef_a, _coef_b;            ///< ChangingIntegrationRate params
    float _output_lpf_rc;              ///< Output LPF RC time constant
    float _derivative_lpf_rc;          ///< Derivative LPF RC time constant
    uint16_t _ols_order;               ///< OLS filter order
    uint8_t _improve;                  ///< Improvement flags bitmask
    error_handler_t _error_handler{};  ///< Error handler state
    user_func_t _user_func1 = nullptr; ///< Callback 1 (pre-calc)
    user_func_t _user_func2 = nullptr; ///< Callback 2 (post-calc)

    // State
    float _ref              = 0.0f; ///< Reference (setpoint)
    float _measure          = 0.0f; ///< Measured value
    float _err              = 0.0f; ///< Error (ref - measure)
    float _last_err         = 0.0f; ///< Error from the previous cycle
    float _p_out            = 0.0f; ///< Proportional term output
    float _i_out            = 0.0f; ///< Integral term output (accumulated)
    float _d_out            = 0.0f; ///< Derivative term output
    float _i_term       = 0.0f; ///< Current frame's integral term (pre-limit)
    float _last_i_term  = 0.0f; ///< Last frame's integral term
    float _output       = 0.0f; ///< Final PID output
    float _last_output  = 0.0f; ///< Final output from the previous cycle
    float _last_d_out   = 0.0f; ///< D-term output from the previous cycle
    float _last_measure = 0.0f; ///< Measured value from the previous cycle

    // Dependencies
    uint32_t _dwt_cnt   = 0;    ///< Counter for DWT delta-time calculation
    float _dt           = 0.0f; ///< Delta-time from DWT
    ols_t _ols;                 ///< OLS instance (constructed with _ols_order)
};

} // namespace pyro

#endif // __PYRO_ALGO_PID_H__