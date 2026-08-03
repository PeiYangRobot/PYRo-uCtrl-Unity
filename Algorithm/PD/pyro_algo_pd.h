/**
 * @file pyro_algo_pd.h
 * @brief Proportional-derivative controller with external velocity feedback.
 */

#ifndef __PYRO_ALGO_PD_H__
#define __PYRO_ALGO_PD_H__

#include <cstdint>

namespace pyro
{

/**
 * @brief PD controller with optional integral and PID-style filters.
 *
 * The derivative input is supplied by the caller. It is interpreted as the
 * measured-variable velocity, so the derivative term is calculated as
 * -Kd * measurement_speed. No finite-difference or OLS derivative estimation
 * is performed by this controller.
 */
class pd_ctrl_t
{
  public:
    enum improvement_t : uint8_t
    {
        NONE               = 0x00,
        INTEGRAL           = 0x01,
        INTEGRAL_LIMIT     = 0x02,
        TRAPEZOID_INTEGRAL = 0x04,
        OUTPUT_FILTER      = 0x10,
        DERIVATIVE_FILTER  = 0x40,
    };

    static constexpr uint8_t MAX_LPF_ORDER = 4;

    /**
     * @brief Basic PD constructor.
     *
     * No optional feature is enabled. Configure the deadband separately with
     * set_deadband() when needed.
     */
    pd_ctrl_t(float kp, float kd, float max_out);

    /** @brief Basic PD constructor with an error deadband. */
    pd_ctrl_t(float kp, float kd, float max_out, float deadband);

    /**
     * @brief Constructor with integral and integral-limit support only.
     *
     * Integral and integral-limit features are enabled by default. Pass an
     * explicit improve value to override that behavior.
     */
    pd_ctrl_t(float kp, float ki, float kd, float integral_limit,
              float max_out,
              uint8_t improve = INTEGRAL | INTEGRAL_LIMIT);

    /**
     * @brief Constructor with output and derivative filters only.
     *
     * Output and derivative filters are enabled by default. Pass an explicit
     * improve value to override that behavior.
     */
    pd_ctrl_t(float kp, float kd, float max_out, float output_cutoff_hz,
              uint8_t output_lpf_order, float derivative_cutoff_hz,
              uint8_t derivative_lpf_order,
              uint8_t improve = OUTPUT_FILTER | DERIVATIVE_FILTER);

    /**
     * @brief Full constructor with optional integral and filters.
     *
     * Integral, integral-limit, output-filter, and derivative-filter features
     * are enabled by default. Pass an explicit improve value to override that
     * behavior.
     */
    pd_ctrl_t(float kp, float ki, float kd, float integral_limit,
              float max_out, float output_cutoff_hz,
              uint8_t output_lpf_order, float derivative_cutoff_hz,
              uint8_t derivative_lpf_order,
              uint8_t improve = INTEGRAL | INTEGRAL_LIMIT |
                                OUTPUT_FILTER | DERIVATIVE_FILTER);

    /**
     * @brief Calculates the controller output.
     * @param reference Desired value.
     * @param measurement Current measured value.
     * @param measurement_speed Current measured-variable velocity.
     */
    float calculate(float reference, float measurement,
                    float measurement_speed);

    /** @brief Clears integral, filter, and output states. */
    void clear();

    /** @brief Updates Kp and Kd while preserving Ki. */
    void set_gains(float kp, float kd);

    /** @brief Updates Kp, Ki, and Kd. */
    void set_gains(float kp, float ki, float kd);

    void set_max_out(float max_out);
    void set_deadband(float deadband);
    void set_improve(uint8_t improve);

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
        return _error;
    }

    [[nodiscard]] float get_measurement_speed() const
    {
        return _measurement_speed;
    }

    [[nodiscard]] float get_kp() const
    {
        return _kp;
    }

    [[nodiscard]] float get_ki() const
    {
        return _ki;
    }

    [[nodiscard]] float get_kd() const
    {
        return _kd;
    }

  private:
    void calculate_dt();
    void integrate();
    void limit_integral();
    void filter_output();
    void filter_derivative();
    void limit_output();

    float _kp;
    float _ki;
    float _kd;
    float _max_out;
    float _integral_limit;
    float _deadband;
    float _output_lpf_rc;
    float _derivative_lpf_rc;

    uint8_t _output_lpf_order;
    uint8_t _derivative_lpf_order;
    uint8_t _improve;

    float _reference;
    float _measurement;
    float _error;
    float _last_error;
    float _measurement_speed;
    float _p_out;
    float _i_out;
    float _i_term;
    float _d_out;
    float _output;
    float _dt;

    uint32_t _dwt_cnt;
    float _output_lpf_state[MAX_LPF_ORDER];
    float _derivative_lpf_state[MAX_LPF_ORDER];
};

} // namespace pyro

#endif // __PYRO_ALGO_PD_H__
