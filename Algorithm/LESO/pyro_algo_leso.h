/**
 * @file pyro_algo_leso.h
 * @brief Header file for the PYRO C++ N-th order Linear Extended State Observer (LESO).
 * Defines the template class interface.
 */

#ifndef __PYRO_ALGO_LESO_H__
#define __PYRO_ALGO_LESO_H__

#include "pyro_dwt_drv.h" // Ensures dwt_drv_t is available
#include <cstdint>
#include <array>

namespace pyro
{

/**
 * @brief N-th Order Linear Extended State Observer (LESO) Template Class.
 *
 * @tparam Order The order of the observer. Must be >= 2.
 */
template <uint32_t Order>
class leso_t
{
    // Compile-time check to ensure order is at least 2
    static_assert(Order >= 2, "LESO order must be at least 2.");

  public:
    /**
     * @brief Constructor for N-th order LESO.
     * @param omega_o Observer bandwidth (rad/s).
     * @param b       Control gain of the plant (b0 in ADRC theory).
     * @param z_limit Maximum absolute value for the disturbance estimation.
     */
    leso_t(float omega_o, float b, float z_limit = 0.0f);

    /**
     * @brief Updates the LESO states using Forward Euler integration.
     */
    void update(float measure, float u);

    /**
     * @brief Clears all internal states.
     */
    void clear();

    /**
     * @brief Dynamically updates the observer parameters.
     */
    void set_params(float omega_o, float b);

    /**
     * @brief Sets a hard limit for the disturbance estimation.
     */
    void set_z_limit(float limit);

    // --- Getters ---
    [[nodiscard]] float get_z(uint32_t index) const;
    [[nodiscard]] float get_disturbance() const;
    [[nodiscard]] float get_b() const;
    [[nodiscard]] constexpr uint32_t get_order() const;

  private:
    // --- Private Helper Functions ---
    static constexpr uint32_t calculate_nCr(uint32_t n, uint32_t k);
    void calculate_betas();

    // --- Private Member Variables ---
    float _omega_o;  ///< Observer bandwidth (rad/s)
    float _b;        ///< Plant control gain parameter
    float _z_limit;  ///< Limit for total disturbance estimation

    // Arrays allocated on the stack
    std::array<float, Order> _beta;   ///< Observer gains
    std::array<float, Order> _z;      ///< Current states
    std::array<float, Order> _next_z; ///< Buffer for state updates

    uint32_t _dwt_cnt = 0;
    float _dt         = 0.0f;
};

} // namespace pyro

// ==============================================================================
// 关键点：在头文件末尾包含模板实现文件
// ==============================================================================
#include "pyro_algo_leso.tpp"

#endif // __PYRO_ALGO_LESO_H__