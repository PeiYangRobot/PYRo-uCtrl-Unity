/**
 * @file pyro_algo_ols.h
 * @brief Header file for the PYRO C++ Ordinary Least Squares (OLS) class.
 *
 * This file defines the `pyro::ols_t` class, which encapsulates
 * OLS linear regression functionality, primarily for signal differentiation.
 *
 * @author Wang Hongxi (Original C)
 * @author Lucky (C++ Refactor)
 * @version 1.0.0
 * @date 2025-11-13
 */

#ifndef __PYRO_ALGO_OLS_H__
#define __PYRO_ALGO_OLS_H__

#include <cstdint>
#include <vector>

namespace pyro
{

/**
 * @brief Ordinary Least Squares (OLS) Linear Regression C++ Implementation.
 *
 * This class refactors the C implementation into an object-oriented
 * C++ class. It uses std::vector for automatic memory management (RAII)
 * and provides a clean API for updating data and retrieving results.
 */
class ols_t
{
  public:
    /**
     * @brief Constructs the OLS filter.
     * @param order The number of samples (window size) for regression.
     * Note: order must be greater than 1.
     */
    explicit ols_t(uint16_t order);

    /**
     * @brief Updates the OLS filter with a new data point.
     *
     * This function adds the new (deltax, y) point to the data window,
     * pushing out the oldest point. It then recalculates the
     * slope (k), intercept (b), and mean absolute deviation.
     *
     * @param deltax The time elapsed (or change in x) since the last point.
     * @param y The new y-value (signal value).
     */
    void update(float deltax, float y);

    /**
     * @brief Gets the calculated derivative (slope 'k').
     * @return The current derivative (k) of the linear regression.
     */
    float get_derivative() const;

    /**
     * @brief Gets the smoothed signal value.
     * @return The most recent smoothed y-value, calculated as (k * x_last + b).
     */
    float get_smooth() const;

    /**
     * @brief Gets the Mean Absolute Deviation.
     * @note The original C code named this 'StandardDeviation',
     * but the calculation is Mean Absolute Deviation (MAD).
     * @return The current mean absolute deviation of the regression line.
     */
    float get_mean_absolute_deviation() const;

  private:
    // Member variables
    uint16_t _order;
    uint32_t _count;

    std::vector<float> _x;
    std::vector<float> _y;

    float _k; // Slope (k)
    float _b; // Intercept (b)

    float _deviation; // Mean absolute deviation
};

} // namespace pyro

#endif // __PYRO_ALGO_OLS_H__