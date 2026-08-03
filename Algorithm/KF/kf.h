#ifndef KF_H
#define KF_H
#include "pyro_core_def.h"
#include "dsppp/memory_pool.hpp"
#include "dsppp/matrix.hpp"
#include "arm_math.h"
#include <cstddef>

namespace pyro
{

using mat = arm_cmsis_dsp::Matrix<float, arm_cmsis_dsp::DYNAMIC, arm_cmsis_dsp::DYNAMIC>;

class kf_t
{
public:
    /**
     * @description: Constructor for the dsppp-based Kalman filter class
     * @param {uint8_t} x_size Size of the state estimate vector
     * @param {uint8_t} u_size Size of the control input vector
     * @param {uint8_t} z_size Size of the measurement vector
     * @param {uint8_t} w_size Size of the process noise vector
     * @return {*}
     */
    kf_t(uint8_t x_size, uint8_t u_size, uint8_t z_size, uint8_t w_size);
    ~kf_t() = default;

    /**
     * @description: Initialize the Kalman filter with the given matrices
     * @param {float} *A_data State transition matrix data
     * @param {float} *B_data Control input matrix data
     * @param {float} *H_data Measurement matrix data
     * @param {float} *G_data Noise transition matrix data
     * @param {float} *Q_data Process noise covariance matrix data
     * @param {float} *R_data Measurement noise covariance matrix data
     * @return {status_t}
     *          PYRO_OK if initialization is successful
     *          PYRO_PARAM_ERROR if any pointer is null
     *          PYRO_ALREADY_INIT if the filter is already initialized
     */
    status_t init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data);

    /**
     * @description: Initialize with custom initial posterior state x0, default P0 = I
     * @param {float} *x0_data Initial state vector data, length = x_size
     */
    status_t init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
                  float *x0_data);

    /**
     * @description: Initialize with default x0 = 0 and custom initial posterior covariance P0
     * @param {std::nullptr_t} Null placeholder for x0
     * @param {float} *P0_data Initial covariance matrix data (row-major), shape = x_size x x_size
     */
    status_t init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
                  std::nullptr_t, float *P0_data);

    /**
     * @description: Initialize with custom initial posterior state x0 and covariance P0
     * @param {float} *x0_data Initial state vector data, length = x_size
     * @param {float} *P0_data Initial covariance matrix data (row-major), shape = x_size x x_size
     */
    status_t init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
                  float *x0_data, float *P0_data);

    /**
     * @description: Update the Kalman filter with the given measurement and control vectors
     * @param {float} *measure_vec Measurement vector
     * @param {float} *control_vec Control input vector
     * @param {float} *estimated_ret Output state estimate vector
     * @return {status_t}
     *          PYRO_OK if update is successful
     *          PYRO_PARAM_ERROR if any pointer is null
     *          PYRO_NOT_FOUND if the filter is not initialized
     *          PYRO_ERROR if matrix inverse fails
     */
    status_t update(float *measure_vec, float *control_vec, float *estimated_ret);

    /**
     * @description: Get the current posterior state estimate x_{k|k}
     * @param {float} *out Output buffer, length must be >= x_size
     * @return {status_t}
     *          PYRO_OK if successful
     *          PYRO_PARAM_ERROR if out is null
     *          PYRO_NOT_FOUND if the filter is not initialized
     */
    status_t get_state(float *out) const;

    /**
     * @description: Reset state vector to zero and error covariance to identity
     * @return {status_t}
     *          PYRO_OK if successful
     *          PYRO_NOT_FOUND if the filter is not initialized
     */
    status_t reset();

    /**
     * @description: Reset state vector and error covariance to given values
     * @param {float} *x0_data New state vector data, length = x_size
     * @param {float} *P0_data New covariance matrix data (row-major), shape = x_size x x_size
     * @return {status_t}
     *          PYRO_OK if successful
     *          PYRO_PARAM_ERROR if any pointer is null or P0 validation fails
     *          PYRO_NOT_FOUND if the filter is not initialized
     */
    status_t reset(float *x0_data, float *P0_data);

    /**
     * @description: Reset only state vector, keep current covariance
     * @param {float} *x0_data New state vector data, length = x_size
     */
    status_t reset(float *x0_data);

private:
    /* Matrix / vector utility helpers */
    /**
     * @description: Fill a matrix from row-major raw data buffer
     * @param {mat} &matrix Destination matrix
     * @param {const float} *data Source data pointer, length = rows * cols
     */
    static void fill_mat(mat &matrix, const float *data);

    /**
     * @description: Fill all elements of a matrix with the same scalar value
     * @param {mat} &matrix Destination matrix
     * @param {float} value Scalar value to broadcast
     */
    static void fill_scalar(mat &matrix, float value);

    /**
     * @description: Assign raw vector data to an internal column vector
     * @param {mat} &vector Destination vector, expected shape = (N x 1)
     * @param {const float} *data Source data pointer, length = N
     */
    static void assign_vector(mat &vector, const float *data);

    /**
     * @description: Copy one internal column vector into another
     * @param {mat} &dst Destination vector, expected shape = (N x 1)
     * @param {const mat} &src Source vector, expected shape = (N x 1)
     */
    static void copy_vector(mat &dst, const mat &src);

    /**
     * @description: Set all elements of an internal column vector to zero
     * @param {mat} &vector Destination vector, expected shape = (N x 1)
     */
    static void clear_vector(mat &vector);

    /**
     * @description: Convert a square matrix to identity form
     * @param {mat} &matrix Destination square matrix
     */
    static void set_identity(mat &matrix);

    /**
     * @description: Validate user-provided initial covariance matrix data
     * @param {const float} *data Covariance matrix data in row-major format
     * @param {uint8_t} n Matrix order (n x n)
     * @return {bool} true if data passes basic covariance checks
     */
    static bool validate_covariance_data(const float *data, uint8_t n);

    /**
     * @description: Invert matrix using CMSIS-DSP backend
     * @param {mat} &src Source matrix
     * @param {mat} &dst Destination matrix for inverse result
     * @return {arm_status} CMSIS-DSP status code
     */
    static arm_status inverse_matrix(mat &src, mat &dst);

    /**
     * @description: Shared implementation for all init overloads
     * @param {float} *A_data State transition matrix data
     * @param {float} *B_data Control input matrix data
     * @param {float} *H_data Measurement matrix data
     * @param {float} *G_data Noise transition matrix data
     * @param {float} *Q_data Process noise covariance matrix data
     * @param {float} *R_data Measurement noise covariance matrix data
     * @param {const float} *x0_data Optional initial state vector data, nullptr -> zero vector
     * @param {const float} *P0_data Optional initial covariance matrix data, nullptr -> identity matrix
     * @return {status_t}
     */
    status_t init_impl(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
                       const float *x0_data, const float *P0_data);

    /* Problem size definition */
    /* State vector dimension: x in R^{x_size} */
    uint8_t _x_size;
    /* Control vector dimension: u in R^{u_size} */
    uint8_t _u_size;
    /* Measurement vector dimension: z in R^{z_size} */
    uint8_t _z_size;
    /* Process noise vector dimension: w in R^{w_size} */
    uint8_t _w_size;

    /* Initialization flag, true after successful init() */
    bool _is_init;

    /* System model matrices */
    /* State transition matrix A (x_size x x_size) */
    mat _mat_A;
    /* Control input matrix B (x_size x u_size) */
    mat _mat_B;
    /* Measurement matrix H (z_size x x_size) */
    mat _mat_H;
    /* Noise transition matrix G (x_size x w_size) */
    mat _mat_G;
    /* Process noise covariance Q (w_size x w_size) */
    mat _mat_Q;
    /* Measurement noise covariance R (z_size x z_size) */
    mat _mat_R;

    /* Kalman core matrices */
    /* Kalman gain K (x_size x z_size) */
    mat _mat_K;
    /* Posterior covariance P_k (x_size x x_size) */
    mat _mat_P;
    /* Prior covariance P_k^- (x_size x x_size) */
    mat _mat_P_minus;

    /* State / input / measurement vectors */
    /* Posterior state estimate x_k|k (x_size x 1) */
    mat _vec_xhat;
    /* Prior state estimate x_k|k-1 (x_size x 1) */
    mat _vec_xhat_minus;
    /* Measurement vector z_k (z_size x 1) */
    mat _vec_z;
    /* Control vector u_k (u_size x 1) */
    mat _vec_u;

    /* Identity matrix I (x_size x x_size), used in Joseph covariance update */
    mat _mat_I;

    /* Temporary matrices for update() (grouped by shape) */
    /* x_size x 1: typically A*x, B*u and similar state-vector intermediates */
    mat _tmp_x_1;
    /* x_size x 1: secondary state-vector temporary */
    mat _tmp_x_2;
    /* z_size x 1: predicted measurement H*x^- */
    mat _tmp_z_1;

    /* x_size x x_size: covariance update intermediates */
    mat _tmp_xx_1;
    /* x_size x x_size: covariance update intermediates */
    mat _tmp_xx_2;
    /* x_size x x_size: A_j = (I - K*H) in Joseph form */
    mat _tmp_xx_3;

    /* Transposed noise transition matrix G^T (w_size x x_size) */
    mat _mat_Gt;
    /* Transposed measurement matrix H^T (x_size x z_size) */
    mat _mat_Ht;
    /* Transposed Kalman gain K^T (z_size x x_size) */
    mat _mat_Kt;

    /* Innovation covariance S = H*P^-*H^T + R (z_size x z_size) */
    mat _mat_S;
    /* Inverse innovation covariance S^{-1} (z_size x z_size) */
    mat _mat_S_inv;

    /* x_size x z_size: P^-*H^T intermediate for gain computation */
    mat _tmp_xz_1;
    /* x_size x z_size: K*R intermediate for Joseph form */
    mat _tmp_xz_2;
    /* x_size x w_size: G*Q intermediate for covariance prediction */
    mat _tmp_xw_1;
};
}
#endif
