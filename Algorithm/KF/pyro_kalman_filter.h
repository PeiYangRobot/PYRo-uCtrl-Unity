/*
 * @Author: vod-x vod_x@outlook.com
 * @Date: 2026-03-17 20:37:19
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-04-06 14:37:02
 * @FilePath: \Wheel-Legged-Robot\embedded_system\PYRo\Algorithm\KF\pyro_kalman_filter.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __PYRO_KALMAN_FILTER_H
#define __PYRO_KALMAN_FILTER_H

#include "pyro_core_def.h"
#include "arm_math.h"

namespace pyro
{

class kf_t
{
using mat = arm_matrix_instance_f32;
public:
    /**
     * @description: Constructor for the Kalman filter class
     * @param {uint8_t} xhatSize Size of the state estimate vector
     * @param {uint8_t} uSize Size of the control input vector
     * @param {uint8_t} zSize Size of the measurement vector
     * @return {*}
     */
    kf_t(uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
    ~kf_t();

    /**
     * @description: Initialize the Kalman filter with the given matrices
     * @param {float} *A_data State transition matrix data
     * @param {float} *B_data Control input matrix data
     * @param {float} *H_data Measurement matrix data
     * @param {float} *Q_data Estimate error covariance matrix data
     * @param {float} *R_data Process noise covariance matrix data
     * @return {status_t} 
     *          PYRO_OK if initialization is successful
     *          PYRO_PARAM_ERROR if the size of matrix data does not match the 
     *          expected size
     *          PYRO_NO_MEMORY if memory allocation fails
     *          PYRO_ALREADY_INIT if the filter is already initialized
     *          PYRO_ERROR if the point is null or any other error occurs
     */
    status_t init(float *A_data, float *B_data, float *H_data, float *Q_data, float *R_data);

    /**
     * @description: Update the Kalman filter with the given measurement and control vectors
     * @param {float} *measure_vec Measurement vector
     * @param {float} *control_vec Control input vector
     * @param {float} *estimated_ret Estimated state vector
     * @return {status_t} 
     *          PYRO_OK if update is successful
     *          PYRO_PARAM_ERROR if the size of vectors does not match the expected size
     *          PYRO_ERROR if the point is null or any other error occurs
     *          PYRO_NOT_FOUND if the filter is not initialized
     */
    status_t update(float *measure_vec, float *control_vec, float *estimated_ret);

private:
/*    MATRIX    */
/* State transition matrix */
    mat *_A;
/* Control input matrix */
    mat *_B;
/* Measurement matrix */
    mat *_H;
/* Estimate error covariance */
    mat *_Q;
/* Process noise covariance */
    mat *_R;

/* Kalman gain */
    mat *_K;
/* prior residual covariance */
    mat *_P_minus;;
/* posterior residual covariance */
    mat *_P;

/*    VECTOR    */
/* posterior state estimate vector */
    mat *_xhat;
    float *_xhat_data;
/* prior state estimate vector */
    mat *_xhat_minus;
    float *_xhat_minus_data;
/* Measurement vector */
    mat *_z;
    float *_z_data;
/* Control input vector */
    mat *_u;
    float *_u_data;

/*    SIZE   */
    uint8_t _xhatSize;
    uint8_t _uSize;
    uint8_t _zSize;
    
    uint8_t _is_init;
};

}

#endif
