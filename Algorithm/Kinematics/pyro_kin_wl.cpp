/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-07 15:14:47
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-07 22:47:21
 * @Description: 
 * The kinematic solve algorithm for wheel legged robot. If you want to use,
 * define a variable which type is wheel_legged_kin_t, than call its init 
 * function. After init, you can call solve function and vmc function to 
 * solve physical angles and do force mapping. This file uses the arm math
 * library to accelerate trigonometric calculation, so you need to add arm math
 * library to your project and enable it in your build system. 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

#include "pyro_kin.wl.h"

using namespace pyro;

status_t wheel_legged_kin_t::init( const phi_k_t *phi_k,
                              const polar_k_t *polar_k,
                              const vmc_k_t *vmc_k)
{
    /* Check if the solver is already initialized */
    if (_is_inited)
    {
        return PYRO_ALREADY_INIT;
    }

    /* Check if the input pointers are not null */
    CHECK_POINT_NULL(phi_k)
    CHECK_POINT_NULL(polar_k)
    CHECK_POINT_NULL(vmc_k)
    
    /* Copy the input cofficients to the member variables */
    _phi_k.k0 = phi_k->k0;
    _phi_k.k1 = phi_k->k1;
    _phi_k.k2 = phi_k->k2;
    _phi_k.k3 = phi_k->k3;
    _phi_k.k4 = phi_k->k4;
    _polar_k.k0 = polar_k->k0;
    _polar_k.k1 = polar_k->k1;
    _polar_k.k2 = polar_k->k2;
    _polar_k.k3 = polar_k->k3;
    _vmc_k.k0 = vmc_k->k0;
    _vmc_k.k1 = vmc_k->k1;

    /* Mark the solver as initialized */
    _is_inited = 1;
    return PYRO_OK;
}

status_t wheel_legged_kin_t::solve(float theta1, float theta2,
                                 float *phi1,  float *phi2,
                                 float *alpha, float *length)
{
    arm_status ret; //variable to store return value of arm math functions
    float temp1, temp2;  //two temporary variables to store claculation results

    /* 1. Check the status of solver*/
    /* 1.1 Check if the solver is initialized */
    if (!_is_inited)
    {
        return PYRO_NOT_FOUND;
    }
    /* 1.2 Check if the output pointers are not null */
    CHECK_POINT_NULL(phi1)
    CHECK_POINT_NULL(phi2)
    CHECK_POINT_NULL(alpha)
    CHECK_POINT_NULL(length)
    /* 2. Calculate phi1, phi2 */
    /* 2.1 calculate the molecule of phi1 and phi2 */
    ret = arm_sqrt_f32((_phi_k.k1 - _phi_k.k2*arm_cos_f32(theta1 - theta2)  
                - _phi_k.k3*arm_cos_f32(2*theta1 - 2*theta2)), &temp1);
    CHECK_ARM_MATH_RET(ret);
    temp1 = _phi_k.k0*arm_sin_f32(theta1) - _phi_k.k0*arm_sin_f32(theta2)
                                                                      + temp1;
    /* 2.2 calculate phi1 */
    ret = arm_atan2_f32(temp1, (_phi_k.k0*arm_cos_f32(theta1) 
                                  - _phi_k.k0*arm_cos_f32(theta2) 
                                  + _phi_k.k4*arm_cos_f32(theta1 - theta2)
                                  - _phi_k.k4), &temp2);
    CHECK_ARM_MATH_RET(ret);
    *phi1 = (2 * temp2);
    /* 2.3 calculate phi2, the diffrenece with phi1 is the sign of K4 cos theta1 
       and constant K4 */
    ret = arm_atan2_f32(temp1, (_phi_k.k0*arm_cos_f32(theta1) 
                                  - _phi_k.k0*arm_cos_f32(theta2) 
                                  - _phi_k.k4*arm_cos_f32(theta1 - theta2)
                                  + _phi_k.k4), &temp2);
    CHECK_ARM_MATH_RET(ret);
    *phi2 = (2 * temp2);
    /* 3. Calculate the position in polar coordinates */
    /* 3.1 calculate alpha */
    ret = arm_atan2_f32(( _polar_k.k0*arm_sin_f32(*phi1))/_polar_k.k2 
        + (_polar_k.k1*arm_sin_f32(theta1))/_polar_k.k3,
    ( _polar_k.k0*arm_cos_f32(*phi1))/_polar_k.k2 
        + (_polar_k.k1*arm_cos_f32(theta1))/_polar_k.k3, &temp2);
    CHECK_ARM_MATH_RET(ret);
    *alpha = temp2;
    /* 3.2 calculate l */
    temp1 = ( _polar_k.k0*arm_cos_f32(*phi1))/_polar_k.k2 + 
                            (_polar_k.k1*arm_cos_f32(theta1))/_polar_k.k3;
    temp1 = temp1 * temp1;
    temp2 = ( _polar_k.k0*arm_sin_f32(*phi1))/_polar_k.k2 +
                            (_polar_k.k1*arm_sin_f32(theta1))/_polar_k.k3;
    temp2 = temp2 * temp2;
    ret = arm_sqrt_f32(temp1 + temp2, length);
    CHECK_ARM_MATH_RET(ret);
    return PYRO_OK;
}

status_t wheel_legged_kin_t::get_VMC_value(float theta1, float theta2,
                                       float phi1, float phi2, 
                                       float length, float alpha,
                                       float *T_val)
{
    /* 1. Check the status of solver*/
    /* 1.1 Check if the solver is initialized */
    if (!_is_inited)
    {
        return PYRO_NOT_FOUND;
    }

    /* 1.2 Check if the input pointers are not null */
    CHECK_POINT_NULL(T_val)

    /* 2. Calculate VMC transform matrix values */
    T_val[0] = ( _vmc_k.k0*arm_cos_f32(phi2)*arm_sin_f32(alpha)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        - (_vmc_k.k0*arm_cos_f32(alpha)*arm_sin_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2));
    T_val[1] = -((_vmc_k.k0*arm_cos_f32(alpha)*arm_cos_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        - (_vmc_k.k0*arm_sin_f32(alpha)*arm_sin_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)))
        /length;
    T_val[2] = ( _vmc_k.k0*arm_cos_f32(alpha)*arm_sin_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        - (_vmc_k.k0*arm_cos_f32(phi1)*arm_sin_f32(alpha)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2));
    T_val[3] = ((_vmc_k.k0*arm_cos_f32(alpha)*arm_cos_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        - (_vmc_k.k0*arm_sin_f32(alpha)*arm_sin_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)))
        /length;
    
    return PYRO_OK;
}