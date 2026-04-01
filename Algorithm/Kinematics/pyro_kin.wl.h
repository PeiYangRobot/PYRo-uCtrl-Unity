/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 19:32:10
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-26 14:43:33
 * @Description: 
 * The kinematic solve algorithm for wheel legged robot. If you want to use,
 * define a variable which type is wheel_legged_kin_t, than call its init 
 * function. After init, you can call solve function and vmc function to 
 * solve physical angles and do force mapping. This file uses the arm math
 * library to accelerate trigonometric calculation, so you need to add arm math
 * library to your project and enable it in your build system.
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */


#ifndef __PYRO_KIN_WL_H__
#define __PYRO_KIN_WL_H__

#include "pyro_core_def.h"

#include "arm_math.h"// IWYU pragma: keep
#include <cstdint>
namespace pyro
{

class wheel_legged_kin_t
{
public:
/*The position of joints, the middle point between j1 and j2 is the origin.
 
       j2-ivl-j1      j6     <-------- derction of forward
       /       \     /  \
      br        \  elr   \
     /           \ /      \
     j3          j5       j7   j5-j6-j7-j8 are a parallelogram
      \         /  \      /
       lr      /   ebr   /
        \     /      \  / 
         \   /        j8 
          \ /         /
          j4         /
                    /          origin, j4 and j9 are in the same line
                   /
                  /
                 /
                /
               j9
*/
   //the cofficients for phi solve
    struct phi_k_t
    {
       float k0;
       float k1;
       float k2;
       float k3;
       float k4;
    };
    
    //the cofficients for polar coordinates solve
    struct polar_k_t
    {
       float k0;
       float k1;
       float k2;
       float k3;
    };
    
    //the cofficients for VMC transform matrix
    struct vmc_k_t
    {
       float k0;
       float k1;
    };

    /**
     * @description:    
       Initialize the kinematic solver with given cofficients.
     * @param {phi_k_t*} phi_k
       The cofficients for phi solve.
     * @param {polar_k_t*} polar_k
       The cofficients for polar coordinates solve.
     * @param {vmc_k_t*} vmc_k
       The cofficients for VMC transform matrix.
     * @return {*}
       PYRO_OK: OK.
       PYRO_PARAM_ERROR: The point of parameter is null.
       PYRO_ALREADY_INIT: The solver is already initialized.
     */
    status_t init( const phi_k_t* phi_k,
                   const polar_k_t* polar_k,
                   const vmc_k_t* vmc_k);
        
    /**
     * @description:  
       Solve polar coordinations of the end point and angles between litte rod 
       and direction of movement through the angles between big rod and 
       direction of movement.
     * @param {float} theta1
       Angle between j2-j3 and direction of movement, counter clockwise is 
       positive(rad).
     * @param {float} theta2 
       Angle between j1-j5 and direction of movement, counter clockwise if 
       positive(rad).
     * @param {float} *phi1  
       Return solved angle between j3-j4 and direction of movment, counter 
       clockwise is positive(rad).
     * @param {float} *phi2
       Return solved angle bwtween j4-j5 and directio of movement, counter 
       clockwise is positive(rad).
     * @param {float} *length 
       Return solved polar radius of j9(m).
     * @param {float} *alpha
       Return solved polar angle of j9, counter clockwise is positive(rad).
     * @return {*}
       PYRO_OK: OK.
       PYRO_NOT_FOUND: Call this function before init.
       PYRO_PARAM_ERROR: The point of parameter is null.
     */
    status_t solve(float theta1, float theta2, 
                   float *phi1,  float *phi2, 
                   float *length, float *alpha);

    
    /**
     * @description: 
       Return VMC transform matrix value.
     * @param {float} theta1
       Angle between j2-j3 and direction of movement, counter clockwise is 
       positive(rad). 
     * @param {float} theta2  
       Angle between j1-j5 and direction of movement, counter clockwise if 
       positive(rad).
     * @param {float} phi1
       Angle between j3-j4 and direction of movement, counter clockwise if 
       positive(rad).
     * @param {float} phi2
       Angle between j4-j5 and direction of movement, counter clockwise if 
       positive(rad).  
     * @param {float} length
       Polar radius of j9(m).
     * @param {float} alpha
       Polar angle of j9(rad).
    *  @param {float*} T_val
       Pointer to an array which stores the VMC transform matrix value. 
     * @return {*}
       PYRO_OK: OK
       PYRO_NOT_FOUND: Call this function before init.
       PYRO_PARAM_ERROR: The point of parameter is null.
     */
    status_t get_VMC_value(float theta1, float theta2,
                           float phi1, float phi2, 
                           float length, float alpha,
                           float *T_val);
   
private: 
    /* Flag to check whether the solver is initialized, 0 for not initialized,
        1 for initialized.*/
    uint8_t _is_inited{0};
   
    /* The cofficients for phi solve */
    phi_k_t _phi_k;
    /* The cofficients for polar coordinates solve */
    polar_k_t _polar_k;
    /* The cofficients for VMC transform matrix */
    vmc_k_t _vmc_k;


};
}
#endif
