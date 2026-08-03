/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-07 15:14:47
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-04-25 15:47:14
 * @Description: Kinematics solver implementation for wheel-legged robot / 轮腿机器人运动学解析正逆解及 VMC 映射矩阵计算算法实现
 * 
 * The kinematic solve algorithm for wheel legged robot. If you want to use,
 * define a variable which type is wheel_legged_kin_t, than call its init 
 * function. After init, you can call solve function and vmc function to 
 * solve physical angles and do force mapping. This file uses the arm math
 * library to accelerate trigonometric calculation, so you need to add arm math
 * library to your project and enable it in your build system. 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

#include "pyro_kin.wl.h"

    // //dx
    // dx = -(_vmc_k.k0 * d_theta1 * arm_sin_f32(theta1)) / (_vmc_k.k1) 
    //     -(_vmc_k.k0 * arm_sin_f32(*phi1) 
    //     * (d_theta1 * arm_sin_f32(*phi2 - theta1) 
    //     - d_theta2 * arm_sin_f32(*phi2 - theta2))) 
    //     / (_vmc_k.k1 * arm_sin_f32(phi1 - phi2));
    // //dy
    // dy = (_vmc_k.k0 * d_theta1 * arm_cos_f32(theta1)) / (_vmc_k.k1) 
    //     +(_vmc_k.k0 * arm_cos_f32(*phi1) 
    //     * (d_theta1 * arm_sin_f32(*phi2 - theta1) 
    //     - d_theta2 * arm_sin_f32(*phi2 - theta2))) 
    //     / (_vmc_k.k1 * arm_sin_f32(phi1 - phi2));
    // dx = -(_vmc_k.k0*(d_theta1*arm_sin_f32(*phi1)*arm_sin_f32(*phi2 - theta1) 
    // + d_theta1*arm_sin_f32(theta1)*arm_sin_f32(*phi1 - *phi2) 
    // - d_theta2*arm_sin_f32(*phi1)*arm_sin_f32(*phi2 - theta2)))
    // /(_vmc_k.k1*arm_sin_f32(*phi1 - *phi2));
    // dy = (_vmc_k.k0*(d_theta1*arm_cos_f32(*phi1)*arm_sin_f32(*phi2 - theta1) 
    // + d_theta1*arm_cos_f32(theta1)*arm_sin_f32(*phi1 - *phi2) 
    // - d_theta2*arm_cos_f32(*phi1)*arm_sin_f32(*phi2 - theta2)))
    // /(_vmc_k.k1*arm_sin_f32(*phi1 - *phi2));
#define WL_KIN_D_LENGTH_LPF_RC 0.000f /* Cutoff constant for leg length rate LPF / 极轴伸缩速率低通滤波时间常数 */
#define WL_KIN_D_ALPHA_LPF_RC 0.0f    /* Cutoff constant for polar angle rate LPF / 极轴偏摆角速度低通滤波时间常数 */

using namespace pyro;
float ax, ay;

/**
 * @brief Initialize kinematics solver parameters.
 *        初始化运动解算器参数与滤波器状态。
 * @param phi_k Kinematic coefficients for 5-bar closed-loop equations / 五连杆闭环计算几何系数
 * @param polar_k Kinematic coefficients for polar coordinates mapping / 极坐标映射几何系数
 * @param vmc_k VMC projection scaling factors / 虚拟模型控制投影系数
 * @return status_t Status code / 执行状态
 */
status_t wheel_legged_kin_t::init( const phi_k_t *phi_k,
                              const polar_k_t *polar_k,
                              const vmc_k_t *vmc_k)
{
    /* Check if the solver is already initialized / 检查是否已经初始化过 */
    if (_is_inited)
    {
        return PYRO_ALREADY_INIT;
    }

    /* Check if the input pointers are not null / 校验输入结构体指针是否为空 */
    CHECK_POINT_NULL(phi_k)
    CHECK_POINT_NULL(polar_k)
    CHECK_POINT_NULL(vmc_k)
    
    /* Copy the input coefficients to the member variables / 复制运动学拟合常数系数 */
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
    
    /* Reset LPF state buffers / 重置左右两腿速率滤波器索引及缓存 */
    _solve_filter_idx = 0;
    _d_length_lpf[0] = _d_length_lpf[1] = 0.0f;
    _d_alpha_lpf[0] = _d_alpha_lpf[1] = 0.0f;

    /* Mark the solver as initialized / 标记初始化完成 */
    _is_inited = 1;
    return PYRO_OK;
}

/**
 * @brief Solve forward kinematics: calculate polar coordinates and rates from joint data.
 *        解析正运动学：输入关节主动角度与角速度，求解极径高度、极角、平动速度及笛卡尔位移。
 * @param theta1 Right/Front joint motor angle / 右/前侧主动臂电机角 (rad)
 * @param theta2 Left/Back joint motor angle / 左/后侧主动臂电机角 (rad)
 * @param d_theta1 Right/Front joint motor speed / 右/前侧电机角速度 (rad/s)
 * @param d_theta2 Left/Back joint motor speed / 左/后侧电机角速度 (rad/s)
 * @param phi1 [out] Solved intermediate angle phi1 / 计算得到的右侧小臂辅助角 phi1 (rad)
 * @param phi2 [out] Solved intermediate angle phi2 / 计算得到的左侧小臂辅助角 phi2 (rad)
 * @param alpha [out] Stance polar angle alpha / 极坐标极角 (rad)
 * @param length [out] Stance polar length l / 极坐标极径/实际腿长 (m)
 * @param d_length [out] Low-pass filtered leg extension velocity / 滤波后的极径伸缩速率 (m/s)
 * @param d_alpha [out] Low-pass filtered leg angular velocity / 滤波后的极角摆动角速度 (rad/s)
 * @param d_x [out] Cartesian coordinate velocity along x / 触地点在机体坐标系下的 x 方向线速度 (m/s)
 * @param d_y [out] Cartesian coordinate velocity along y / 触地点在机体坐标系下的 y 方向线速度 (m/s)
 * @param x [out] Cartesian coordinate x / 触地点在机体坐标系下的 x 轴相对位移 (m)
 * @param y [out] Cartesian coordinate y / 触地点在机体坐标系下的 y 轴相对位移 (m)
 * @return status_t Status code / 执行状态
 */
status_t wheel_legged_kin_t::solve(float theta1, float theta2,
                                 float d_theta1, float d_theta2,
                                 float *phi1,  float *phi2,
                                 float *alpha, float *length,
                                 float *d_length, float *d_alpha,
                                 float *d_x, float *d_y,
                                 float *x, float *y)
{
    arm_status ret; // Variable to store return value of arm math functions / ARM 数学函数库执行状态返回值
    float temp1, temp2;  // Two temporary variables to store calculation results / 存放中间运算过程的局部变量

    /* 1. Check the status of solver / 1. 检验解算器初始化状态 */
    if (!_is_inited)
    {
        return PYRO_NOT_FOUND;
    }
    
    /* 1.2 Check if the output pointers are not null / 1.2 校验输出指针，防止野指针解引用 */
    CHECK_POINT_NULL(phi1)
    CHECK_POINT_NULL(phi2)
    CHECK_POINT_NULL(alpha)
    CHECK_POINT_NULL(length)
    CHECK_POINT_NULL(d_length)
    CHECK_POINT_NULL(d_alpha)

    /* 2. Calculate phi1, phi2 using tangent half-angle substitution
       2. 根据万能公式消元求解非线性连杆方程，获得小臂摆角 phi1 与 phi2 */
    /* 2.1 Calculate the numerator radical term for phi Weierstrass equations
       2.1 求解 DARE/Weierstrass 根方程判别式的根号分子项 */
    ret = arm_sqrt_f32((_phi_k.k1 - _phi_k.k2*arm_cos_f32(theta1 - theta2)  
                - _phi_k.k3*arm_cos_f32(2*theta1 - 2*theta2)), &temp1);
    CHECK_ARM_MATH_RET(ret);
    temp1 = _phi_k.k0*arm_sin_f32(theta1) - _phi_k.k0*arm_sin_f32(theta2)
                                                                      + temp1;
    /* 2.2 Calculate phi1 by solving atan2 / 2.2 求 atan2 解出右小臂倾角 phi1 */
    ret = arm_atan2_f32(temp1, (_phi_k.k0*arm_cos_f32(theta1) 
                                  - _phi_k.k0*arm_cos_f32(theta2) 
                                  + _phi_k.k4*arm_cos_f32(theta1 - theta2)
                                  - _phi_k.k4), &temp2);
    CHECK_ARM_MATH_RET(ret);
    *phi1 = (2 * temp2);
    
    /* 2.3 Calculate phi2 (symmetry equations, signs for k4 are flipped)
       2.3 求 atan2 解出左小臂倾角 phi2，公式与 phi1 仅在 K4 cos 项和常数项上符号相反 */
    ret = arm_atan2_f32(temp1, (_phi_k.k0*arm_cos_f32(theta1) 
                                  - _phi_k.k0*arm_cos_f32(theta2) 
                                  - _phi_k.k4*arm_cos_f32(theta1 - theta2)
                                  + _phi_k.k4), &temp2);
    CHECK_ARM_MATH_RET(ret);
    *phi2 = (2 * temp2);

    /* 3. Calculate the position in polar coordinates
       3. 极坐标几何解算：根据大臂角 theta1 与小臂角 phi1 求解触地点位置 */
    /* 3.1 Calculate polar angle alpha / 3.1 解算极坐标摆角 alpha */
    ret = arm_atan2_f32(( _polar_k.k0*arm_sin_f32(*phi1))/_polar_k.k2 
        + (_polar_k.k1*arm_sin_f32(theta1))/_polar_k.k3,
    ( _polar_k.k0*arm_cos_f32(*phi1))/_polar_k.k2 
        + (_polar_k.k1*arm_cos_f32(theta1))/_polar_k.k3, &temp2);
    CHECK_ARM_MATH_RET(ret);
    *alpha = temp2;

    /* 3.2 Calculate polar length l (using intermediate Cartesian x, y)
       3.2 解算笛卡尔坐标 x, y 并通过模长求解极径摆长 length */
    *x = ( _polar_k.k0*arm_cos_f32(*phi1))/_polar_k.k2 + 
                            (_polar_k.k1*arm_cos_f32(theta1))/_polar_k.k3;
    temp1 = (*x) * (*x);
    *y = ( _polar_k.k0*arm_sin_f32(*phi1))/_polar_k.k2 +
                            (_polar_k.k1*arm_sin_f32(theta1))/_polar_k.k3;
    temp2 = (*y) * (*y);
    ret = arm_sqrt_f32(temp1 + temp2, length);
    CHECK_ARM_MATH_RET(ret);

    /* 4. Calculate the differential of polar coordinates (same constants as VMC)
       4. 根据雅可比微分项，计算笛卡尔速度及极坐标变化率 */
    /* 4.1 Calculate Cartesian velocity dx and dy / 4.1 求解机体坐标系下的触点线速度 */
    *d_x = -d_theta1 * (21059*arm_sin_f32(*phi2)*arm_sin_f32(*phi1 - theta1))/(100000*arm_sin_f32(*phi1 - *phi2))+d_theta2*(21059*arm_sin_f32(*phi1)*arm_sin_f32(*phi2 - theta2))/(100000*arm_sin_f32(*phi1 - *phi2));
    *d_y = d_theta1 * (21059*arm_cos_f32(*phi2)*arm_sin_f32(*phi1 - theta1))/(100000*arm_sin_f32(*phi1 - *phi2)) - d_theta2 * (21059*arm_cos_f32(*phi1)*arm_sin_f32(*phi2 - theta2))/(100000*arm_sin_f32(*phi1 - *phi2));
        
        constexpr float solver_dt = 0.001f;
        const uint8_t filter_idx = _solve_filter_idx;
        
        /* Coordinate translation from Cartesian rates to polar rates
           利用投影变换，解算出无滤波的极径变动速率 raw_d_length 与极轴角速度 raw_d_alpha */
        const float raw_d_length = (*x * (*d_x) + *y * (*d_y)) / (*length);
        const float raw_d_alpha = (*x * (*d_y) - *y * (*d_x)) / (*length * *length);
        
        /* Low pass filtering / 一阶低通滤波平滑 */
        _d_length_lpf[filter_idx] = _d_length_lpf[filter_idx] * WL_KIN_D_LENGTH_LPF_RC /
                                                                (solver_dt + WL_KIN_D_LENGTH_LPF_RC)
                                                            + raw_d_length * solver_dt /
                                                                (solver_dt + WL_KIN_D_LENGTH_LPF_RC);
        _d_alpha_lpf[filter_idx] = _d_alpha_lpf[filter_idx] * WL_KIN_D_ALPHA_LPF_RC /
                                                             (solver_dt + WL_KIN_D_ALPHA_LPF_RC)
                                                         + raw_d_alpha * solver_dt /
                                                             (solver_dt + WL_KIN_D_ALPHA_LPF_RC);
        *d_length = _d_length_lpf[filter_idx];
        *d_alpha = _d_alpha_lpf[filter_idx];
        _solve_filter_idx = (filter_idx + 1U) & 0x01U; /* Ping-pong index swap / 双腿缓冲翻转 */
        
    *d_x = *d_x;
    *d_y = *d_y;
    // static float l_temp1, l_temp2;
    // if(abs(temp1 - l_temp1) > 0.001f)ax+= temp1;
    // if(abs(temp2 - l_temp2) > 0.001f)ay+= temp2;
    
    // l_temp1 = temp1;
    // l_temp2 = temp2;
    
    // /* 4.2 calculate differential alpha */
    // *d_alpha = (arm_arm_sin_f32_f32(*alpha) * temp1 + arm_arm_cos_f32_f32(*alpha) * temp2) / (*length);
    // /* 4.3 calculate differential length */
    // *d_length = arm_sin_f32(*alpha) * temp2 - arm_cos_f32(*alpha) * temp1;

    return PYRO_OK;
}

/**
 * @brief Calculate the VMC force-to-torque Jacobian projection matrix values.
 *        计算虚拟模型控制（VMC）力矩投影雅可比矩阵 T_val (2x2)。
 *        该矩阵满足：[T_front, T_rear]^T = T_val * [F_leg, T_hip]^T
 * @param theta1 Front joint angle / 前主动臂角度 (rad)
 * @param theta2 Rear joint angle / 后主动臂角度 (rad)
 * @param phi1 Solved right intermediate angle / 解算所得右侧辅助角 phi1 (rad)
 * @param phi2 Solved left intermediate angle / 解算所得左侧辅助角 phi2 (rad)
 * @param length Stance polar length l / 极坐标极径腿长 (m)
 * @param alpha Stance polar angle alpha / 极坐标偏角 (rad)
 * @param T_val [out] Array to store the 4 elements of the Jacobian transpose matrix J^T / 存储 4 个矩阵元素的输出数组
 * @return status_t Status code / 执行状态
 */
status_t wheel_legged_kin_t::get_VMC_value(float theta1, float theta2,
                                       float phi1, float phi2, 
                                       float length, float alpha,
                                       float *T_val)
{
    /* 1. Check the status of solver / 1. 验证初始化状态 */
    if (!_is_inited)
    {
        return PYRO_NOT_FOUND;
    }

    /* 1.2 Check if output pointer is valid / 1.2 校验指针 */
    CHECK_POINT_NULL(T_val)

    /* 2. Calculate VMC transform matrix values: J^T * R_rot * N_scale
       2. 解析计算雅可比转置映射矩阵 T_val 成员 (2x2) */
    /* Row 0, Col 0: Maps F_leg to T_front / 前臂高度力转换项 */
    T_val[0] = ( _vmc_k.k0*arm_cos_f32(phi2)*arm_sin_f32(alpha)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        - (_vmc_k.k0*arm_cos_f32(alpha)*arm_sin_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2));
        
    /* Row 0, Col 1: Maps T_hip to T_front (scaled by 1/length) / 前臂摆动力矩转换项 */
    T_val[1] = ((_vmc_k.k0*arm_cos_f32(alpha)*arm_cos_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        + (_vmc_k.k0*arm_sin_f32(alpha)*arm_sin_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)))
        /length;
        
    /* Row 1, Col 0: Maps F_leg to T_rear / 后臂高度力转换项 */
    T_val[2] = ( _vmc_k.k0*arm_cos_f32(alpha)*arm_sin_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        - (_vmc_k.k0*arm_cos_f32(phi1)*arm_sin_f32(alpha)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2));
        
    /* Row 1, Col 1: Maps T_hip to T_rear (scaled by 1/length) / 后臂摆动力矩转换项 */
    T_val[3] = -((_vmc_k.k0*arm_cos_f32(alpha)*arm_cos_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)) 
        + (_vmc_k.k0*arm_sin_f32(alpha)*arm_sin_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(_vmc_k.k1*arm_sin_f32(phi1 - phi2)))
        /length;
    
    return PYRO_OK;
}
