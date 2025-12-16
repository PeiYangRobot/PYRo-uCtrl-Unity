/**
 * @file pyro_power_control_drv.cpp
 * @brief 功率控制驱动实现文件
 *
 * 该文件包含功率控制驱动类 power_control_drv_t 所有成员函数的具体实现，
 * 包括电机系数设置、功率预测、电流限制计算、批量电流限制等核心逻辑，
 * 实现了基于功率模型的多电机电流动态限制功能。
 * @namespace: pyro
 * 
 * @authors: Butterbutterfly0（架构）、Pason（实现）
 * @date: 2025-11-26
 * @version: 1.0
 */
#include "pyro_power_control_drv.h"
#include <math.h>

namespace pyro
{

/**
 * @brief 设置电机的功率系数
 *
 * 为指定索引的电机设置功率预测模型的系数。
 * 注意：此函数实现中将输入的 `motor_index` 减 1，表明它期望的是从 1 开始的索引。
 *
 * @param motor_index 电机的索引 (1-based)。
 * @param coefficient 包含四个系数的结构体。
 */
void power_control_drv_t::set_motor_coefficient(int motor_index, const motor_coefficient_t& coefficient)
{
    int zero_based_index = motor_index - 1; // 转换为 0-based 索引
    if (zero_based_index >= 0 && zero_based_index < static_cast<int>(_motor_coefficients.size())) 
    {
            _motor_coefficients[zero_based_index] = coefficient;
    }
}

/**
 * @brief 预测电机功率
 *
 * 使用预存的电机系数和当前的电流、角速度计算预测的电机功率消耗。
 * 公式: P = k1*tau*gyro + k2*|gyro| + k3*tau^2 + k4
 *
 * @param motor_index 电机的索引 (0-based)。
 * @param tau 当前的电流指令 (N·m)。
 * @param gyro 当前的角速度 (rad/s)。
 * @return float 预测的功率值 (W)。如果索引无效，返回 0.0f。
 */
float power_control_drv_t::motor_power_predict(int motor_index, float tau, float gyro) const
{
    if (motor_index >= 0 && motor_index < static_cast<int>(_motor_coefficients.size())) 
    {
        const motor_coefficient_t& c = _motor_coefficients[motor_index];
        return c.k1 * tau * gyro + c.k2 * std::fabs(gyro) + c.k3 * tau * tau + c.k4;
    }
    else
    {
        // 索引越界，返回0
        return 0.0f;
    }
}

/**
 * @brief 计算限制后电流
 *
 * 给定一个期望电流和功率限制，通过解二次方程来找到最大允许的电流。
 * 方程源于功率预测公式: P_limit = k3*tau^2 + (k1*gyro)*tau + (k2*|gyro| + k4)
 * 这是一个关于 tau 的一元二次方程: a*tau^2 + b*tau + c = 0
 *
 * @param motor_index 电机的索引。
 * @param origin_torque 原始的、未受限制的电流指令（用于判断符号）。
 * @param gyro 当前的角速度。
 * @param restricted_power 允许的最大功率。
 * @return float 计算出的限制后电流。
 */
float power_control_drv_t::_motor_power_restrict_torque(int motor_index, float origin_torque, float gyro, float restricted_power) const
{
    // 假设 motor_index 已经是 0-based 且有效
    const motor_coefficient_t& c = _motor_coefficients[motor_index];
    
    float a = c.k3;
    float b = c.k1 * gyro;
    float c_term = c.k2 * std::fabs(gyro) + c.k4 - restricted_power;
    
    float delta = b * b - 4 * a * c_term; // 判别式

    if (delta <= 0) 
    {
        // 判别式小于等于0，方程无实根或有唯一实根，
        // 返回抛物线顶点的x坐标，这是功率最接近限制值的点。
        return -b / (2 * a);
    } 
    else 
    {
        // 有两个实根，根据原始电流的符号选择合适的根
        float sqrt_delta = std::sqrt(delta);
        float tau1 = (-b + sqrt_delta) / (2 * a);
        float tau2 = (-b - sqrt_delta) / (2 * a);

        if (origin_torque > 0) 
        {
            return tau1; // 取较大的正根
        } 
        else if (origin_torque < 0) 
        {
            return tau2; // 取较小的负根
        } 
        else 
        {
            return 0.0f; // 原始电流为0，直接返回0
        }
    }
}

/**
 * @brief 批量计算多个电机的限制后电流（内部实现）
 *
 * 这是实际执行计算的私有函数。它会根据总功率限制和功率分配比例，
 * 为每个电机计算其允许的最大功率，然后调用 `motor_power_restrict_torque` 获取限制后电流，
 * 最后应用一阶低通滤波以平滑电流变化。
 *
 * @param motor_data 指向 motor_data_t 结构体数组的指针。
 * @param motor_num 电机的数量。
 * @param power_limit 总的可用功率限制。
 * @param power_ratios 指向浮点数数组的指针，每个元素代表对应电机的功率分配比例。
 */
void power_control_drv_t::calculate_restricted_torques(
    motor_data_t* motor_data,
    int motor_num,
    float power_limit,
    const float* power_ratios
) const
{
    if (motor_num <= 0 || motor_data == nullptr || power_limit <= 0) {
        return;
    }

    // 初始化功率分配比例
    std::vector<float> ratios(motor_num);
    if (nullptr == power_ratios) 
    {
        // 如果未提供比例，则平均分配功率
        float avg_ratio = 1.0f / motor_num;
        for (int i = 0; i < motor_num; i++) 
        {
            ratios[i] = avg_ratio;
        }
    }
    else
    {
        // 使用提供的功率分配比例
        for (int i = 0; i < motor_num; i++) 
        {
            ratios[i] = power_ratios[i];
        }
    }

    // 计算总预测功率（此处使用 motor_data 中的 power_predict 字段）
    float total_power = 0.0f;
    for (int i = 0; i < motor_num; i++) 
    {
        total_power += motor_data[i].power_predict;
    }

    // 动态调整滤波系数 alpha，用于平滑电流变化
    float alpha = 1.0f; // 默认无滤波
    const float POWER_THRESHOLD = power_limit * 1.1f; // 功率阈值，用于判断是否需要强力限制
    if (total_power > POWER_THRESHOLD) 
    {
        alpha = 0.8f; // 功率远超限制，快速响应
    } 
    else if (total_power > power_limit) 
    {
        alpha = 0.10f; // 功率略超限制，平滑过渡
    }

    // 如果总功率超过限制，则进行电流限制
    if (total_power > power_limit) 
    {
        for (int i = 0; i < motor_num; i++) 
        {
            // 计算每个电机的允许最大功率
            float motor_power_limit = power_limit * ratios[i];
            
            // 计算该电机的限制后电流
            float restricted_torque = _motor_power_restrict_torque(
                i,                               
                motor_data[i].torque_cmd,      
                motor_data[i].gyro,            
                motor_power_limit
            );

            // 应用一阶滤波，更新限制后电流
            motor_data[i].restricted_torque = 
                alpha * restricted_torque + 
                (1 - alpha) * motor_data[i].last_torque;
        }
    }
    else 
    {
        // 功率在限制范围内，不进行限制，直接使用原始指令
        for (int i = 0; i < motor_num; i++) 
        {
            motor_data[i].restricted_torque = motor_data[i].torque_cmd;
        }
    }

    // 更新 last_torque 为下一次滤波做准备
    for(int i = 0; i < motor_num; i++)
    {
        motor_data[i].last_torque = motor_data[i].restricted_torque;
    }
}

/**
 * @brief 批量计算多个电机的限制后电流（公共接口）
 *
 * 这是一个公共接口，它调用内部的实现函数，并将 `power_ratios` 设置为 nullptr，
 * 从而实现功率的平均分配。
 *
 * @param motor_data 指向 motor_data_t 结构体数组的指针。
 * @param motor_num 电机的数量。
 * @param power_limit 总的可用功率限制。
 */
void power_control_drv_t::calculate_restricted_torques(
    motor_data_t* motor_data,
    int motor_num,
    float power_limit
) const
{ 
    calculate_restricted_torques(motor_data, motor_num, power_limit, nullptr);
}

/**
 * @brief 私有构造函数
 *
 * 根据指定的电机数量初始化 `_motor_coefficients` 向量。
 *
 * @param motor_num 电机的数量。
 */
power_control_drv_t::power_control_drv_t(int motor_index)
{
    _motor_coefficients.resize(motor_index);
}

}