/**
 * @file: pyro_power_control_drv.h
 * @brief: 功率控制驱动头文件
 * 
 * 该文件包含功率控制驱动类 power_control_drv_t 的声明，
 * 提供多电机功率输出管理与限制功能。定义了电机系数结构体、车轮数据结构体，
 * 以及功率预测、电流限制相关的函数原型。类采用单例模式设计，
 * 确保应用中全局唯一实例。
 * @namespace: pyro
 * 
 * @authors: Butterbutterfly0（架构）、Pason（实现）
 * @date: 2025-11-26
 * @version: 1.0
 */
#ifndef __PYRO__POWER_CONTROL_DRV_H__
#define __PYRO__POWER_CONTROL_DRV_H__

#include <vector>

namespace pyro
{

/**
 * @brief 功率控制驱动类
 *
 * 该类用于管理和限制多个电机的功率输出。它通过预测电机功率消耗，并根据给定的功率限制
 * 来调整电机的电流指令，以防止功率过载。支持多电机配置，并提供了线程安全的单例模式。
 */
class power_control_drv_t
{
public:
    /**
     * @brief 电机系数结构体
     *
     * 存储用于电机功率预测的多项式系数。这些系数通常通过系统辨识或校准获得。
     * 功率预测公式为: P = k1*tau*gyro + k2*|gyro| + k3*tau^2 + k4
     */
    struct motor_coefficient_t
    {
        float k1; ///< 电流-角速度交叉项系数
        float k2; ///< 角速度绝对值项系数
        float k3; ///< 电流平方项系数
        float k4; ///< 常数项系数
    };

    /**
     * @brief 电机数据结构体
     *
     * 作为 `calculate_restricted_torques` 函数的输入和输出参数，
     * 包含了单个电机的电流指令、状态和限制后结果。
     */
    struct motor_data_t
    {
        float torque_cmd;        ///< 输入: 当前期望电流指令
        float last_torque;       ///< 输入/输出: 上一次的限制后电流（用于滤波）
        float gyro;              ///< 输入: 电机的角速度 (rad/s)
        float power_predict;     ///< 输入: 外部预测的功率值
        float restricted_torque; ///< 输出: 经过功率限制后的最终电流指令
    };
    
    /**
     * @brief 删除拷贝构造函数
     * 防止通过拷贝创建新实例，确保单例模式的唯一性。
     */
    power_control_drv_t(const power_control_drv_t&) = delete;
    
    /**
     * @brief 删除赋值运算符
     * 防止实例间的赋值，确保单例模式的唯一性。
     */
    power_control_drv_t& operator=(const power_control_drv_t&) = delete;

    /**
     * @brief 获取单例实例
     *
     * 这是获取类实例的唯一方法。如果实例不存在，将使用指定的电机数量创建一个新实例。
     * 后续调用将返回同一个实例，忽略电机数量参数。
     *
     * @param motor_num 电机的数量。
     * @return power_control_drv_t& 返回单例实例的引用。
     */
    static power_control_drv_t& get_instance(int motor_num)
    {
        static power_control_drv_t instance(motor_num);
        return instance;
    }

    /**
     * @brief 获取单例实例（默认参数）
     *
     * 重载版本，使用默认电机数量（0）获取实例。主要用于首次创建实例后，
     * 或不确定电机数量时获取实例。
     *
     * @return power_control_drv_t& 返回单例实例的引用。
     */
    static power_control_drv_t& get_instance()
    {
        return get_instance(0);
    }

    /**
     * @brief 设置电机的功率系数
     *
     * 为指定索引的电机设置功率预测模型的系数。
     *
     * @param motor_index 电机的索引 (从 1 开始？请参见cpp实现)。
     * @param coefficient 包含四个系数的结构体。
     */
    void set_motor_coefficient(int motor_index, const motor_coefficient_t& coefficient);    

    /**
     * @brief 预测电机功率
     *
     * 使用预存的电机系数和当前的电流、角速度计算预测的电机功率消耗。
     *
     * @param motor_index 电机的索引。
     * @param tau 当前的电流指令（通过电流值代替 A）。
     * @param gyro 当前的角速度 (rad/s)。
     * @return float 预测的功率值 (W)。如果索引无效，返回 0.0f。
     */
    float motor_power_predict(int motor_index, float tau, float gyro) const;

    /**
     * @brief 批量计算多个电机的限制后电流
     *
     * 这是核心功能函数。它会遍历所有电机数据，根据总功率限制 `power_limit`
     * 和可选的功率分配比例 `power_ratios`，为每个电机计算并设置限制后的电流。
     *
     * @param motor_data 指向 motor_data_t 结构体数组的指针。
     * @param motor_num 电机的数量。
     * @param power_limit 总的可用功率限制。
     */
    void calculate_restricted_torques(
        motor_data_t* motor_data,
        int motor_num,
        float power_limit
    ) const;

    /**
     * @brief 批量计算多个电机的限制后电流（带功率分配）
     *
     * 这是 `calculate_restricted_torques` 的重载版本。它允许你通过 `power_ratios`
     * 参数为不同的电机分配不同比例的总功率。
     *
     * @param motor_data 指向 motor_data_t 结构体数组的指针。
     * @param motor_num 电机的数量。
     * @param power_limit 总的可用功率限制。
     * @param power_ratios 指向浮点数数组的指针，每个元素代表对应电机的功率分配比例。
     */
    void calculate_restricted_torques(
        motor_data_t* motor_data,
        int motor_num,
        float power_limit,
        const float* power_ratios
    ) const;

private:
    /**
     * @brief 私有构造函数
     *
     * 构造函数被声明为私有，以强制通过 `get_instance` 方法来创建和获取实例。
     *
     * @param motor_num 电机的数量，用于初始化内部数据结构。
     */
    explicit power_control_drv_t(int motor_num);
    
    /**
     * @brief 私有析构函数
     */
    ~power_control_drv_t()
    {
    }

    /**
     * @brief 计算限制后电流
     *
     * 给定一个期望电流和功率限制，通过解二次方程计算出最大允许的电流，
     * 以确保电机功率不超过 `restricted_power`。
     *
     * @param motor_index 电机的索引。
     * @param origin_torque 原始的、未受限制的电流指令。
     * @param gyro 当前的角速度。
     * @param restricted_power 允许的最大功率。
     * @return float 计算出的限制后电流。
     */
    float _motor_power_restrict_torque(int motor_index, float origin_torque, float gyro, float restricted_power) const;

    std::vector<motor_coefficient_t> _motor_coefficients; ///< 存储每个电机的功率系数
};

}

#endif