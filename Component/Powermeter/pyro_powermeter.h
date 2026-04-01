/**
 * @file pyro_powermeter.h
 * @brief 功率计设备驱动头文件
 * 该文件包含了powermeter_drv_t类的声明以及相关的数据结构定义。
 * 这个驱动类提供了与CAN总线功率计设备进行通信的接口，包括初始化、
 * 数据接收和解析等功能。
 * @namespace: pyro
 * @note: 依赖 pyro_can_drv.h
 * 
 * @author Pason
 * @date 2025-11-26
 * @version 1.0
 */
/**
 * @file pyro_powermeter.h
 * @brief Power Meter Device Driver Header File
 *
 * This file contains the declaration of the powermeter_drv_t class and related data structure definitions.
 * The driver class provides an interface for communicating with CAN bus-connected power meter devices,
 * including initialization, data reception, and parsing functionalities.
 */
#ifndef __POWERMETER_H__
#define __POWERMETER_H__

#include "pyro_can_drv.h"
#include <array>

namespace pyro {

/**
 * @brief 功率计数据结构体
 *
 * 用于存储从功率计设备读取的电流、电压和功率值。
 */
/**
 * @brief Power Meter Data Structure
 *
 * Stores current, voltage, and power values read from the power meter device.
 */
struct powermeter_data {
    float current;
    float voltage;
    float power;
};

/**
 * @brief 功率计设备驱动类
 *
 * 该类提供了与CAN总线功率计设备通信的接口，负责初始化、数据接收和解析。
 */
/**
 * @brief Power Meter Device Driver Class
 *
 * Provides an interface for communicating with CAN bus-connected power meter devices,
 * responsible for initialization, data reception, and parsing.
 */
class powermeter_drv_t {
public:
    /**
     * @brief 构造函数
     *
     * 初始化功率计驱动实例，设置CAN设备ID和总线通道。
     *
     * @param can_id 功率计设备的CAN ID
     * @param which 使用的CAN总线通道 (can_hub_t::which_can枚举值)
     */
    /**
     * @brief Constructor
     *
     * Initializes the power meter driver instance, sets the CAN device ID and bus channel.
     *
     * @param can_id CAN node ID of the power meter device
     * @param which CAN bus channel to use (enumeration value of can_hub_t::which_can)
     */
    powermeter_drv_t(uint32_t can_id, can_hub_t::which_can which);

    /**
     * @brief 析构函数
     */
    ~powermeter_drv_t()
    {
    }

    /**
     * @brief 初始化驱动
     *
     * 初始化CAN驱动，分配消息缓冲区并注册接收回调。
     *
     * @return status_t 初始化结果
     * @retval PYRO_OK 初始化成功
     * @retval PYRO_ERROR CAN集线器获取失败或CAN驱动获取失败
     * @retval PYRO_NO_MEMORY 消息缓冲区分配失败
     */
    /**
     * @brief Initialize the Driver
     *
     * Initializes the CAN driver, allocates the message buffer, and registers the receive callback.
     *
     * @return status_t Initialization result
     * @retval PYRO_OK Initialization successful
     * @retval PYRO_ERROR Failed to get CAN hub or CAN driver instance
     * @retval PYRO_NO_MEMORY Failed to allocate message buffer
     */
    status_t init();

    /**
     * @brief 获取功率计数据
     *
     * 从消息缓冲区读取最新数据，解析后存入powermeter_data结构体。
     *
     * @param data 用于存储功率计数据的结构体引用
     * @return bool 数据获取是否成功
     * @retval true 成功获取并解析数据
     * @retval false 无新数据或消息缓冲区为空
     */
    /**
     * @brief Get Power Meter Data
     *
     * Reads the latest data from the message buffer, parses it, and stores it in the powermeter_data structure.
     *
     * @param data Reference to the structure for storing power meter data
     * @return bool Whether data acquisition is successful
     * @retval true Data successfully acquired and parsed
     * @retval false No new data or message buffer is empty
     */
    bool get_data(powermeter_data& data);

private:
    uint32_t _can_id;                      ///< 功率计设备CAN ID
    can_hub_t::which_can _which;           ///< CAN总线通道
    can_drv_t* _can_drv;                   ///< CAN驱动实例指针
    can_msg_buffer_t* _msg_buffer;         ///< CAN消息缓冲区指针
    powermeter_data _latest_data;          ///< 最新解析的功率计数据
    volatile bool _data_updated;           ///< 数据更新标志位

    /**
     * @brief 处理CAN原始数据
     *
     * 将从CAN总线接收到的原始字节数据解析为电流、电压和功率值。
     *
     * @param data 包含CAN原始数据的std::array数组 (长度为8)
     */
    /**
     * @brief Process CAN Raw Data
     *
     * Parses the raw byte data received from the CAN bus into current, voltage, and power values.
     *
     * @param data std::array containing raw CAN data (length: 8)
     */
    void _process_can_data(const std::array<uint8_t, 8>& data);
};

}

#endif

