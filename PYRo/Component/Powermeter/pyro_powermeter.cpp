/**
 * @file pyro_powermeter.cpp
 * @brief 功率计设备驱动实现文件
 * 该文件包含了powermeter_drv_t类所有成员函数的具体实现。
 * 实现了驱动的初始化、数据读取和CAN消息处理等核心逻辑。
 * @namespace: pyro
 * 
 * @author Pason
 * @date 2025-11-26
 * @version 1.0
*/
/**
 * @file pyro_powermeter.cpp
 * @brief Power Meter Device Driver Implementation File
 *
 * This file contains the specific implementation of all member functions of the powermeter_drv_t class.
 * Implements core logic such as driver initialization, data reading, and CAN message processing.
 */
#include "pyro_powermeter.h"
#include <cstring>

namespace pyro {

/**
 * @brief powermeter_drv_t类构造函数实现
 *
 * 初始化成员变量，设置CAN ID和总线通道，将CAN驱动和消息缓冲区指针置为nullptr，
 * 初始化数据更新标志位为true，并将最新数据结构体清零。
 *
 * @param can_id 功率计设备的CAN节点ID
 * @param which 使用的CAN总线通道 (can_hub_t::which_can枚举值)
 */
/**
 * @brief Implementation of powermeter_drv_t Constructor
 *
 * Initializes member variables, sets CAN ID and bus channel, sets CAN driver and message buffer pointers to nullptr,
 * initializes the data update flag to true, and clears the latest data structure.
 *
 * @param can_id CAN node ID of the power meter device
 * @param which CAN bus channel to use (enumeration value of can_hub_t::which_can)
 */
 powermeter_drv_t::powermeter_drv_t(uint32_t can_id, can_hub_t::which_can which)
    : _can_id(can_id),
      _which(which),
      _can_drv(nullptr),
      _msg_buffer(nullptr),
      _data_updated(true) 
{
    _latest_data = {0.0f, 0.0f, 0.0f};
}

/**
 * @brief init()成员函数实现
 *
 * 1. 获取CAN集线器实例
 * 2. 根据指定通道获取CAN驱动实例
 * 3. 分配CAN消息缓冲区内存
 * 4. 注册CAN接收消息缓冲区
 * 5. 若注册失败，释放已分配的缓冲区内存并返回错误状态
 *
 * @return status_t 初始化结果
 * @retval PYRO_OK 初始化成功
 * @retval PYRO_ERROR CAN集线器获取失败或CAN驱动获取失败
 * @retval PYRO_NO_MEMORY 消息缓冲区分配失败
 */
/**
 * @brief Implementation of init() Member Function
 *
 * 1. Get the CAN hub instance
 * 2. Get the CAN driver instance based on the specified channel
 * 3. Allocate memory for the CAN message buffer
 * 4. Register the CAN receive message buffer
 * 5. If registration fails, release the allocated buffer memory and return the error status
 *
 * @return status_t Initialization result
 * @retval PYRO_OK Initialization successful
 * @retval PYRO_ERROR Failed to get CAN hub or CAN driver instance
 * @retval PYRO_NO_MEMORY Failed to allocate message buffer
 */
 status_t powermeter_drv_t::init() 
{
    can_hub_t* can_hub = can_hub_t::get_instance();
    if (can_hub  == nullptr) {
        return PYRO_ERROR;
    }

    _can_drv = can_hub->hub_get_can_obj(_which);
    if (_can_drv == nullptr) {
        return PYRO_ERROR;
    }

    _msg_buffer = new can_msg_buffer_t(_can_id);
    if (_msg_buffer == nullptr) 
    {
        return PYRO_NO_MEMORY;
    }

    status_t reg_status = _can_drv->register_rx_msg(_msg_buffer);
    if (reg_status != PYRO_OK)
    {
        delete _msg_buffer;
        _msg_buffer = nullptr;
        return reg_status;
    }

    return PYRO_OK;
}

/**
 * @brief get_data()成员函数实现
 *
 * 1. 检查是否有新数据或消息缓冲区是否有效
 * 2. 从消息缓冲区获取原始CAN数据
 * 3. 若获取成功，调用process_can_data()解析数据
 * 4. 将解析后的最新数据存入输出参数
 * 5. 标记消息为已读取并返回成功
 *
 * @param data 用于存储功率计数据的结构体引用
 * @return bool 数据获取是否成功
 * @retval true 成功获取并解析数据
 * @retval false 无新数据或消息缓冲区为空
 */
/**
 * @brief Implementation of get_data() Member Function
 *
 * 1. Check if there is new data or if the message buffer is valid
 * 2. Get raw CAN data from the message buffer
 * 3. If acquisition is successful, call process_can_data() to parse the data
 * 4. Store the parsed latest data in the output parameter
 * 5. Mark the message as read and return success
 *
 * @param data Reference to the structure for storing power meter data
 * @return bool Whether data acquisition is successful
 * @retval true Data successfully acquired and parsed
 * @retval false No new data or message buffer is empty
 */
bool powermeter_drv_t::get_data(powermeter_data& data) {
    if (!_data_updated || _msg_buffer == nullptr) 
    {
        return false;
    }

    std::array<uint8_t, 8> raw_data;
    if (_msg_buffer->get_data(raw_data)) 
    {
        _data_updated = false;
        _process_can_data(raw_data);
        data = _latest_data;
        _msg_buffer->mark_read();
        return true;
    }

    return false;
}

/**
 * @brief process_can_data()私有成员函数实现
 *
 * 1. 从原始CAN数据中解析电流值 (data[0]和data[1]组成16位整数，除以100.0f转换为A)
 * 2. 从原始CAN数据中解析电压值 (data[2]和data[3]组成16位整数，除以100.0f转换为V)
 * 3. 计算功率值 (电流 * 电压，单位为W)
 * 4. 设置数据更新标志位为true
 *
 * @param data 包含CAN原始数据的std::array数组 (长度为8)
 */
/**
 * @brief Implementation of process_can_data() Private Member Function
 *
 * 1. Parse the current value from raw CAN data (data[0] and data[1] form a 16-bit integer, converted to A by dividing by 100.0f)
 * 2. Parse the voltage value from raw CAN data (data[2] and data[3] form a 16-bit integer, converted to V by dividing by 100.0f)
 * 3. Calculate the power value (Current * Voltage, Unit: W)
 * 4. Set the data update flag to true
 *
 * @param data std::array containing raw CAN data (length: 8)
 */
 void powermeter_drv_t::_process_can_data(const std::array<uint8_t, 8>& data) 
{
    int16_t raw_current = (int16_t)((data[1] << 8) | data[0]);
    _latest_data.current = static_cast<float>(raw_current) / 100.0f;

    int16_t raw_voltage = (int16_t)((data[3] << 8) | data[2]);
    _latest_data.voltage = static_cast<float>(raw_voltage) / 100.0f;

    _latest_data.power = _latest_data.current * _latest_data.voltage;

    _data_updated = true;
}

}
