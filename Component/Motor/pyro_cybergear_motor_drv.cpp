/**
 * @version 1.1.0
 */

#include "pyro_cybergear_motor_drv.h"
#include <cstring>

namespace pyro
{

// ==========================================
// cybergear_can_id_t Implementation
// ==========================================

cybergear_can_id_t::cybergear_can_id_t(uint8_t comm_type, uint16_t data2, uint8_t motor_id)
    : _comm_type(comm_type), _data2(data2), _motor_id(motor_id)
{
}

cybergear_can_id_t::cybergear_can_id_t(uint32_t raw_extended_id)
{
    decode(raw_extended_id);
}

uint32_t cybergear_can_id_t::encode() const
{
    uint32_t id = 0;
    id |= (static_cast<uint32_t>(_comm_type & 0x1F) << COMM_TYPE_SHIFT);
    id |= (static_cast<uint32_t>(_data2) << DATA2_SHIFT) & DATA2_MASK;
    id |= static_cast<uint32_t>(_motor_id) & MOTOR_ID_MASK;
    return id;
}

void cybergear_can_id_t::decode(uint32_t raw_extended_id)
{
    _comm_type = static_cast<uint8_t>((raw_extended_id & COMM_TYPE_MASK) >> COMM_TYPE_SHIFT);
    _data2 = static_cast<uint16_t>((raw_extended_id & DATA2_MASK) >> DATA2_SHIFT);
    _motor_id = static_cast<uint8_t>(raw_extended_id & MOTOR_ID_MASK);
}

// ==========================================
// cybergear_motor_drv_t Implementation
// ==========================================

cybergear_motor_drv_t::cybergear_motor_drv_t(uint8_t motor_id, uint8_t master_id,
                                             bsp_can::which_can which)
    : motor_base_t(which), _motor_id(motor_id), _master_id(master_id),
      _current_run_mode(MODE_CURRENT),  // 默认电流模式（按声明顺序）
      _min_position(-12.566f), _max_position(12.566f),
      _min_velocity(-30.0f), _max_velocity(30.0f),
      _min_torque(-12.0f), _max_torque(12.0f)
{
    // 初始化故障状态和模式
    _fault_status = {false, false, false, false, false, false};
    _motor_mode = MODE_RESET;
    _detailed_fault = {false, false, false, false, false, false, false, false, false, false};

    // 注册设备ID应答帧接收缓冲区
    // 应答帧ID: bit[28:24]=0, bit[23:16]=motor_id, bit[7:0]=0xFE
    cybergear_can_id_t device_id_rx(0, motor_id << 8, 0xFE);
    uint32_t device_id_rx_id = device_id_rx.encode();

    _device_id_msg = new can_msg_buffer_t(device_id_rx_id, can_msg_buffer_t::EXTENDED_ID);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_device_id_msg);
    }

    // 注册运控模式反馈帧接收缓冲区
    // 根据实际抓包发现：
    // ID格式：bit[28:24]=comm_type, bit[23:16]=状态, bit[15:8]=motor_id(127), bit[7:0]=master_id(0)
    // 实际ID: 0x2807F00 = comm_type=2, data2=0x807F (bit[15:8]=0x7F=127), motor_id=0
    // 所以motor_id(127)在data2的低8位，master_id(0)在bit[7:0]

    uint16_t data2 = motor_id;  // bit[15:8]会是motor_id，bit[23:16]会被掩码忽略
    cybergear_can_id_t motion_feedback_rx(2, data2, master_id);
    uint32_t motion_feedback_rx_id = motion_feedback_rx.encode();

    // 掩码：0x1F00FFFF (忽略bit[23:16]的状态位)
    uint32_t mask = 0x1F00FFFF;

    _motion_feedback_msg = new can_msg_buffer_t(motion_feedback_rx_id, mask, can_msg_buffer_t::EXTENDED_ID);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_motion_feedback_msg);
    }

    // 注册参数读写应答帧接收缓冲区
    // 应答帧ID: bit[28:24]=17, bit[15:8]=motor_id, bit[7:0]=master_id
    cybergear_can_id_t param_response_rx(17, motor_id, master_id);
    uint32_t param_response_rx_id = param_response_rx.encode();

    _param_response_msg = new can_msg_buffer_t(param_response_rx_id, can_msg_buffer_t::EXTENDED_ID);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_param_response_msg);
    }

    // 注册故障反馈帧接收缓冲区
    // 故障反馈帧ID: bit[28:24]=21, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t fault_feedback_rx(21, master_id, motor_id);
    uint32_t fault_feedback_rx_id = fault_feedback_rx.encode();

    _fault_feedback_msg = new can_msg_buffer_t(fault_feedback_rx_id, can_msg_buffer_t::EXTENDED_ID);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_fault_feedback_msg);
    }
}

cybergear_motor_drv_t::~cybergear_motor_drv_t()
{
    if (_device_id_msg)
    {
        delete _device_id_msg;
        _device_id_msg = nullptr;
    }

    if (_motion_feedback_msg)
    {
        delete _motion_feedback_msg;
        _motion_feedback_msg = nullptr;
    }

    if (_param_response_msg)
    {
        delete _param_response_msg;
        _param_response_msg = nullptr;
    }

    if (_fault_feedback_msg)
    {
        delete _fault_feedback_msg;
        _fault_feedback_msg = nullptr;
    }
}

status_t cybergear_motor_drv_t::enable()
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 1. 先设置为电流模式
    status_t ret = set_run_mode(MODE_CURRENT);
    if (ret != PYRO_OK)
    {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // 等待模式切换完成

    // 2. 发送使能命令
    // 构建29位ID: bit[28:24]=3, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(3, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 数据区为空
    uint8_t data[8] = {0};
    ret = _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);

    if (ret == PYRO_OK)
    {
        _enable = true;
    }

    return ret;
}

status_t cybergear_motor_drv_t::disable()
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=4, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(4, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 正常停止：数据区清0
    uint8_t data[8] = {0};
    status_t ret = _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);

    if (ret == PYRO_OK)
    {
        _enable = false;
    }

    return ret;
}

status_t cybergear_motor_drv_t::clear_fault()
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=4, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(4, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 清故障：Byte[0]=1
    uint8_t data[8] = {0};
    data[0] = 1;

    return _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
}

status_t cybergear_motor_drv_t::set_mechanical_zero()
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=6, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(6, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 设置零位：Byte[0]=1
    uint8_t data[8] = {0};
    data[0] = 1;

    return _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
}

status_t cybergear_motor_drv_t::set_can_id(uint8_t new_can_id)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=7, bit[23:16]=new_can_id, bit[15:8]=master_id, bit[7:0]=motor_id
    uint16_t data2 = (static_cast<uint16_t>(new_can_id) << 8) | _master_id;
    cybergear_can_id_t tx_id(7, data2, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 数据区为空
    uint8_t data[8] = {0};
    status_t ret = _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);

    // 如果设置成功，更新本地保存的电机ID
    if (ret == PYRO_OK)
    {
        // 注意：设置成功后电机ID已改变，后续通信需要使用新ID
        // 但这里不直接修改_motor_id，因为可能需要等待应答确认
        // 用户需要在确认后手动更新或重新创建驱动对象
    }

    return ret;
}

status_t cybergear_motor_drv_t::read_param(param_index_t index, float &value)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=17, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(17, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 构建数据区
    uint8_t data[8] = {0};
    data[0] = (index >> 8) & 0xFF;  // index高字节
    data[1] = index & 0xFF;         // index低字节
    // Byte2~7保持为0

    status_t ret = _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
    if (ret != PYRO_OK)
    {
        return ret;
    }

    // 等待应答 (简单超时等待)
    const uint32_t timeout_ms = 100;
    uint32_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms))
    {
        if (_param_response_msg && _param_response_msg->is_fresh())
        {
            std::array<uint8_t, 8> rx_data;
            if (_param_response_msg->get_data(rx_data))
            {
                // 验证返回的index是否匹配
                uint16_t response_index = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
                if (response_index == index)
                {
                    // 提取参数值 (Byte4~7, 小端序)
                    // 1字节数据在Byte4，多字节数据从Byte4开始
                    uint32_t raw_value = rx_data[4] | (rx_data[5] << 8) |
                                        (rx_data[6] << 16) | (rx_data[7] << 24);
                    memcpy(&value, &raw_value, sizeof(float));

                    _param_response_msg->mark_read();
                    return PYRO_OK;
                }
            }
        }
        vTaskDelay(1);
    }

    return PYRO_TIMEOUT;
}

status_t cybergear_motor_drv_t::read_param(param_index_t index, uint8_t &value)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=17, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(17, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 构建数据区
    uint8_t data[8] = {0};
    data[0] = (index >> 8) & 0xFF;
    data[1] = index & 0xFF;

    status_t ret = _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
    if (ret != PYRO_OK)
    {
        return ret;
    }

    // 等待应答
    const uint32_t timeout_ms = 100;
    uint32_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms))
    {
        if (_param_response_msg && _param_response_msg->is_fresh())
        {
            std::array<uint8_t, 8> rx_data;
            if (_param_response_msg->get_data(rx_data))
            {
                uint16_t response_index = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
                if (response_index == index)
                {
                    // uint8数据在Byte4
                    value = rx_data[4];

                    _param_response_msg->mark_read();
                    return PYRO_OK;
                }
            }
        }
        vTaskDelay(1);
    }

    return PYRO_TIMEOUT;
}

status_t cybergear_motor_drv_t::read_param(param_index_t index, int16_t &value)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=17, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(17, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 构建数据区
    uint8_t data[8] = {0};
    data[0] = (index >> 8) & 0xFF;
    data[1] = index & 0xFF;

    status_t ret = _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
    if (ret != PYRO_OK)
    {
        return ret;
    }

    // 等待应答
    const uint32_t timeout_ms = 100;
    uint32_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms))
    {
        if (_param_response_msg && _param_response_msg->is_fresh())
        {
            std::array<uint8_t, 8> rx_data;
            if (_param_response_msg->get_data(rx_data))
            {
                uint16_t response_index = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
                if (response_index == index)
                {
                    // int16数据在Byte4~5（小端序）
                    value = static_cast<int16_t>(rx_data[4] | (rx_data[5] << 8));

                    _param_response_msg->mark_read();
                    return PYRO_OK;
                }
            }
        }
        vTaskDelay(1);
    }

    return PYRO_TIMEOUT;
}

status_t cybergear_motor_drv_t::write_param(param_index_t index, float value)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=18, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(18, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 构建数据区
    uint8_t data[8] = {0};
    data[0] = (index >> 8) & 0xFF;  // index高字节
    data[1] = index & 0xFF;         // index低字节
    // Byte2~3保持为0

    // Byte4~7: 参数数据（小端序）
    uint32_t raw_value;
    memcpy(&raw_value, &value, sizeof(float));
    data[4] = raw_value & 0xFF;
    data[5] = (raw_value >> 8) & 0xFF;
    data[6] = (raw_value >> 16) & 0xFF;
    data[7] = (raw_value >> 24) & 0xFF;

    return _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
}

status_t cybergear_motor_drv_t::write_param(param_index_t index, uint8_t value)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=18, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(18, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 构建数据区
    uint8_t data[8] = {0};
    data[0] = (index >> 8) & 0xFF;  // index高字节
    data[1] = index & 0xFF;         // index低字节
    // Byte2~3保持为0
    data[4] = value;  // 1字节数据在Byte4
    // Byte5~7保持为0

    return _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
}

status_t cybergear_motor_drv_t::write_param(param_index_t index, int16_t value)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建29位ID: bit[28:24]=18, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(18, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 构建数据区
    uint8_t data[8] = {0};
    data[0] = (index >> 8) & 0xFF;  // index高字节
    data[1] = index & 0xFF;         // index低字节
    // Byte2~3保持为0

    // int16数据在Byte4~5（小端序）
    data[4] = value & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    // Byte6~7保持为0

    return _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
}

status_t cybergear_motor_drv_t::update_feedback()
{
    if (!_motion_feedback_msg || !_motion_feedback_msg->is_fresh())
    {
        return PYRO_ERROR;
    }

    std::array<uint8_t, 8> data;
    uint32_t received_id;
    if (!_motion_feedback_msg->get_data(data, received_id))
    {
        return PYRO_ERROR;
    }

    // 解析数据区
    // Byte0~1: 当前角度 [0~65535] 对应配置的范围
    uint16_t angle_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    _current_position = (angle_raw / 65535.0f) * (_max_position - _min_position) + _min_position;

    // Byte2~3: 当前角速度 [0~65535] 对应配置的范围
    uint16_t velocity_raw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    _current_rotate = (velocity_raw / 65535.0f) * (_max_velocity - _min_velocity) + _min_velocity;

    // Byte4~5: 当前力矩 [0~65535] 对应配置的范围
    uint16_t torque_raw = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    _current_torque = (torque_raw / 65535.0f) * (_max_torque - _min_torque) + _min_torque;

    // Byte6~7: 当前温度 (摄氏度 * 10)
    uint16_t temp_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];
    _temperature = static_cast<int8_t>(temp_raw / 10);

    // 从29位ID中解析状态信息
    cybergear_can_id_t rx_id(received_id);
    uint16_t status_data = rx_id.get_data2();

    // 提取故障状态 (bit16~21)
    _fault_status.under_voltage = (status_data & (1 << 0)) != 0;  // bit16
    _fault_status.over_current = (status_data & (1 << 1)) != 0;   // bit17
    _fault_status.over_temp = (status_data & (1 << 2)) != 0;      // bit18
    _fault_status.magnetic_error = (status_data & (1 << 3)) != 0; // bit19
    _fault_status.hall_error = (status_data & (1 << 4)) != 0;     // bit20
    _fault_status.uncalibrated = (status_data & (1 << 5)) != 0;   // bit21

    // 提取电机模式 (bit22~23)
    _motor_mode = static_cast<motor_mode_t>((status_data >> 6) & 0x03);

    _motion_feedback_msg->mark_read();
    _online = true;
    _last_update_time = xTaskGetTickCount();

    // 检查是否有故障反馈帧
    if (_fault_feedback_msg && _fault_feedback_msg->is_fresh())
    {
        std::array<uint8_t, 8> fault_data;
        if (_fault_feedback_msg->get_data(fault_data))
        {
            // 解析fault值 (Byte0~3)
            uint32_t fault = fault_data[0] | (fault_data[1] << 8) |
                            (fault_data[2] << 16) | (fault_data[3] << 24);

            _detailed_fault.motor_over_temp = (fault & (1 << 0)) != 0;
            _detailed_fault.driver_chip_fault = (fault & (1 << 1)) != 0;
            _detailed_fault.under_voltage = (fault & (1 << 2)) != 0;
            _detailed_fault.over_voltage = (fault & (1 << 3)) != 0;
            _detailed_fault.phase_b_over_current = (fault & (1 << 4)) != 0;
            _detailed_fault.phase_c_over_current = (fault & (1 << 5)) != 0;
            _detailed_fault.encoder_uncalibrated = (fault & (1 << 7)) != 0;
            _detailed_fault.overload = (fault & 0xFF00) != 0;  // bit15~8
            _detailed_fault.phase_a_over_current = (fault & (1 << 16)) != 0;

            // 解析warning值 (Byte4~7)
            uint32_t warning = fault_data[4] | (fault_data[5] << 8) |
                              (fault_data[6] << 16) | (fault_data[7] << 24);

            _detailed_fault.motor_temp_warning = (warning & (1 << 0)) != 0;

            _fault_feedback_msg->mark_read();
        }
    }

    return PYRO_OK;
}

status_t cybergear_motor_drv_t::send_torque(float torque)
{
    // 直接发送力矩指令（电流模式）
    // 这是基类接口，不依赖当前模式，直接控制力矩
    return send_motion_control(0.0f, 0.0f, 0.0f, 0.0f, torque);
}

status_t cybergear_motor_drv_t::get_device_id(uint64_t &mcu_id)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 构建发送帧ID: bit[28:24]=0, bit[15:8]=master_id, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(0, _master_id, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 发送全0数据
    uint8_t data[8] = {0};
    status_t ret = _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
    if (ret != PYRO_OK)
    {
        return ret;
    }

    // 等待应答 (简单超时等待)
    const uint32_t timeout_ms = 100;
    uint32_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms))
    {
        if (_device_id_msg && _device_id_msg->is_fresh())
        {
            std::array<uint8_t, 8> rx_data;
            if (_device_id_msg->get_data(rx_data))
            {
                // 提取64位MCU唯一标识符
                mcu_id = 0;
                for (int i = 0; i < 8; i++)
                {
                    mcu_id |= (static_cast<uint64_t>(rx_data[i]) << (i * 8));
                }

                _device_id_msg->mark_read();
                return PYRO_OK;
            }
        }
        vTaskDelay(1);
    }

    return PYRO_TIMEOUT;
}

// ==========================================
// 高层模式控制接口
// ==========================================

status_t cybergear_motor_drv_t::set_run_mode(run_mode_t mode)
{
    // 记录当前运行模式
    _current_run_mode = mode;
    return write_param(PARAM_RUN_MODE, static_cast<uint8_t>(mode));
}

status_t cybergear_motor_drv_t::set_current_ref(float iq_ref)
{
    return write_param(PARAM_IQ_REF, iq_ref);
}

status_t cybergear_motor_drv_t::set_speed_ref(float spd_ref)
{
    return write_param(PARAM_SPD_REF, spd_ref);
}

status_t cybergear_motor_drv_t::set_position_ref(float loc_ref)
{
    return write_param(PARAM_LOC_REF, loc_ref);
}

// ==========================================
// 模式专用控制接口（推荐使用）
// ==========================================

status_t cybergear_motor_drv_t::send_control(float value)
{
    // 根据当前运行模式自动分配
    switch (_current_run_mode)
{
    case MODE_CURRENT:
        // 电流模式：value = 力矩 (Nm)
        return send_motion_control(0.0f, 0.0f, 0.0f, 0.0f, value);

    case MODE_SPEED:
        // 速度模式：value = 速度 (rad/s)
        return send_motion_control(0.0f, value, 0.0f, 0.0f, 0.0f);

    case MODE_POSITION:
        // 位置模式：value = 位置 (rad)，使用电机内部PID
        return send_motion_control(value, 0.0f, 0.0f, 0.0f, 0.0f);

    default:
        return PYRO_ERROR;  // 未知模式
    }
}

status_t cybergear_motor_drv_t::send_control(float value, float kp, float kd)
{
    // 位置模式（带自定义PID参数）
    // 检查当前是否为位置模式
    if (_current_run_mode != MODE_POSITION)
    {
        return PYRO_ERROR;  // 只有位置模式支持Kp/Kd参数
    }

    return send_motion_control(value, 0.0f, kp, kd, 0.0f);
}

void cybergear_motor_drv_t::set_position_range(float min, float max)
{
    _min_position = min;
    _max_position = max;
}

void cybergear_motor_drv_t::set_rotate_range(float min, float max)
{
    _min_velocity = min;
    _max_velocity = max;
}

void cybergear_motor_drv_t::set_torque_range(float min, float max)
{
    _min_torque = min;
    _max_torque = max;
}

status_t cybergear_motor_drv_t::send_motion_control(float target_angle, float target_velocity,
                                                    float kp, float kd, float torque)
{
    if (!_can_drv)
    {
        return PYRO_ERROR;
    }

    // 限幅和映射：力矩 [-12, 12] -> [0, 65535]
    if (torque < -12.0f) torque = -12.0f;
    if (torque > 12.0f) torque = 12.0f;
    uint16_t torque_raw = static_cast<uint16_t>((torque + 12.0f) * 65535.0f / 24.0f);

    // 限幅和映射：角度 [-4π, 4π] -> [0, 65535]
    const float pi = 3.14159265359f;
    if (target_angle < -4.0f * pi) target_angle = -4.0f * pi;
    if (target_angle > 4.0f * pi) target_angle = 4.0f * pi;
    uint16_t angle_raw = static_cast<uint16_t>((target_angle + 4.0f * pi) * 65535.0f / (8.0f * pi));

    // 限幅和映射：角速度 [-30, 30] -> [0, 65535]
    if (target_velocity < -30.0f) target_velocity = -30.0f;
    if (target_velocity > 30.0f) target_velocity = 30.0f;
    uint16_t velocity_raw = static_cast<uint16_t>((target_velocity + 30.0f) * 65535.0f / 60.0f);

    // 限幅和映射：Kp [0, 500] -> [0, 65535]
    if (kp < 0.0f) kp = 0.0f;
    if (kp > 500.0f) kp = 500.0f;
    uint16_t kp_raw = static_cast<uint16_t>(kp * 65535.0f / 500.0f);

    // 限幅和映射：Kd [0, 5] -> [0, 65535]
    if (kd < 0.0f) kd = 0.0f;
    if (kd > 5.0f) kd = 5.0f;
    uint16_t kd_raw = static_cast<uint16_t>(kd * 65535.0f / 5.0f);

    // 构建29位ID: bit[28:24]=1, bit[23:8]=torque_raw, bit[7:0]=motor_id
    cybergear_can_id_t tx_id(1, torque_raw, _motor_id);
    uint32_t can_id = tx_id.encode();

    // 构建数据区
    uint8_t data[8];
    data[0] = (angle_raw >> 8) & 0xFF;
    data[1] = angle_raw & 0xFF;
    data[2] = (velocity_raw >> 8) & 0xFF;
    data[3] = velocity_raw & 0xFF;
    data[4] = (kp_raw >> 8) & 0xFF;
    data[5] = kp_raw & 0xFF;
    data[6] = (kd_raw >> 8) & 0xFF;
    data[7] = kd_raw & 0xFF;

    return _can_drv->send_msg(can_id, data, can_msg_buffer_t::EXTENDED_ID);
}

} // namespace pyro
