/**
 * @file pyro_referee.cpp
 * @brief RoboMaster Referee System Driver (Modern C++ Style Implementation)
 */

#include "pyro_referee.h"

#include "pyro_bsp_uart.h"
#include "pyro_crc.h"
#include "pyro_dwt_drv.h"

#include "pyro_core_dma_heap.h"
#include <cstring> // for memcpy, strlen

namespace pyro
{

// ==========================================================================
// Task Implementation
// ==========================================================================

status_t referee_drv_t::referee_task::init()
{
    // Task-specific initialization if needed
    return PYRO_OK;
}

void referee_drv_t::referee_task::run_loop()
{
    while (true)
    {
        if (_parent)
        {
            _parent->unpack_fifo_data();

            // 2s timeout check
            if (dwt_drv_t::get_timeline_ms() - _parent->_last_update_time >
                2000)
            {
                _parent->_is_online = false;
            }
        }
        vTaskDelay(1);
    }
}

// ==========================================================================
// Driver Singleton & Constructor
// ==========================================================================
#ifdef REFEREE_UART
referee_drv_t *referee_drv_t::get_instance()
{
    static referee_drv_t instance(&REFEREE_UART);
    return &instance;
}
#endif

referee_drv_t::referee_drv_t(uart_drv_t *uart_handle)
    : _uart(uart_handle), _task(nullptr), _data{}, _unpack_obj{}, _send_seq(0),
      _robot_id(0), _tx_buffer_idx(0), _is_online(false), _last_update_time(0)
{
    fifo_s_init(&_fifo, _fifo_buf, FIFO_BUF_LEN);

    // 【修改】申请多个 DMA 缓冲区用于 Ping-Pong 流水线
    for (auto &buf : _tx_buffers)
    {
        buf = static_cast<uint8_t *>(pvPortDmaMalloc(MAX_TX_FRAME_LEN));
        memset(buf, 0, MAX_TX_FRAME_LEN);
    }

    // 【修改】创建二值信号量，初始必须 Give 一次，代表 UART 外设一开始是空闲的
    _tx_cplt_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(_tx_cplt_sem);

    _task = new referee_task(this);
}

// ==========================================================================
// Init
// ==========================================================================

void referee_drv_t::init(const std::initializer_list<cmd_id> listening_ids)
{
    if (!_uart)
        return;

    _enabled_ids.reset();
    for (auto id : listening_ids)
    {
        const auto id_val = static_cast<uint16_t>(id);
        if (id_val < MAX_CMD_ID_COUNT)
        {
            _enabled_ids.set(id_val);
        }
    }

    _uart->add_rx_event_callback(
        [this](uint8_t *p, const uint16_t size,
               const BaseType_t task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));

    // 【新增】注册 TX DMA 发送完成中断回调
    _uart->set_tx_cplt_callback(
        [this](BaseType_t &woken)
        { xSemaphoreGiveFromISR(this->_tx_cplt_sem, &woken); });

    if (_task)
        _task->start();
}

void referee_drv_t::init()
{
    if (!_uart)
        return;

    _enabled_ids.set(); // Enable all

    _uart->add_rx_event_callback(
        [this](uint8_t *p, const uint16_t size,
               const BaseType_t task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));

    // 【新增】注册 TX DMA 发送完成中断回调
    _uart->set_tx_cplt_callback(
        [this](BaseType_t &woken)
        { xSemaphoreGiveFromISR(this->_tx_cplt_sem, &woken); });

    if (_task)
        _task->start();
}

// ==========================================================================
// Logic
// ==========================================================================

uint16_t referee_drv_t::get_client_id() const
{
    return _robot_id + 0x0100;
}

bool referee_drv_t::send_packet(cmd_id cmd_id_val, const void *data,
                                uint16_t const len)
{
    const auto cmd_val             = static_cast<uint16_t>(cmd_id_val);
    const uint16_t frame_total_len = HEADER_CMDID_LEN + len + CRC16_SIZE;

    if (frame_total_len > MAX_TX_FRAME_LEN || !_uart)
        return false;

    // 获取互斥锁，确保多线程调用组包发送互不干扰
    const scoped_mutex_t lock(_tx_mutex, pdMS_TO_TICKS(100));
    if (!lock.is_locked())
        return false;

    // 【关键优化：CPU与DMA并行】
    // 1. 获取一个尚未被 DMA 占用的后台缓冲区
    // 由于我们使用的是二值信号量拦截，能走到这里的 buffer
    // 必定是上一包的闲置内存
    uint8_t *current_tx_buf = _tx_buffers[_tx_buffer_idx];
    _tx_buffer_idx          = (_tx_buffer_idx + 1) % TX_BUFFER_NUM;

    // 2. 组装数据。此时 CPU 在干活，而前一包可能仍在通过 DMA 在后台高速发送中！
    auto *p_header        = reinterpret_cast<frame_header_t *>(current_tx_buf);
    p_header->sof         = HEADER_SOF;
    p_header->data_length = len;
    p_header->seq         = _send_seq++;
    p_header->crc8        = 0;

    append_crc8_check_sum(current_tx_buf, HEADER_SIZE);

    uint8_t *p_cmd_start = current_tx_buf + HEADER_SIZE;
    auto *p_cmd_id       = reinterpret_cast<uint16_t *>(p_cmd_start);
    *p_cmd_id            = cmd_val;

    if (len > 0 && data != nullptr)
    {
        memcpy(current_tx_buf + HEADER_CMDID_LEN, data, len);
    }

    append_crc16_check_sum(current_tx_buf, frame_total_len);

    // 3. 阻塞拿取二值信号量
    // 如果 UART 处于空闲状态，直接拿取通过；
    // 如果上一包 DMA 发送仍在进行，则任务休眠挂起，直到上一包发送完毕并进中断
    // Give 释放。
    if (xSemaphoreTake(_tx_cplt_sem, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return false;
    }

    // 4. 一旦上包发完（或本来就空闲），无缝衔接立即发起当前缓冲区的非阻塞 DMA
    // 发送
    if (_uart->write(current_tx_buf, frame_total_len) != PYRO_OK)
    {
        // 遇到极端的底层错误被拒发，归还信号量以免死锁
        xSemaphoreGive(_tx_cplt_sem);
        return false;
    }

    return true;
}

bool referee_drv_t::_send_interaction_packet_base(const uint16_t sub_cmd_id,
                                                  const uint16_t receiver_id,
                                                  const void *data,
                                                  const uint16_t len)
{
    // 边界修复：交互数据段负载包含 6 字节 header，总长不得超过 118
    if (len + sizeof(interaction_header_t) > 118)
        return false;

    uint8_t buffer[128]; // Use stack buffer
    auto *p_header        = reinterpret_cast<interaction_header_t *>(buffer);

    p_header->data_cmd_id = sub_cmd_id;
    p_header->sender_id   = _robot_id;
    p_header->receiver_id = receiver_id;

    if (len > 0 && data != nullptr)
    {
        memcpy(buffer + sizeof(interaction_header_t), data, len);
    }

    return send_packet(cmd_id::STUDENT_INTERACTIVE, buffer,
                       sizeof(interaction_header_t) + len);
}

bool referee_drv_t::send_robot_interaction(const uint16_t receiver_id,
                                           const uint16_t sub_cmd_id,
                                           const void *data, const uint16_t len)
{
    if (_robot_id == 0)
        return false;

    const bool is_my_team_red = (_robot_id < 100);
    const bool is_target_red  = (receiver_id < 100);

    if (is_my_team_red != is_target_red)
        return false;

    return _send_interaction_packet_base(sub_cmd_id, receiver_id, data, len);
}

bool referee_drv_t::send_ui_interaction(const uint16_t sub_cmd_id,
                                        const void *data)
{
    uint8_t len = 0;
    switch (sub_cmd_id)
    {
        case static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DELETE):
            len = 2;
            break;
        case static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_1):
            len = 15;
            break;
        case static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_2):
            len = 30;
            break;
        case static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_5):
            len = 75;
            break;
        case static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_7):
            len = 105;
            break;
        case static_cast<uint16_t>(interaction_sub_cmd::UI_CMD_DRAW_CHAR):
            len = 45;
            break;
        default:
            len = 0;
            break;
    }
    bool ret =
        _send_interaction_packet_base(sub_cmd_id, get_client_id(), data, len);
    vTaskDelay(pdMS_TO_TICKS(40));
    return ret;
}

bool referee_drv_t::send_custom_info(const char *message)
{
    if (_robot_id == 0)
        return false;

    custom_info_t custom_data{};
    custom_data.sender_id   = _robot_id;
    custom_data.receiver_id = get_client_id();

    memset(custom_data.user_data, 0, sizeof(custom_data.user_data));

    size_t str_len = std::strlen(message);
    if (str_len > 30)
        str_len = 30;

    memcpy(custom_data.user_data, message, str_len);

    return send_packet(cmd_id::MAP_RECEIVE_ROBOT, &custom_data,
                       sizeof(custom_info_t));
}

// ==========================================================================
// RX Implementation
// ==========================================================================

__attribute__((section(".itcm_text"))) bool
referee_drv_t::rx_callback(uint8_t *p, const uint16_t size,
                           BaseType_t task_woken)
{
    // FIFO expects char*
    fifo_s_puts(&_fifo, reinterpret_cast<char *>(p), size);
    return true;
}

void referee_drv_t::unpack_fifo_data()
{
    uint8_t byte = 0;

    // Use local references for cleaner code
    auto &step   = _unpack_obj.step;
    auto &idx    = _unpack_obj.index;
    auto &packet = _unpack_obj.protocol_packet;
    auto &len    = _unpack_obj.data_len;

    while (fifo_s_used(&_fifo))
    {
        byte = static_cast<uint8_t>(fifo_s_get(&_fifo));

        switch (step)
        {
            case unpack_step::HEADER_SOF:
                if (byte == HEADER_SOF)
                {
                    step          = unpack_step::LENGTH_LOW;
                    idx           = 0;
                    packet[idx++] = byte;
                }
                else
                {
                    idx = 0;
                }
                break;

            case unpack_step::LENGTH_LOW:
                len           = byte;
                packet[idx++] = byte;
                step          = unpack_step::LENGTH_HIGH;
                break;

            case unpack_step::LENGTH_HIGH:
                len |= (static_cast<uint16_t>(byte) << 8);
                packet[idx++] = byte;

                if (len < (FRAME_MAX_SIZE - HEADER_CRC_CMDID_LEN))
                    step = unpack_step::FRAME_SEQ;
                else
                {
                    step = unpack_step::HEADER_SOF;
                    idx  = 0;
                }
                break;

            case unpack_step::FRAME_SEQ:
                packet[idx++] = byte;
                step          = unpack_step::HEADER_CRC8;
                break;

            case unpack_step::HEADER_CRC8:
                packet[idx++] = byte;

                if (idx == HEADER_SIZE)
                {
                    if (verify_crc8_check_sum(packet, HEADER_SIZE))
                        step = unpack_step::DATA_CRC16;
                    else
                    {
                        step = unpack_step::HEADER_SOF;
                        idx  = 0;
                    }
                }
                break;

            case unpack_step::DATA_CRC16:
                if (idx < (HEADER_CRC_CMDID_LEN + len))
                {
                    packet[idx++] = byte;
                }

                if (idx >= (HEADER_CRC_CMDID_LEN + len))
                {
                    step = unpack_step::HEADER_SOF;
                    idx  = 0; // Prepare for next frame

                    if (verify_crc16_check_sum(packet,
                                               HEADER_CRC_CMDID_LEN + len))
                    {
                        solve_data(packet);
                    }
                }
                break;
        }
    }
}

void referee_drv_t::solve_data(const uint8_t *frame)
{
    auto *p_header = reinterpret_cast<const frame_header_t *>(frame);
    const uint16_t data_length =
        p_header->data_length; // 获取协议发来的实际长度

    uint16_t cmd_id_val = 0;
    uint8_t index       = HEADER_SIZE;

    // Extract CMD_ID
    memcpy(&cmd_id_val, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    _is_online        = true;
    _last_update_time = dwt_drv_t::get_timeline_ms();

    // White-list check
    if (cmd_id_val >= MAX_CMD_ID_COUNT || !_enabled_ids.test(cmd_id_val))
        return;

    // Convert integer to Enum Class for switch-case
    auto cmd = static_cast<cmd_id>(cmd_id_val);

    switch (cmd)
    {
        case cmd_id::GAME_STATE:
            safe_copy(_data.game_status, frame + index, data_length);
            break;
        case cmd_id::GAME_RESULT:
            safe_copy(_data.game_result, frame + index, data_length);
            break;
        case cmd_id::GAME_ROBOT_HP:
            safe_copy(_data.game_robot_hp, frame + index, data_length);
            break;
        case cmd_id::FIELD_EVENTS:
            safe_copy(_data.field_event, frame + index, data_length);
            break;
        case cmd_id::REFEREE_WARNING:
            safe_copy(_data.referee_warning, frame + index, data_length);
            break;
        case cmd_id::DART_INFO:
            safe_copy(_data.dart_info, frame + index, data_length);
            break;
        case cmd_id::ROBOT_STATE:
            safe_copy(_data.robot_status, frame + index, data_length);
            _robot_id = _data.robot_status.robot_id; // 自动同步真实ID
            break;
        case cmd_id::POWER_HEAT_DATA:
            safe_copy(_data.power_heat, frame + index, data_length);
            break;
        case cmd_id::ROBOT_POS:
            safe_copy(_data.robot_pos, frame + index, data_length);
            break;
        case cmd_id::BUFF_INFO:
            safe_copy(_data.buff, frame + index, data_length);
            break;
        case cmd_id::ROBOT_HURT:
            safe_copy(_data.hurt, frame + index, data_length);
            break;
        case cmd_id::SHOOT_DATA:
            safe_copy(_data.shoot, frame + index, data_length);
            _data.shoot_launching_count++;
            break;
        case cmd_id::PROJECTILE_ALLOWANCE:
            safe_copy(_data.allowance, frame + index, data_length);
            break;
        case cmd_id::ROBOT_RFID:
            safe_copy(_data.rfid, frame + index, data_length);
            break;
        case cmd_id::DART_CLIENT_CMD:
            safe_copy(_data.dart_client_cmd, frame + index, data_length);
            break;
        case cmd_id::GROUND_ROBOT_POS:
            safe_copy(_data.ground_robot_pos, frame + index, data_length);
            break;
        case cmd_id::RADAR_MARK:
            safe_copy(_data.radar_mark, frame + index, data_length);
            break;
        case cmd_id::SENTRY_INFO:
            safe_copy(_data.sentry_info, frame + index, data_length);
            break;
        case cmd_id::RADAR_INFO:
            safe_copy(_data.radar_info, frame + index, data_length);
            break;
        case cmd_id::TINY_MAP_INTERACT:
            safe_copy(_data.map_command, frame + index, data_length);
            break;
        case cmd_id::STUDENT_INTERACTIVE:
            safe_copy(_data.robot_interaction, frame + index, data_length);
            break;

        default:
            break;
    }
}

} // namespace pyro
