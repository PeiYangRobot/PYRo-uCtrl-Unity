/**
 * @file pyro_image_drv.cpp
 * @brief RoboMaster 图传链路驱动实现 (轻量级优化版)
 */

#include "pyro_image_drv.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_crc.h"
#include "pyro_dwt_drv.h"
#include "pyro_core_dma_heap.h"
#include <cstring>

namespace pyro
{

#ifdef IMAGE_UART
image_drv_t &image_drv_t::get_instance()
{
    static image_drv_t instance(&IMAGE_UART);
    return instance;
}
#endif

image_drv_t::image_drv_t(uart_drv_t *uart_handle)
    : _uart(uart_handle), _task(nullptr), _rx_msg_buf(nullptr),
      _tx_controller_pkt(nullptr), _tx_client_pkt(nullptr),
      _subscribed_count(0), _is_online(false), _last_update_time(0.0f), _send_seq(0)
{
    _rx_msg_buf = xMessageBufferCreate(MSG_BUF_SIZE);

    _tx_controller_pkt = static_cast<tx_controller_packet_t *>(pvPortDmaMalloc(sizeof(tx_controller_packet_t)));
    _tx_client_pkt     = static_cast<tx_client_packet_t *>(pvPortDmaMalloc(sizeof(tx_client_packet_t)));

    if (_tx_controller_pkt) memset(_tx_controller_pkt, 0, sizeof(tx_controller_packet_t));
    if (_tx_client_pkt)     memset(_tx_client_pkt, 0, sizeof(tx_client_packet_t));

    memset(_subscribed_ids, 0, sizeof(_subscribed_ids));

    _task = new image_task_t(this);
}

image_drv_t::~image_drv_t()
{
    if (_tx_controller_pkt) vPortFree(_tx_controller_pkt);
    if (_tx_client_pkt)     vPortFree(_tx_client_pkt);
    if (_rx_msg_buf)        vMessageBufferDelete(_rx_msg_buf);

    if (_uart)
        _uart->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));

    if (_task)
    {
        _task->stop();
        delete _task;
    }
}

void image_drv_t::init(std::initializer_list<cmd_id> listening_ids)
{
    if (!_uart) return;

    _subscribed_count = 0;
    for (auto id : listening_ids)
    {
        if (_subscribed_count < MAX_SUBSCRIBE_NUM)
        {
            _subscribed_ids[_subscribed_count++] = static_cast<uint16_t>(id);
        }
    }

    _uart->add_rx_event_callback(
        [this](const uint8_t *p, uint16_t s, BaseType_t &tw) -> bool
        { return this->rx_callback(p, s, tw); },
        reinterpret_cast<uint32_t>(this));
}

void image_drv_t::init()
{
    // 默认订阅
    init({cmd_id::CUSTOM_CONTROLLER, static_cast<cmd_id>(0x0311)});
}

void image_drv_t::start() const
{
    if (_task && _tx_controller_pkt && _tx_client_pkt)
        _task->start();
}

/**
 * @brief ISR 接收回调
 */
bool image_drv_t::rx_callback(const uint8_t *p, uint16_t size, BaseType_t &task_woken) const
{
    if (size < HEADER_SIZE || p[0] != HEADER_SOF)
        return false;

    if (size < HEADER_SIZE + 2)
        return true;

    uint16_t data_len   = p[1] | (static_cast<uint16_t>(p[2]) << 8);
    uint16_t cmd_id_val = p[5] | (static_cast<uint16_t>(p[6]) << 8);
    uint16_t total_len  = HEADER_SIZE + 2 + data_len + 2;

    // O(N) 遍历过滤，对于 N<=4 性能远高于操作大体积结构体
    bool is_subscribed = false;
    for (uint8_t i = 0; i < _subscribed_count; ++i)
    {
        if (_subscribed_ids[i] == cmd_id_val)
        {
            is_subscribed = true;
            break;
        }
    }

    if (!is_subscribed) return true; // 是 0xA5 但未订阅，拦截

    if (size >= total_len && _rx_msg_buf != nullptr)
    {
        xMessageBufferSendFromISR(_rx_msg_buf, p, total_len, &task_woken);
    }

    return true;
}

void image_drv_t::task_loop()
{
    uint8_t frame_temp[MAX_FRAME_LEN];
    while (true)
    {
        size_t recv_len = xMessageBufferReceive(_rx_msg_buf, frame_temp, MAX_FRAME_LEN, pdMS_TO_TICKS(100));

        if (recv_len > 0)
        {
            if (verify_crc8_check_sum(frame_temp, HEADER_SIZE) &&
                verify_crc16_check_sum(frame_temp, recv_len))
            {
                solve_frame(frame_temp, static_cast<uint16_t>(recv_len));
            }
        }

        if (dwt_drv_t::get_timeline_ms() - _last_update_time > 500.0f)
        {
            _is_online = false;
        }
    }
}

void image_drv_t::solve_frame(const uint8_t *frame, uint16_t full_len)
{
    // C++ 提取方式
    uint16_t data_len   = frame[1] | (static_cast<uint16_t>(frame[2]) << 8);
    uint16_t cmd_id_val = frame[5] | (static_cast<uint16_t>(frame[6]) << 8);
    const uint8_t *data_ptr = frame + HEADER_SIZE + 2;

    _is_online = true;
    _last_update_time = dwt_drv_t::get_timeline_ms();

    if (cmd_id_val == static_cast<uint16_t>(cmd_id::CUSTOM_CONTROLLER))
    {
        size_t copy_len = (data_len > 30) ? 30 : data_len;
        memcpy(_controller_rx_buf, data_ptr, copy_len);
    }
    else if (cmd_id_val == 0x0311)
    {
        size_t copy_len = (data_len > 30) ? 30 : data_len;
        memcpy(_client_rx_buf, data_ptr, copy_len);
    }
}

status_t image_drv_t::send_controller_data()
{
    if (!_uart || !_tx_controller_pkt) return PYRO_ERROR;

    _tx_controller_pkt->header.sof = HEADER_SOF;
    _tx_controller_pkt->header.data_length = sizeof(tx_controller_payload_t);
    _tx_controller_pkt->header.seq = _send_seq++;
    _tx_controller_pkt->header.crc8 = 0;

    append_crc8_check_sum(reinterpret_cast<uint8_t *>(&_tx_controller_pkt->header), HEADER_SIZE);

    _tx_controller_pkt->cmd_id = 0x0309;

    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_controller_pkt), sizeof(tx_controller_packet_t));

    auto ret = _uart->write(reinterpret_cast<uint8_t *>(_tx_controller_pkt), sizeof(tx_controller_packet_t));
    if (ret != PYRO_OK)
    {
        _send_seq--;
    }

    return ret;
}

status_t image_drv_t::send_client_data()
{
    if (!_uart || !_tx_client_pkt) return PYRO_ERROR;

    _tx_client_pkt->header.sof = HEADER_SOF;
    _tx_client_pkt->header.data_length = sizeof(tx_client_payload_t);
    _tx_client_pkt->header.seq = _send_seq;
    _tx_client_pkt->header.crc8 = 0;

    append_crc8_check_sum(reinterpret_cast<uint8_t *>(&_tx_client_pkt->header), HEADER_SIZE);

    _tx_client_pkt->cmd_id = 0x0310;

    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_client_pkt), sizeof(tx_client_packet_t));

    status_t ret = _uart->write(reinterpret_cast<uint8_t *>(_tx_client_pkt), sizeof(tx_client_packet_t));
    _send_seq++;
    if (ret != PYRO_OK)
    {
        _send_seq--;
    }
    return ret;
}

} // namespace pyro