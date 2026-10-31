/**
 * @version 1.1.0
 */

#include "pyro_can_drv.h"
#include "main.h"
#include <cstring>

namespace pyro
{
namespace
{
constexpr uint32_t CAN_NOTIFY_FLAGS =
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
    FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_ERROR_WARNING |
    FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF |
    FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR;

void abort_pending_tx(FDCAN_HandleTypeDef *hfdcan)
{
    if (nullptr == hfdcan)
        return;

    const uint32_t pending = hfdcan->Instance->TXBRP;
    if (0U != pending)
        (void)HAL_FDCAN_AbortTxRequest(hfdcan, pending);
}

status_t reactivate_can_notifications(FDCAN_HandleTypeDef *hfdcan)
{
    if (HAL_OK != HAL_FDCAN_ActivateNotification(hfdcan, CAN_NOTIFY_FLAGS, 0))
        return PYRO_ERROR;

    return PYRO_OK;
}

status_t recover_bus_off(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_ProtocolStatusTypeDef protocol_status{};

    if (HAL_OK != HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status))
        return PYRO_ERROR;

    if (0U == protocol_status.BusOff)
        return PYRO_OK;

    abort_pending_tx(hfdcan);

    (void)HAL_FDCAN_Stop(hfdcan);
    hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;

    if (HAL_OK != HAL_FDCAN_Start(hfdcan))
        return PYRO_ERROR;

    const status_t notify_status = reactivate_can_notifications(hfdcan);
    if (PYRO_OK != notify_status)
        return notify_status;

    return PYRO_BUSY;
}
} // namespace

// ==========================================
// can_msg_buffer_t Implementation
// ==========================================
can_msg_buffer_t::can_msg_buffer_t(const uint32_t id, const id_type_t type)
    : _id(id), _mask(0xFFFFFFFF), _use_mask(false), _id_type(type), _buffer{}, _is_fresh(false), _last_update_time(0), _received_id(0)
{
}

can_msg_buffer_t::can_msg_buffer_t(const uint32_t id, const uint32_t mask, const id_type_t type)
    : _id(id), _mask(mask), _use_mask(true), _id_type(type), _buffer{}, _is_fresh(false), _last_update_time(0), _received_id(0)
{
}

can_msg_buffer_t::~can_msg_buffer_t() = default;

uint32_t can_msg_buffer_t::get_id() const
{
    return _id;
}

can_msg_buffer_t::id_type_t can_msg_buffer_t::get_id_type() const
{
    return _id_type;
}

bool can_msg_buffer_t::is_fresh() const
{
    return _is_fresh;
}

void can_msg_buffer_t::mark_read()
{
    _is_fresh = false;
}

uint32_t can_msg_buffer_t::get_mask() const
{
    return _mask;
}

bool can_msg_buffer_t::matches(uint32_t received_id) const
{
    if (!_use_mask)
    {
        // 精确匹配
        return received_id == _id;
    }
    else
    {
        // 掩码匹配：只比较mask中为1的位
        return (received_id & _mask) == (_id & _mask);
    }
}

TickType_t can_msg_buffer_t::get_last_update_time() const
{
    return _last_update_time;
}

__attribute__((section(".itcm_text"))) void
can_msg_buffer_t::update_data(const uint8_t *data, const uint32_t received_id)
{
    // [中断安全] 使用带 FROM_ISR 后缀的临界区宏
    const UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    memcpy(_buffer.data(), data, 8);
    _received_id      = received_id;
    _last_update_time = xTaskGetTickCountFromISR();
    _is_fresh         = true;

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

bool can_msg_buffer_t::get_data(std::array<uint8_t, 8> &data) const
{
    // [任务安全] 防止在读取时被 CAN 接收中断打断导致脏数据
    taskENTER_CRITICAL();
    memcpy(data.data(), _buffer.data(), 8);
    const bool fresh_status = _is_fresh;
    taskEXIT_CRITICAL();

    return fresh_status;
}

bool can_msg_buffer_t::get_data(std::array<uint8_t, 8> &data, uint32_t &received_id) const
{
    // [任务安全] 防止在读取时被 CAN 接收中断打断导致脏数据
    taskENTER_CRITICAL();
    memcpy(data.data(), _buffer.data(), 8);
    received_id = _received_id;
    const bool fresh_status = _is_fresh;
    taskEXIT_CRITICAL();

    return fresh_status;
}

uint32_t can_msg_buffer_t::get_received_id() const
{
    return _received_id;
}

// ==========================================
// can_drv_t Implementation
// ==========================================
can_drv_t::can_drv_t(FDCAN_HandleTypeDef *hfdcan) : _hfdcan(hfdcan)
{
    _registerlist.clear();
}

can_drv_t::~can_drv_t() = default;

pyro::status_t can_drv_t::init()
{
    // Configure standard ID filter
    FDCAN_FilterTypeDef fdcan_filter_std;
    fdcan_filter_std.IdType       = FDCAN_STANDARD_ID;
    fdcan_filter_std.FilterIndex  = 0;
    fdcan_filter_std.FilterType   = FDCAN_FILTER_MASK;
    fdcan_filter_std.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter_std.FilterID1    = 0x00;
    fdcan_filter_std.FilterID2    = 0x00;

    if (HAL_OK != HAL_FDCAN_ConfigFilter(_hfdcan, &fdcan_filter_std))
        return pyro::PYRO_ERROR;

    // Configure extended ID filter
    FDCAN_FilterTypeDef fdcan_filter_ext;
    fdcan_filter_ext.IdType       = FDCAN_EXTENDED_ID;
    fdcan_filter_ext.FilterIndex  = 0;
    fdcan_filter_ext.FilterType   = FDCAN_FILTER_MASK;
    fdcan_filter_ext.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter_ext.FilterID1    = 0x00;
    fdcan_filter_ext.FilterID2    = 0x00;

    if (HAL_OK != HAL_FDCAN_ConfigFilter(_hfdcan, &fdcan_filter_ext))
        return pyro::PYRO_ERROR;

    // Configure global filter to accept both standard and extended frames
    if (HAL_OK !=
        HAL_FDCAN_ConfigGlobalFilter(_hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE))
        return pyro::PYRO_ERROR;
    if (HAL_OK != HAL_FDCAN_ConfigFifoWatermark(_hfdcan, FDCAN_CFG_RX_FIFO0, 1))
        return pyro::PYRO_ERROR;

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::start() const
{
    if (HAL_OK != HAL_FDCAN_Start(_hfdcan))
        return pyro::PYRO_ERROR;

    return reactivate_can_notifications(_hfdcan);
}

pyro::status_t can_drv_t::send_msg(const uint32_t id, const uint8_t *data,
                                   const can_msg_buffer_t::id_type_t type) const
{
    if (nullptr == _hfdcan || nullptr == data)
        return pyro::PYRO_PARAM_ERROR;

    if (HAL_FDCAN_STATE_BUSY != _hfdcan->State)
        return pyro::PYRO_BUSY;

    const status_t recover_status = recover_bus_off(_hfdcan);
    if (PYRO_OK != recover_status)
        return recover_status;

    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.IdType = (type == can_msg_buffer_t::EXTENDED_ID)
                       ? FDCAN_EXTENDED_ID
                       : FDCAN_STANDARD_ID;
    tx_header.Identifier          = id;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    taskENTER_CRITICAL();
    if (0U == HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan))
    {
        abort_pending_tx(_hfdcan);
        _hfdcan->ErrorCode &= ~HAL_FDCAN_ERROR_FIFO_FULL;
        taskEXIT_CRITICAL();
        return pyro::PYRO_BUSY;
    }

    const HAL_StatusTypeDef hal_status =
        HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &tx_header, data);

    if (HAL_OK != hal_status)
    {
        if (0U != (_hfdcan->ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL))
        {
            abort_pending_tx(_hfdcan);
            _hfdcan->ErrorCode &= ~HAL_FDCAN_ERROR_FIFO_FULL;
            taskEXIT_CRITICAL();
            return pyro::PYRO_BUSY;
        }

        taskEXIT_CRITICAL();
        return pyro::PYRO_ERROR;
    }
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::register_rx_msg(can_msg_buffer_t *msg_buffer)
{
    if (msg_buffer->get_mask() == 0xFFFFFFFF)
    {
        // 精确匹配，用map存储
        const register_key_t key(msg_buffer->get_id(), msg_buffer->get_id_type());

        taskENTER_CRITICAL();
        if (this->_registerlist.exist(key))
        {
            taskEXIT_CRITICAL();
            return pyro::PYRO_ERROR;
        }
        this->_registerlist[key] = msg_buffer;
        taskEXIT_CRITICAL();
    }
    else
    {
        // 使用掩码，存到数组
        taskENTER_CRITICAL();
        if (_mask_buffer_count < 10)
        {
            _mask_buffers[_mask_buffer_count++] = msg_buffer;
            taskEXIT_CRITICAL();
        }
        else
        {
            taskEXIT_CRITICAL();
            return PYRO_ERROR;
        }
    }

    return pyro::PYRO_OK;
}

// CAN调试全局变量（外部定义，如果不存在则使用弱引用）
extern "C" {
    extern uint32_t can_debug_last_extended_id __attribute__((weak));
    extern uint32_t can_debug_frame_count __attribute__((weak));
    extern uint32_t can_debug_ids[10] __attribute__((weak));
    extern uint8_t can_debug_id_index __attribute__((weak));
}

__attribute__((section(".itcm_text"))) pyro::status_t
can_drv_t::handle_rx_msg(const uint32_t id, const uint8_t *data,
                         const can_msg_buffer_t::id_type_t type)
{
    // 调试：记录所有扩展帧（如果调试变量存在）
    if (type == can_msg_buffer_t::EXTENDED_ID)
    {
        if (&can_debug_last_extended_id != nullptr)
        {
            can_debug_last_extended_id = id;
            can_debug_frame_count++;
            can_debug_ids[can_debug_id_index] = id;
            can_debug_id_index = (can_debug_id_index + 1) % 10;
        }
    }

    // 先尝试精确匹配（快速路径）
    const register_key_t key(id, type);
    if (this->_registerlist.exist(key))
    {
        can_msg_buffer_t *msg = this->_registerlist[key];
        msg->update_data(data, id);
        return pyro::PYRO_OK;
    }

    // 如果精确匹配失败，尝试掩码匹配
    for (int i = 0; i < _mask_buffer_count; i++)
    {
        can_msg_buffer_t *msg = _mask_buffers[i];
        if (msg && msg->get_id_type() == type && msg->matches(id))
        {
            msg->update_data(data, id);
            return pyro::PYRO_OK;
        }
    }

    return pyro::PYRO_NOT_FOUND;
}

}; // namespace pyro

// ==========================================
// Global Hardware Interruption Callbacks
// ==========================================
// HAL_FDCAN_RxFifo0Callback is now implemented in pyro_bsp_can.cpp
