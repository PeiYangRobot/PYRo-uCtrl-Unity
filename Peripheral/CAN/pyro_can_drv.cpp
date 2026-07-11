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
can_msg_buffer_t::can_msg_buffer_t(const uint32_t id)
    : _id(id), _buffer{}, _is_fresh(false), _last_update_time(0)
{
}

can_msg_buffer_t::~can_msg_buffer_t() = default;

uint32_t can_msg_buffer_t::get_id() const
{
    return _id;
}

bool can_msg_buffer_t::is_fresh() const
{
    return _is_fresh;
}

void can_msg_buffer_t::mark_read()
{
    _is_fresh = false;
}

TickType_t can_msg_buffer_t::get_last_update_time() const
{
    return _last_update_time;
}

__attribute__((section(".itcm_text"))) void
can_msg_buffer_t::update_data(const uint8_t *data)
{
    const UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    memcpy(_buffer.data(), data, 8);
    _last_update_time = xTaskGetTickCountFromISR();
    _is_fresh         = true;

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

bool can_msg_buffer_t::get_data(std::array<uint8_t, 8> &data) const
{
    taskENTER_CRITICAL();
    memcpy(data.data(), _buffer.data(), 8);
    const bool fresh_status = _is_fresh;
    taskEXIT_CRITICAL();

    return fresh_status;
}

// ==========================================
// can_drv_t Implementation
// ==========================================
can_drv_t::can_drv_t(FDCAN_HandleTypeDef *hfdcan) : _hfdcan(hfdcan)
{
    _registerlist.clear();
    can_map()[hfdcan] = this;
}

can_drv_t::~can_drv_t()
{
    can_map().erase(_hfdcan);
}

map_t<FDCAN_HandleTypeDef *, can_drv_t *> &can_drv_t::can_map()
{
    static map_t<FDCAN_HandleTypeDef *, can_drv_t *> instance;
    return instance;
}

pyro::status_t can_drv_t::init() const
{
    FDCAN_FilterTypeDef fdcan_filter;
    fdcan_filter.IdType       = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex  = 0;
    fdcan_filter.FilterType   = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1    = 0x00;
    fdcan_filter.FilterID2    = 0x00;

    if (HAL_OK != HAL_FDCAN_ConfigFilter(_hfdcan, &fdcan_filter))
        return pyro::PYRO_ERROR;
    if (HAL_OK !=
        HAL_FDCAN_ConfigGlobalFilter(_hfdcan, FDCAN_REJECT, FDCAN_REJECT,
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

pyro::status_t can_drv_t::send_msg(const uint32_t id, const uint8_t *data) const
{
    if (nullptr == _hfdcan || nullptr == data)
        return pyro::PYRO_PARAM_ERROR;

    if (HAL_FDCAN_STATE_BUSY != _hfdcan->State)
        return pyro::PYRO_BUSY;

    const status_t recover_status = recover_bus_off(_hfdcan);
    if (PYRO_OK != recover_status)
        return recover_status;

    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.IdType              = FDCAN_STANDARD_ID;
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
    const uint32_t id = msg_buffer->get_id();

    taskENTER_CRITICAL();
    if (this->_registerlist.exist(id))
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_ERROR;
    }
    this->_registerlist[id] = msg_buffer;
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

__attribute__((section(".itcm_text"))) pyro::status_t
can_drv_t::handle_rx_msg(const uint32_t id, const uint8_t *data)
{
    if (!this->_registerlist.exist(id))
    {
        return pyro::PYRO_NOT_FOUND;
    }

    can_msg_buffer_t *msg = this->_registerlist[id];
    msg->update_data(data);

    return pyro::PYRO_OK;
}

}; // namespace pyro

// ==========================================
// Global Hardware Interruption Callbacks
// ==========================================
__attribute__((section(".itcm_text"))) void
can_global_handle(FDCAN_HandleTypeDef *hfdcan, const uint32_t identifier,
                  const uint8_t *data)
{
    auto &map = pyro::can_drv_t::can_map();
    if (map.exist(hfdcan))
    {
        (void)map[hfdcan]->handle_rx_msg(identifier, data);
    }
}

extern "C" __attribute__((section(".itcm_text"))) void
HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    if (HAL_OK !=
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data))
    {
        return;
    }

    if (FDCAN_FRAME_CLASSIC == rx_header.RxFrameType &&
        FDCAN_STANDARD_ID == rx_header.IdType)
    {
        can_global_handle(hfdcan, rx_header.Identifier, data);
    }
}
