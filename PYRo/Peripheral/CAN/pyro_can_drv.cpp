#include "pyro_can_drv.h"
#include "main.h"

#include <cstring>



namespace pyro
{
can_msg_buffer_t::can_msg_buffer_t(uint32_t id)
    : _id(id), _is_fresh(false), _last_update_time(0)
{
    _buffer.fill(0);
    //_mtx = xSemaphoreCreateMutex();
}

can_msg_buffer_t::~can_msg_buffer_t(void)
{
    // vSemaphoreDelete(_mtx);
}

uint32_t can_msg_buffer_t::get_id(void)
{
    return _id;
}

bool can_msg_buffer_t::is_fresh(void)
{
    return _is_fresh;
}

void can_msg_buffer_t::mark_read(void)
{
    // if(xSemaphoreTake(_mtx,portMAX_DELAY)==pdTRUE){
    _is_fresh = false;
    // xSemaphoreGive(_mtx);
    //}
}

void can_msg_buffer_t::update_data(const uint8_t *data) // Mutex or not
{
    // if(xSemaphoreTake(_mtx,portMAX_DELAY)==pdTRUE){
    memcpy(_buffer.data(), data, 8);
    _last_update_time = xTaskGetTickCount();
    _is_fresh         = true;
    // xSemaphoreGive(_mtx);
    // }
}

bool can_msg_buffer_t::get_data(std::array<uint8_t, 8> &data)
{
    // if(xSemaphoreTake(_mtx,portMAX_DELAY)==pdTRUE){
    memcpy(data.data(), _buffer.data(), 8);
    // xSemaphoreGive(_mtx);
    return true;
    // }
    // return false;
}



can_drv_t::can_drv_t(FDCAN_HandleTypeDef *hfdcan)
{
    _hfdcan = hfdcan;
    _registerlist.clear();
    //_registermtx = xSemaphoreCreateMutex();
}

can_drv_t::~can_drv_t(void)
{
    // vSemaphoreDelete(_registermtx);
}

pyro::status_t can_drv_t::init(void)
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
    if (pyro::PYRO_OK !=
        pyro::can_hub_t::get_instance()->hub_register_can_obj(_hfdcan, this))
        return pyro::PYRO_ERROR;

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::start(void)
{
    if (HAL_OK != HAL_FDCAN_Start(_hfdcan))
        return pyro::PYRO_ERROR;
    if (HAL_OK != HAL_FDCAN_ActivateNotification(
                      _hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0))
        return pyro::PYRO_ERROR;
    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::send_msg(uint32_t id, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef tx_header;
    // if(xSemaphoreTake(_registermtx,portMAX_DELAY)==pdTRUE){
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.Identifier          = id;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = 8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &tx_header, data))
    {
        // xSemaphoreGive(_registermtx);
        return pyro::PYRO_ERROR;
    }

    // xSemaphoreGive(_registermtx);
    return pyro::PYRO_OK;
    // }
    // return pyro::PYRO_ERROR;
}

pyro::status_t can_drv_t::register_rx_msg(can_msg_buffer_t *msg_buffer)
{
    // if(xSemaphoreTake(_registermtx,portMAX_DELAY)==pdTRUE)
    // {
    uint32_t id = msg_buffer->get_id();
    if (this->_registerlist.exist(id))
    {
        // xSemaphoreGive(_registermtx);
        return pyro::PYRO_ERROR;
    }
    this->_registerlist[id] = msg_buffer;
    // xSemaphoreGive(_registermtx);
    return pyro::PYRO_OK;
    // }
    // return pyro::PYRO_ERROR;
}

pyro::status_t can_drv_t::handle_rx_msg(uint32_t id,
                                        uint8_t *data) // mutex or not
{
    // if(xSemaphoreTake(_registermtx,portMAX_DELAY)==pdTRUE)
    // {
    if (!this->_registerlist.exist(id))
    {
        // xSemaphoreGive(_registermtx);
        return pyro::PYRO_NOT_FOUND;
    }
    can_msg_buffer_t *msg = this->_registerlist[id];
    msg->update_data(data);
    // xSemaphoreGive(_registermtx);
    return pyro::PYRO_OK;
    // }
    // return pyro::PYRO_ERROR;
}

can_hub_t::can_hub_t() : _can_drv_map()
{ // Log
    this->_can_drv_map.clear();
}

can_hub_t *can_hub_t::_instancePtr = nullptr;
can_hub_t *_instancePtr_copy       = nullptr;
can_hub_t *can_hub_t::get_instance(void)
{
    if (_instancePtr == nullptr) // Mutex
    {
        _instancePtr      = new can_hub_t();
        _instancePtr_copy = _instancePtr;
    }
    return _instancePtr;
}
pyro::status_t can_hub_t::hub_register_can_obj(FDCAN_HandleTypeDef *hfdcan,
                                               can_drv_t *can_drv)
{
    if (this->_can_drv_map.exist(hfdcan))
        return PYRO_ERROR;
    this->_can_drv_map[hfdcan] = can_drv;
    return pyro::PYRO_OK;
}

status_t can_hub_t::hub_unregister_can_obj(FDCAN_HandleTypeDef *hfdcan)
{
    if (!this->_can_drv_map.exist(hfdcan))
        return pyro::PYRO_ERROR;
    this->_can_drv_map.erase(hfdcan);
    return pyro::PYRO_OK;
}

can_drv_t *can_hub_t::hub_get_can_obj(which_can which_can)
{
    FDCAN_HandleTypeDef *hfdcan = nullptr;
    switch (which_can)
    {
        case can1:
            hfdcan = &hfdcan1;
            break;
        case can2:
            hfdcan = &hfdcan2;
            break;
        case can3:
            hfdcan = &hfdcan3;
            break;
        default:
            return nullptr;
    }
    can_drv_t *can_drv = this->_can_drv_map[hfdcan];
    return can_drv;
}
//    pyro::status_t hub_unregister_can_client(which_can which_can,uint32_t id);

pyro::status_t can_hub_t::hub_handle_callback(FDCAN_HandleTypeDef *hfdcan,
                                              uint32_t identifier,
                                              uint8_t *data)
{
    if (!this->_can_drv_map.exist(hfdcan))
        return pyro::PYRO_ERROR;
    // return this->_can_drv_map[hfdcan]->hub_handle_callback(hfdcan, data);
    return this->_can_drv_map[hfdcan]->handle_rx_msg(identifier, data);
}

}; // namespace pyro

void can_global_handle(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier,
                       uint8_t *data)
{
    pyro::can_hub_t::get_instance()->hub_handle_callback(hfdcan, identifier,
                                                         data);
}

FDCAN_RxHeaderTypeDef rx_header;
uint32_t a;
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs)
{

    uint8_t data[8];
    if (HAL_OK !=
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data))
    {
        return;
    }
    if (FDCAN_FRAME_CLASSIC == rx_header.RxFrameType &&
        FDCAN_STANDARD_ID == rx_header.IdType)
    {
        if (hfdcan == &hfdcan1)
            a++;
        else if (hfdcan == &hfdcan2)
            a--;
        else if (hfdcan == &hfdcan3)
            a;

        can_global_handle(hfdcan, rx_header.Identifier, data);
    }
}