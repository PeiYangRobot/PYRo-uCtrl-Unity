/**
 * @file pyro_bsp_can.cpp
 * @brief Implementation of the Board Support Package for CAN.
 * CAN板级支持包实现文件。
 *
 * @author Antigravity
 * @version 1.0.0
 * @date 2026-07-03
 */

#include "pyro_bsp_can.h"
#include "fdcan.h"

namespace pyro
{

can_drv_t& bsp_can::get_can1()
{
    static can_drv_t instance(&hfdcan1);
    return instance;
}

can_drv_t& bsp_can::get_can2()
{
    static can_drv_t instance(&hfdcan2);
    return instance;
}

can_drv_t& bsp_can::get_can3()
{
    static can_drv_t instance(&hfdcan3);
    return instance;
}

can_drv_t* bsp_can::get_can(const which_can which)
{
    switch (which)
    {
        case can1:
            return &get_can1();
        case can2:
            return &get_can2();
        case can3:
            return &get_can3();
        default:
            return nullptr;
    }
}

status_t bsp_can::init_all()
{
    CHECK_PYRO_RET(get_can1().init());
    CHECK_PYRO_RET(get_can2().init());
    CHECK_PYRO_RET(get_can3().init());

    CHECK_PYRO_RET(get_can1().start());
    CHECK_PYRO_RET(get_can2().start());
    CHECK_PYRO_RET(get_can3().start());
    return PYRO_OK;
}

} // namespace pyro

// ==========================================
// Global Hardware Interruption Callbacks
// ==========================================

static pyro::can_drv_t* get_can_drv_from_handle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1)
        return &pyro::bsp_can::get_can1();
    if (hfdcan == &hfdcan2)
        return &pyro::bsp_can::get_can2();
    if (hfdcan == &hfdcan3)
        return &pyro::bsp_can::get_can3();
    return nullptr;
}

extern "C" __attribute__((section(".itcm_text"))) void
HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    if (HAL_OK != HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data))
    {
        return;
    }

    if (FDCAN_FRAME_CLASSIC == rx_header.RxFrameType &&
        (FDCAN_STANDARD_ID == rx_header.IdType || FDCAN_EXTENDED_ID == rx_header.IdType))
    {
        pyro::can_drv_t* can_drv = get_can_drv_from_handle(hfdcan);
        if (can_drv)
        {
            const pyro::can_msg_buffer_t::id_type_t type =
                (rx_header.IdType == FDCAN_EXTENDED_ID)
                ? pyro::can_msg_buffer_t::EXTENDED_ID
                : pyro::can_msg_buffer_t::STANDARD_ID;
            can_drv->handle_rx_msg(rx_header.Identifier, data, type);
        }
    }
}
