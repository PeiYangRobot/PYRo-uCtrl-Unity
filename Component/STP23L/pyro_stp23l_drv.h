#ifndef __PYRO_STP23L_DRV_H__
#define __PYRO_STP23L_DRV_H__

#include "pyro_sonic_base_drv.h"

namespace pyro
{

/**
 * @brief LDROBOT STP-23L UART distance sensor driver.
 *
 * STP-23L continuously sends little-endian packets at 230400 baud.  The
 * driver parses packets from the UART DMA stream and exposes the nearest valid
 * point in the packet through sonic_base_drv::get_distance(), in millimetres.
 */
class stp23l_drv final : public sonic_base_drv
{
public:
#ifdef STP23L_DOWN_UART
    static stp23l_drv& down_get_instance();
#endif
#ifdef STP23L_FRONT_UART
    // Second sensor: separate parser buffer and distance from down_get_instance().
    static stp23l_drv& front_get_instance();
#endif

    stp23l_drv(const stp23l_drv&) = delete;
    stp23l_drv& operator=(const stp23l_drv&) = delete;

    void init();

private:
    explicit stp23l_drv(uart_drv_t *uart);
    ~stp23l_drv() override;

    status_t trigger_impl() override;
    bool rx_callback_impl(const uint8_t *p_data, uint16_t size);
    void consume_byte(uint8_t byte);
    void reset_parser();
    void process_frame();

    static constexpr uint8_t PACK_GET_DISTANCE = 0x02U;
    static constexpr uint16_t MAX_DATA_LENGTH = 320U;
    static constexpr uint16_t FRAME_HEADER_LENGTH = 10U;
    static constexpr uint16_t MAX_FRAME_LENGTH =
        FRAME_HEADER_LENGTH + MAX_DATA_LENGTH + 1U;
    static constexpr uint8_t POINT_SIZE = 15U;

    uint8_t _frame[MAX_FRAME_LENGTH]{};
    uint16_t _frame_size{0};
    uint16_t _expected_frame_size{0};
};

} // namespace pyro

#endif // __PYRO_STP23L_DRV_H__
