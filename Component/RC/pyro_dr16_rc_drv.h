#ifndef __PYRO_DR16_RC_DRV_H__
#define __PYRO_DR16_RC_DRV_H__

#include "pyro_rc_base_drv.h"
#include "pyro_core_config.h"


namespace pyro
{

class dr16_drv_t final : public rc_drv_t
{
  public:
#ifdef DR16_UART
    static dr16_drv_t &instance();
#endif
    dr16_drv_t(const dr16_drv_t &)            = delete;
    dr16_drv_t &operator=(const dr16_drv_t &) = delete;

  private:
    explicit dr16_drv_t(uart_drv_t &dr16_uart);

    typedef struct __packed
    {
        uint32_t ch0 : 11;
        uint32_t ch1 : 11;
        uint32_t ch2 : 11;
        uint32_t ch3 : 11;
        uint32_t s1  : 2;
        uint32_t s2  : 2;
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t press_l;
        uint8_t press_r;
        uint16_t key_code;
        uint16_t wheel;
    } dr16_buf_t;

    bool check_packet(const uint8_t *buf) override;
    void unpack(const uint8_t *buf) override;

    static status_t error_check(const dr16_buf_t *dr16_buf);
};

} // namespace pyro
#endif
