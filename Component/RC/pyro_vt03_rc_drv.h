#ifndef __PYRO_VT03_RC_DRV_H__
#define __PYRO_VT03_RC_DRV_H__

#include "pyro_rc_base_drv.h"
#include "pyro_core_config.h"



namespace pyro {

class vt03_drv_t final : public rc_drv_t {
public:
#ifdef VT03_UART
    static vt03_drv_t& instance();
#endif
    vt03_drv_t(const vt03_drv_t&) = delete;
    vt03_drv_t& operator=(const vt03_drv_t&) = delete;

private:
    explicit vt03_drv_t(uart_drv_t& vt03_uart);

    typedef struct __packed {
        uint8_t sof1;
        uint8_t sof2;
        uint64_t ch0     : 11;
        uint64_t ch1     : 11;
        uint64_t ch2     : 11;
        uint64_t ch3     : 11;
        uint64_t gear    : 2;
        uint64_t pause   : 1;
        uint64_t fn_l    : 1;
        uint64_t fn_r    : 1;
        uint64_t wheel   : 11;
        uint64_t trigger : 1;
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t press_l : 2;
        uint8_t press_r : 2;
        uint8_t press_m : 2;
        uint16_t key_code;
        uint16_t crc;
    } vt03_buf_t;

    bool check_packet(const uint8_t *buf) override;
    void unpack(const uint8_t *buf) override;

    static status_t error_check(const vt03_buf_t *vt03_buf);
};

}
#endif
