#ifndef __PYRO_SR04_DRV_H__
#define __PYRO_SR04_DRV_H__

#include "pyro_sonic_base_drv.h"

namespace pyro
{

class sr04_drv final : public sonic_base_drv
{
public:
    /**
     * @brief 获取单例引用
     */
#ifdef SR04_UART
    static sr04_drv& get_instance();
#endif


    // 禁用拷贝构造和赋值操作
    sr04_drv(const sr04_drv&) = delete;
    sr04_drv& operator=(const sr04_drv&) = delete;

    void init();

private:
    explicit sr04_drv(uart_drv_t *uart);
    ~sr04_drv() override;

    status_t trigger_impl() override;
    bool rx_callback_impl(const uint8_t *p_data, uint16_t size) override;

    uint8_t *_tx_buf;
};

} // namespace pyro

#endif // __PYRO_SR04_DRV_H__