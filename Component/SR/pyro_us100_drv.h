#ifndef __PYRO_US100_DRV_H__
#define __PYRO_US100_DRV_H__

#include "pyro_sonic_base_drv.h"

namespace pyro
{

class us100_drv final : public sonic_base_drv
{
public:
    /**
     * @brief 获取单例引用
     */
#ifdef US100_UART
    static us100_drv& get_instance();
#endif

    // 禁用拷贝构造和赋值操作，确保单例唯一性
    us100_drv(const us100_drv&) = delete;
    us100_drv& operator=(const us100_drv&) = delete;

    void init(); // 注册串口回调

private:
    explicit us100_drv(uart_drv_t *uart);
    ~us100_drv() override;

    status_t trigger_impl() override;
    bool rx_callback_impl(const uint8_t *p_data, uint16_t size) override;

    uint8_t *_tx_buf; // DMA 发送缓冲
};

} // namespace pyro

#endif // __PYRO_US100_DRV_H__