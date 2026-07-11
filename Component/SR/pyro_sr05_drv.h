#ifndef __PYRO_SR05_DRV_H__
#define __PYRO_SR05_DRV_H__

#include "pyro_sonic_base_drv.h"

namespace pyro
{

class sr05_drv final : public sonic_base_drv
{
public:
    /**
     * @brief 获取单例引用
     */
#ifdef SR05_UART
    static sr05_drv& get_instance();
#endif

    // 禁用拷贝构造和赋值操作，确保单例唯一性
    sr05_drv(const sr05_drv&) = delete;
    sr05_drv& operator=(const sr05_drv&) = delete;

    void init(); // 注册串口回调

private:
    explicit sr05_drv(uart_drv_t *uart);
    ~sr05_drv() override;

    status_t trigger_impl() override;
    bool rx_callback_impl(const uint8_t *p_data, uint16_t size) override;
};

} // namespace pyro

#endif // __PYRO_SR05_DRV_H__