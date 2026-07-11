#ifndef __PYRO_SONIC_BASE_DRV_H__
#define __PYRO_SONIC_BASE_DRV_H__

#include "pyro_task.h"
#include "pyro_uart_drv.h"

namespace pyro
{

class sonic_base_drv
{
public:
    virtual ~sonic_base_drv();

    void start(); // 启动任务
    uint32_t get_distance() const; // 外部调用接口

protected:
    explicit sonic_base_drv(uart_drv_t *uart, const char* name);

    // 派生类必须实现
    virtual status_t trigger_impl() = 0;
    virtual bool rx_callback_impl(const uint8_t *p_data, uint16_t size) = 0;

    void update_distance(uint32_t dist); // 供子类更新数据

    uart_drv_t *_uart_drv;

    // 加上 volatile，防止编译器将 ISR 中更新的变量缓存在寄存器中
    volatile uint32_t _distance_mm;

private:
    // 内部任务类
    class sensor_task : public task_base_t
    {
    public:
        explicit sensor_task(sonic_base_drv *owner, const char* name)
            : task_base_t(name, 128, 128, priority_t::NORMAL), _owner(owner) {}
    protected:
        status_t init() override { return PYRO_OK; }
        void run_loop() override;
    private:
        sonic_base_drv *_owner;
    };

    sensor_task *_task;
};

}

#endif // __PYRO_SONIC_BASE_DRV_H__