#include "pyro_sonic_base_drv.h"

namespace pyro
{

sonic_base_drv::sonic_base_drv(uart_drv_t *uart, const char* name)
    : _uart_drv(uart), _distance_mm(0)
{
    // 任务实例化
    _task = new sensor_task(this, name);
}

sonic_base_drv::~sonic_base_drv()
{
    if (_task) 
    { 
        _task->stop(); 
        delete _task; 
        _task = nullptr;
    }
}

void sonic_base_drv::start()
{
    if (_task) 
    {
        _task->start();
    }
}

uint32_t sonic_base_drv::get_distance() const
{
    // 32 位系统中，32 位对齐的数据读取是单指令的原子的
    return _distance_mm;
}

void sonic_base_drv::update_distance(uint32_t dist)
{
    // 32 位系统中，32 位对齐的数据写入是单指令的原子的
    _distance_mm = dist;
}

void sonic_base_drv::sensor_task::run_loop()
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        _owner->trigger_impl();
        // 1ms 周期触发
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}

} // namespace pyro