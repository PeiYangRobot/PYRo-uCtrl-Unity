/**
 * @file pyro_module_base.tpp
 * @brief Implementation of the PYRO Module Base Template.
 * PYRO 模块基类模板实现文件。
 *
 * This file contains the implementation of the `pyro::module_base_t` template
 * class. It handles the low-level details of command double-buffering,
 * thread-safe mutex locking, and the internal task loop execution logic.
 * 本文件包含了 `pyro::module_base_t` 模板类的实现。它处理命令环形缓冲区
 * 线程安全互斥锁以及内部任务循环执行逻辑的底层细节。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-01-28
 */

#pragma once

namespace pyro
{

template <typename Derived, typename ModuleParams>
module_base_t<Derived, ModuleParams>::module_base_t(
    const char *name, uint16_t init_stack, uint16_t loop_stack,
    task_base_t::priority_t priority)
    : _task(this, name, init_stack, loop_stack, priority)
{
}

template <typename Derived, typename ModuleParams>
status_t module_base_t<Derived, ModuleParams>::start()
{
    return _task.start();
}

/**
 * @brief 写入命令到环形缓冲区 (生产者)
 * 注意：如果缓冲区已满，新命令将被丢弃（防止覆盖未执行的旧轨迹）
 */
template <typename Derived, typename ModuleParams>
bool module_base_t<Derived, ModuleParams>::set_command(
    const typename module_base_t<Derived, ModuleParams>::CmdType &cmd)
{
    scoped_mutex_t lock(_mutex);
    uint8_t next_head = (_head + 1) % CMD_BUF_SIZE;
    if (next_head != _tail)
    {
        _cmd_buffer[_head] = cmd;
        _head              = next_head;
        return true;
    }
    return false;
}

/**
 * @brief 设置模块配置 (在 start 前调用)
 * 注意：配置数据通常在模块启动前设置，且不应频繁更改。
 */
template <typename Derived, typename ModuleParams>
void module_base_t<Derived, ModuleParams>::configure(
    const typename module_base_t<Derived, ModuleParams>::ModuleDeps &deps)
{
    _module_deps = deps;
}


/**
 * @brief 从环形缓冲区更新命令 (消费者)
 * 注意：如果缓冲区为空，_current_cmd 保持上一次的值不变 (Zero-Order Hold)
 */
template <typename Derived, typename ModuleParams>
void module_base_t<Derived, ModuleParams>::_update_command()
{
    scoped_mutex_t lock(_mutex);
    if (_head != _tail)
    {
        _current_cmd = _cmd_buffer[_tail];
        _tail        = (_tail + 1) % CMD_BUF_SIZE;
    }
}

template <typename Derived, typename ModuleParams>
mutex_t &module_base_t<Derived, ModuleParams>::get_mutex()
{
    return _mutex;
}

template <typename Derived, typename ModuleParams>
const typename module_base_t<Derived, ModuleParams>::ModuleCtx &
module_base_t<Derived, ModuleParams>::get_ctx() const
{
    return _ctx;
}

/**
 * @brief Core loop invoking virtual callbacks.
 * 调用虚函数回调的核心循环。
 */
template <typename Derived, typename ModuleParams>
void module_base_t<Derived, ModuleParams>::_run_loop_impl()
{
    TickType_t xLastWakeTime        = xTaskGetTickCount();
    constexpr TickType_t xFrequency = pdMS_TO_TICKS(1);

    while (true)
    {
        _update_command();
        _update_feedback();
        _fsm_execute();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* Internal Task Proxy Implementations --------------------------------------*/

template <typename Derived, typename ModuleParams>
module_base_t<Derived, ModuleParams>::module_task_t::module_task_t(
    module_base_t *owner_ptr, const char *name, const uint16_t init_stack,
    const uint16_t loop_stack, const task_base_t::priority_t priority)
    : task_base_t(name, init_stack, loop_stack, priority), _owner(owner_ptr)
{
}

/**
 * @brief Invokes the initialization function of the module instance.
 * 调用模块实例的初始化函数。
 */
template <typename Derived, typename ModuleParams>
status_t module_base_t<Derived, ModuleParams>::module_task_t::init()
{
    if (_owner)
        return _owner->_init();
    return status_t::PYRO_ERROR;
}

/**
 * @brief Invokes the core loop implementation of the module instance.
 * 调用模块实例的核心循环实现。
 */
template <typename Derived, typename ModuleParams>
void module_base_t<Derived, ModuleParams>::module_task_t::run_loop()
{
    if (_owner)
        _owner->_run_loop_impl();
}

} // namespace pyro
