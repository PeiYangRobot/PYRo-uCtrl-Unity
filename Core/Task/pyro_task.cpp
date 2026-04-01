/**
 * @file pyro_task.cpp
 * @brief Implementation of the task management base.
 * 任务管理基类的实现。
 */

#include "pyro_task.h"

namespace pyro
{

/**
 * @brief Constructor for task_base_t.
 * task_base_t 构造函数。
 */
task_base_t::task_base_t(const char *name, const uint16_t init_stack,
                         const uint16_t loop_stack, const priority_t priority)
    : _loop_task_handle(nullptr), _task_name(name),
      _loop_stack_depth(loop_stack), _priority(priority)
{
}

/**
 * @brief Destructor ensuring task stop.
 * 确保任务停止的析构函数。
 */
task_base_t::~task_base_t()
{
    stop();
}

/**
 * @brief Spawns the initial setup task.
 * 生成初始设置任务。
 */
status_t task_base_t::start()
{
    if (_loop_task_handle != nullptr)
    {
        return PYRO_ERROR;
    }

    const status_t ret = init_entry_point(this);

    return ret;
}

/**
 * @brief Deletes the loop task if it exists.
 * 如果循环任务存在则将其删除。
 */
void task_base_t::stop()
{
    if (_loop_task_handle != nullptr)
    {
        vTaskDelete(_loop_task_handle);
        _loop_task_handle = nullptr;
    }
}

/**
 * @brief Internal entry for the initialization phase.
 * 初始化阶段的内部入口。
 */
status_t task_base_t::init_entry_point(void *arg)
{
    auto *self = static_cast<task_base_t *>(arg);

    if (self)
    {
        const status_t init_ret = self->init();
        if (init_ret != PYRO_OK)
        {
            return init_ret;
        }

        if (self->_loop_stack_depth > 0)
        {
            const BaseType_t ret = xTaskCreate(
                loop_entry_point, self->_task_name, self->_loop_stack_depth,
                self, convert_priority(self->_priority),
                &self->_loop_task_handle);
            if (ret == pdPASS)
            {
                return PYRO_OK;
            }
        }
    }
    return status_t::PYRO_OK;
}

/**
 * @brief Internal entry for the continuous loop phase.
 * 持续循环阶段的内部入口。
 */
void task_base_t::loop_entry_point(void *arg)
{
    auto *self = static_cast<task_base_t *>(arg);
    if (self)
    {
        self->run_loop();

        self->_loop_task_handle = nullptr;
        vTaskDelete(nullptr);
    }
    vTaskDelete(nullptr);
}

/**
 * @brief Converts abstract priority to FreeRTOS scale.
 * 将抽象优先级转换为 FreeRTOS 刻度。
 *
 * @details
 * Formula: (p * (max - 1)) / 6.
 * 公式：(p * (max - 1)) / 6。
 */
UBaseType_t task_base_t::convert_priority(priority_t p)
{
    return static_cast<UBaseType_t>(p) * (configMAX_PRIORITIES - 1) / 6;
}

} // namespace pyro