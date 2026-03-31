/**
 * @file pyro_task.h
 * @brief Base class for FreeRTOS task management.
 * FreeRTOS 任务管理基类。
 */

#ifndef __PYRO_TASK_H__
#define __PYRO_TASK_H__

#include "FreeRTOS.h"
#include "pyro_core_def.h"
#include "task.h"
#include <cstdint>

namespace pyro
{

/**
 * @brief Abstract base class for managing FreeRTOS tasks.
 * 管理 FreeRTOS 任务的抽象基类。
 */
class task_base_t
{
  public:
    /**
     * @brief Task priority levels.
     * 任务优先级等级。
     */
    enum class priority_t : uint8_t
    {
        IDLE,         /**< Idle (0). 空闲级。 */
        LOW,          /**< Low (1). 低优先级。 */
        BELOW_NORMAL, /**< Below Normal (2). 次正常级。 */
        NORMAL,       /**< Normal (3). 正常级。 */
        ABOVE_NORMAL, /**< Above Normal (4). 高正常级。 */
        HIGH,         /**< High (5). 高优先级。 */
        REALTIME,     /**< Real-time (6). 实时级。 */
    };

    /**
     * @brief Constructor for task_base_t.
     * task_base_t 构造函数。
     *
     * @param name Task name. 任务名称。
     * @param init_stack Init stack size. 初始化栈大小。
     * @param loop_stack Loop stack size. 循环栈大小。
     * @param priority Priority level. 优先级。
     */
    task_base_t(const char *name, uint16_t init_stack, uint16_t loop_stack,
                priority_t priority);

    /** @brief Virtual destructor. 虚析构函数。 */
    virtual ~task_base_t();

    /** @brief Starts the task. 启动任务。 */
    status_t start();

    /** @brief Stops the task. 停止任务。 */
    void stop();

  protected:
    /** @brief Initialization callback. 初始化回调。 */
    virtual status_t init()     = 0;

    /** @brief Main loop callback. 主循环回调。 */
    virtual void run_loop() = 0;

    /** @brief Task handle. 任务句柄。 */
    TaskHandle_t _loop_task_handle;

  private:
    const char *_task_name;
    uint16_t _loop_stack_depth;
    priority_t _priority;

    /** @brief Entry point for init. 初始化入口。 */
    static status_t init_entry_point(void *arg);

    /** @brief Entry point for loop. 循环入口。 */
    static void loop_entry_point(void *arg);

    /** @brief Priority converter. 优先级转换器。 */
    static UBaseType_t convert_priority(priority_t p);
};

} // namespace pyro
#endif