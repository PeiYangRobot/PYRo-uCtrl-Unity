#ifndef __PYRO_MUTEX_H__
#define __PYRO_MUTEX_H__


#include "FreeRTOS.h"
#include "semphr.h"

namespace pyro {
/**
 * @brief 基于 FreeRTOS 的互斥锁封装
 */
class mutex_t {
    friend class scoped_mutex_t;
public:
    mutex_t();
    ~mutex_t();

    // 禁用拷贝
    mutex_t(const mutex_t&) = delete;
    mutex_t& operator=(const mutex_t&) = delete;
private:
    /**
     * @brief 获取锁
     * @param timeout_ticks 等待时间，默认为永久等待
     * @return true 获取成功, false 超时或失败
     */
    [[nodiscard]] bool lock(TickType_t timeout_ticks = portMAX_DELAY) const;

    /**
     * @brief 释放锁
     * @return true 释放成功, false 失败
     */
    [[nodiscard]] bool unlock() const;

    /**
     * @brief 获取原始 FreeRTOS 句柄
     */
    [[nodiscard]] SemaphoreHandle_t native_handle() const;


    SemaphoreHandle_t _handle;
};

/**
 * @brief RAII 模式的锁守卫 (仿照 read_scope_lock 风格)
 */
class scoped_mutex_t {
public:
    /**
     * @brief 构造函数 (无限等待)
     * 阻塞直到获取锁
     */
    explicit scoped_mutex_t(mutex_t& mutex);

    /**
     * @brief 构造函数 (带超时)
     * @param mutex 互斥锁引用
     * @param timeout_ticks 超时时间
     */
    scoped_mutex_t(mutex_t& mutex, TickType_t timeout_ticks);

    /**
     * @brief 析构函数
     * 如果持有锁，则自动释放
     */
    ~scoped_mutex_t();

    /**
     * @brief 检查是否成功持有锁
     * (用于带超时的构造函数后检查结果)
     */
    [[nodiscard]] bool is_locked() const;

    // 禁用拷贝
    scoped_mutex_t(const scoped_mutex_t&) = delete;
    scoped_mutex_t& operator=(const scoped_mutex_t&) = delete;

private:
    mutex_t& _mutex;
    bool _is_locked;
};

} // namespace pyro



#endif