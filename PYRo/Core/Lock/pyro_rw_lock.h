#ifndef __PYRO_RW_LOCK_H__
#define __PYRO_RW_LOCK_H__

#include "freertos.h"
#include "semphr.h"
#include "task.h"

namespace pyro
{

/**
 * @brief 基于FreeRTOS的C++读写锁（写优先）
 *
 * 该类实现了一个“写优先”的读写锁，并支持基于Tick的超时。
 * - 允许多个读线程同时访问。
 * - 只允许一个写线程访问，且读写互斥。
 * - “写优先”：当一个写线程请求锁时，
 * 任何新的读线程将被阻塞，直到所有等待的写线程完成。
 */
class rw_lock
{
  public:
    rw_lock();
    ~rw_lock();

    // 禁用拷贝构造和拷贝赋值
    rw_lock(const rw_lock &)            = delete;
    rw_lock &operator=(const rw_lock &) = delete;

    // ----------------------------------------------------------------
    // 阻塞式 (无限等待) API
    // ----------------------------------------------------------------

    /**
     * @brief 请求读锁 (无限期等待)
     */
    void read_lock();

    /**
     * @brief 释放读锁
     */
    void read_unlock();

    /**
     * @brief 请求写锁 (无限期等待)
     */
    void write_lock();

    /**
     * @brief 释放写锁
     */
    void write_unlock();


    // ----------------------------------------------------------------
    // 带超时的 API
    // ----------------------------------------------------------------

    /**
     * @brief 尝试请求读锁，带超时
     * @param timeout_ticks 等待的 FreeRTOS Tick 数量
     * @return true 如果成功获取锁, false 如果超时
     */
    bool read_lock(TickType_t timeout_ticks);

    /**
     * @brief 尝试请求写锁，带超时
     * @param timeout_ticks 等待的 FreeRTOS Tick 数量
     * @return true 如果成功获取锁, false 如果超时
     */
    bool write_lock(TickType_t timeout_ticks);


  private:
    SemaphoreHandle_t _internal_mutex;  // 用于保护内部计数器的互斥锁
    SemaphoreHandle_t _read_gate;       // 读者“大门”信号量
    SemaphoreHandle_t _write_gate;      // 写者“大门”信号量
    volatile int _reader_count;         // 当前正在读取的读者数量
    volatile int _writer_waiting_count; // 正在等待的写者数量
};


// ----------------------------------------------------------------------------
// RAII (Scoped Lock) 辅助类 (已更新)
// ----------------------------------------------------------------------------

/**
 * @brief RAII辅助类，用于自动获取和释放“读锁”
 */
class read_scope_lock
{
  public:
    /**
     * @brief 构造函数 (无限等待)
     */
    explicit read_scope_lock(rw_lock &lock) : _lock(lock), _is_locked(true)
    {
        _lock.read_lock();
    }

    /**
     * @brief 构造函数 (带超时, Ticks)
     */
    read_scope_lock(rw_lock &lock, const TickType_t timeout_ticks) : _lock(lock)
    {
        _is_locked = _lock.read_lock(timeout_ticks);
    }

    /**
     * @brief 析构函数，如果锁已获取，则释放
     */
    ~read_scope_lock()
    {
        if (_is_locked)
        {
            _lock.read_unlock();
        }
    }

    /**
     * @brief 检查锁是否成功获取
     * @return true 如果锁被持有, false 如果超时
     */
    bool is_locked() const
    {
        return _is_locked;
    }

    // 禁用拷贝
    read_scope_lock(const read_scope_lock &)            = delete;
    read_scope_lock &operator=(const read_scope_lock &) = delete;

  private:
    rw_lock &_lock;
    bool _is_locked;
};

/**
 * @brief RAII辅助类，用于自动获取和释放“写锁”
 */
class write_scope_lock
{
  public:
    /**
     * @brief 构造函数 (无限等待)
     */
    explicit write_scope_lock(rw_lock &lock) : _lock(lock), _is_locked(true)
    {
        _lock.write_lock();
    }

    /**
     * @brief 构造函数 (带超时, Ticks)
     */
    write_scope_lock(rw_lock &lock, const TickType_t timeout_ticks)
        : _lock(lock)
    {
        _is_locked = _lock.write_lock(timeout_ticks);
    }

    /**
     * @brief 析构函数，如果锁已获取，则释放
     */
    ~write_scope_lock()
    {
        if (_is_locked)
        {
            _lock.write_unlock();
        }
    }

    /**
     * @brief 检查锁是否成功获取
     * @return true 如果锁被持有, false 如果超时
     */
    bool is_locked() const
    {
        return _is_locked;
    }

    // 禁用拷贝
    write_scope_lock(const write_scope_lock &)            = delete;
    write_scope_lock &operator=(const write_scope_lock &) = delete;

  private:
    rw_lock &_lock;
    bool _is_locked;
};

} // namespace pyro
#endif // __PYRO_RW_LOCK_H__