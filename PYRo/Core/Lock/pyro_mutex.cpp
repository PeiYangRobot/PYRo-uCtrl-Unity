#include "pyro_mutex.h"

namespace pyro
{

// ----------------------------------------------------------------
// mutex_t 实现
// ----------------------------------------------------------------

mutex_t::mutex_t()
{
    _handle = xSemaphoreCreateMutex();
    // 确保创建成功，类似 rw_lock 中的处理
    configASSERT(_handle != nullptr);
}

mutex_t::~mutex_t()
{
    if (_handle != nullptr)
    {
        vSemaphoreDelete(_handle);
        _handle = nullptr;
    }
}

bool mutex_t::lock(const TickType_t timeout_ticks) const
{
    // 安全检查：防止句柄为空
    if (_handle == nullptr)
    {
        return false;
    }
    return (xSemaphoreTake(_handle, timeout_ticks) == pdTRUE);
}

bool mutex_t::unlock() const
{
    if (_handle == nullptr)
    {
        return false;
    }
    return (xSemaphoreGive(_handle) == pdTRUE);
}

SemaphoreHandle_t mutex_t::native_handle() const
{
    return _handle;
}

// ----------------------------------------------------------------
// scoped_lock_t 实现
// ----------------------------------------------------------------

// 构造函数：无限等待
scoped_mutex_t::scoped_mutex_t(mutex_t &mutex) : _mutex(mutex), _is_locked(false)
{
    // 默认使用 portMAX_DELAY
    _is_locked = _mutex.lock(portMAX_DELAY);
}

// 构造函数：带超时
scoped_mutex_t::scoped_mutex_t(mutex_t &mutex, const TickType_t timeout_ticks)
    : _mutex(mutex), _is_locked(false)
{
    // 使用用户指定的超时时间尝试上锁
    _is_locked = _mutex.lock(timeout_ticks);
}

// 析构函数：自动解锁
scoped_mutex_t::~scoped_mutex_t()
{
    // 仅当确实持有锁时才释放，仿照 scope_lock 逻辑
    if (_is_locked)
    {
        if (_mutex.unlock())
        {
            _is_locked = false;
        }
    }
}

bool scoped_mutex_t::is_locked() const
{
    return _is_locked;
}

} // namespace pyro