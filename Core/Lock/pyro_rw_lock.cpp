#include "pyro_rw_lock.h"

namespace pyro
{

rw_lock::rw_lock() : _reader_count(0), _writer_waiting_count(0)
{
    _internal_mutex = xSemaphoreCreateMutex();
    _read_gate      = xSemaphoreCreateBinary();
    _write_gate     = xSemaphoreCreateBinary();

    configASSERT(_internal_mutex != nullptr);
    configASSERT(_read_gate != nullptr);
    configASSERT(_write_gate != nullptr);

    // 初始化信号量为“可用”状态
    xSemaphoreGive(_read_gate);
    xSemaphoreGive(_write_gate);
}

rw_lock::~rw_lock()
{
    vSemaphoreDelete(_internal_mutex);
    vSemaphoreDelete(_read_gate);
    vSemaphoreDelete(_write_gate);
}

// ----------------------------------------------------------------
// 阻塞式 (无限等待) API - 已修正
// ----------------------------------------------------------------

void rw_lock::read_lock()
{
    // 1. 等待 "读者大门" (写优先)
    xSemaphoreTake(_read_gate, portMAX_DELAY);

    // 2. 保护计数器 (极短的临界区)
    xSemaphoreTake(_internal_mutex, portMAX_DELAY);
    _reader_count++;
    bool first_reader = (_reader_count == 1);
    xSemaphoreGive(_internal_mutex); // 立即释放

    // 3. 如果是第一个读者，获取 "写者大门" 以阻止写者
    //    (这个等待现在发生在 _internal_mutex 之外)
    if (first_reader)
    {
        xSemaphoreTake(_write_gate, portMAX_DELAY);
    }

    // 4. 归还 "读者大门"，允许其他读者进入
    xSemaphoreGive(_read_gate);
}

void rw_lock::read_unlock()
{
    // (此函数逻辑正确，临界区已很短)
    xSemaphoreTake(_internal_mutex, portMAX_DELAY);
    _reader_count--;
    if (_reader_count == 0)
    {
        // 最后一个读者释放 "写者大门"
        xSemaphoreGive(_write_gate);
    }
    xSemaphoreGive(_internal_mutex);
}

void rw_lock::write_lock()
{
    bool first_writer = false;

    // 1. 保护计数器 (极短的临界区)
    xSemaphoreTake(_internal_mutex, portMAX_DELAY);
    _writer_waiting_count++;
    if (_writer_waiting_count == 1)
    {
        first_writer = true;
    }
    xSemaphoreGive(_internal_mutex); // 立即释放

    // 2. 如果是第一个等待的写者，拿走 "读者大门"
    //    (这个等待现在发生在 _internal_mutex 之外)
    if (first_writer)
    {
        xSemaphoreTake(_read_gate, portMAX_DELAY);
    }

    // 3. 等待获取 "写者大门" (独占访问)
    xSemaphoreTake(_write_gate, portMAX_DELAY);
}

void rw_lock::write_unlock()
{
    // (此函数逻辑正确，临界区已很短)
    xSemaphoreGive(_write_gate); // 释放 "写者大门"

    xSemaphoreTake(_internal_mutex, portMAX_DELAY);
    _writer_waiting_count--;
    if (_writer_waiting_count == 0)
    {
        // 最后一个写者归还 "读者大门"
        xSemaphoreGive(_read_gate);
    }
    xSemaphoreGive(_internal_mutex);
}


// ----------------------------------------------------------------
// 带超时的 API - 已修正
// ----------------------------------------------------------------

bool rw_lock::read_lock(TickType_t timeout_ticks)
{
    TickType_t start_time     = xTaskGetTickCount();
    TickType_t remaining_time = timeout_ticks;
    TickType_t elapsed_time   = 0;
    bool first_reader         = false;

    // 1. 尝试获取 "读者大门"
    if (xSemaphoreTake(_read_gate, remaining_time) == pdFALSE)
    {
        return false; // 超时
    }

    // 2. 保护计数器 (极短的临界区)
    xSemaphoreTake(_internal_mutex, portMAX_DELAY);
    _reader_count++;
    if (_reader_count == 1)
    {
        first_reader = true;
    }
    xSemaphoreGive(_internal_mutex); // 立即释放

    // 3. 如果是第一个读者，尝试获取 "写者大门"
    if (first_reader)
    {
        // 计算剩余时间
        elapsed_time = xTaskGetTickCount() - start_time;
        if (elapsed_time >= timeout_ticks)
        {
            remaining_time = 0; // 时间已用完
        }
        else
        {
            remaining_time = timeout_ticks - elapsed_time;
        }

        if (xSemaphoreTake(_write_gate, remaining_time) == pdFALSE)
        {
            // 超时，必须 "撤销"
            xSemaphoreTake(_internal_mutex, portMAX_DELAY);
            _reader_count--; // 撤销计数
            xSemaphoreGive(_internal_mutex);

            xSemaphoreGive(_read_gate); // 归还大门
            return false;
        }
    }

    // 4. 归还 "读者大门"
    xSemaphoreGive(_read_gate);
    return true; // 成功
}


bool rw_lock::write_lock(TickType_t timeout_ticks)
{
    const TickType_t start_time = xTaskGetTickCount();
    TickType_t remaining_time   = timeout_ticks;
    TickType_t elapsed_time     = 0;
    bool first_writer           = false;

    // 1. 保护计数器 (极短的临界区)

    if (xSemaphoreTake(_internal_mutex, portMAX_DELAY) == pdFALSE)
    {
        // 理论上不应发生，但如果互斥锁无效，则失败
        return false;
    }

    _writer_waiting_count++;
    if (_writer_waiting_count == 1)
    {
        first_writer = true;
    }
    xSemaphoreGive(_internal_mutex); // 立即释放

    // 2. 如果是第一个写者，尝试拿走 "读者大门"
    if (first_writer)
    {
        // 计算剩余时间
        elapsed_time = xTaskGetTickCount() - start_time;
        if (elapsed_time >= timeout_ticks)
        {
            remaining_time = 0; // 时间已用完
        }
        else
        {
            remaining_time = timeout_ticks - elapsed_time;
        }

        if (xSemaphoreTake(_read_gate, remaining_time) == pdFALSE)
        {
            // 超时，"撤销"
            xSemaphoreTake(_internal_mutex, portMAX_DELAY);
            _writer_waiting_count--; // 撤销计数
            xSemaphoreGive(_internal_mutex);
            return false;
        }
    }

    // 3. 尝试获取 "写者大门" (独占访问)
    elapsed_time = xTaskGetTickCount() - start_time;
    if (elapsed_time >= timeout_ticks)
    {
        remaining_time = 0; // 时间已用完
    }
    else
    {
        remaining_time = timeout_ticks - elapsed_time;
    }

    if (xSemaphoreTake(_write_gate, remaining_time) == pdFALSE)
    {
        // 超时，必须执行 "撤销"
        xSemaphoreTake(_internal_mutex, portMAX_DELAY);
        _writer_waiting_count--; // 减少等待计数

        // 如果我们是第一个写者，并且现在没有其他写者在等待
        // 我们必须归还 "读者大门"
        if (first_writer && _writer_waiting_count == 0)
        {
            xSemaphoreGive(_read_gate);
        }
        xSemaphoreGive(_internal_mutex);
        return false;
    }

    // 成功获取写锁
    return true;
}

} // namespace pyro