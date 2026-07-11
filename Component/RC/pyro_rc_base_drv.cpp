#include "pyro_rc_base_drv.h"

namespace pyro
{

// ===================================================================
// 私有内部任务类实现 (代理桥接)
// ===================================================================
rc_drv_t::rc_task_t::rc_task_t(rc_drv_t *parent, const char *name)
    : task_base_t(name, 0, 256, task_base_t::priority_t::REALTIME),
      _parent(parent)
{
}


virtual_rc_t& rc_drv_t::read()
{
    return shared_v_rc;
}

status_t rc_drv_t::rc_task_t::init()
{
    return _parent->task_init();//_parent是拥有这个任务对象的类，放在这里就是rc_drv_t
}

void rc_drv_t::rc_task_t::run_loop()
{
    _parent->task_run_loop();
}

// ===================================================================
// 遥控器基类实现
// ===================================================================
rc_drv_t::rc_drv_t(uart_drv_t &uart, const char *task_name,
                   const uint8_t priority_bit, const uint16_t frame_len)
    : _task(this, task_name), // 初始化包含的任务对象
      _rc_uart(uart), _priority_bit(priority_bit), _frame_len(frame_len)
{
    sequence = 0x80;
}

void rc_drv_t::start()
{
    _task.start();
}

void rc_drv_t::stop()
{
    _task.stop();
}

status_t rc_drv_t::task_init()
{
    _rc_msg_buffer = xMessageBufferCreate(_frame_len * 6 + 24);
    if (_rc_msg_buffer == nullptr)
        return PYRO_ERROR;

    _rx_buf = new uint8_t[_frame_len];
    return PYRO_OK;
}

void rc_drv_t::enable()
{
    _rc_uart.add_rx_event_callback(
        [this](const uint8_t *buf, const uint16_t len,
               BaseType_t &xHigherPriorityTaskWoken) -> bool
        { return rc_callback(buf, len, xHigherPriorityTaskWoken); },
        reinterpret_cast<uint32_t>(this));
}

void rc_drv_t::disable()
{
    sequence &= ~(1 << _priority_bit);
    _rc_uart.remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
}

__attribute__((section(".itcm_text")))
bool rc_drv_t::rc_callback(const uint8_t *buf, const uint16_t len,
                           BaseType_t &xHigherPriorityTaskWoken)
{
    if (len == _frame_len && check_packet(buf))
    {
        if (__builtin_ctz(sequence) >= _priority_bit)
        {
            xMessageBufferSendFromISR(_rc_msg_buffer, buf, len,
                                      &xHigherPriorityTaskWoken);
            return true;
        }
    }
    return false;
}

void rc_drv_t::task_run_loop()
{
    while (true)
    {
        if (xMessageBufferReceive(_rc_msg_buffer, _rx_buf, _frame_len,
                                  portMAX_DELAY) == _frame_len)
        {
            sequence |= (1 << _priority_bit);
        }

        while (sequence >> _priority_bit & 0x01)
        {
            const size_t bytes = xMessageBufferReceive(
                _rc_msg_buffer, _rx_buf, _frame_len, pdMS_TO_TICKS(100));
            if (bytes == _frame_len)
            {
                unpack(_rx_buf);
            }
            else if (bytes == 0)
            {
                sequence &= ~(1 << _priority_bit);
            }
        }
    }
}

rw_lock &rc_drv_t::get_lock()
{
    static rw_lock _lock{};
    return _lock;
}

bool rc_drv_t::check_online() const
{
    return sequence >> _priority_bit & 0x01;
}

rc_drv_t::~rc_drv_t()
{
    _task.stop(); // 析构时必须先停止线程，防止跑飞
    if (_rc_msg_buffer)
    {
        vMessageBufferDelete(_rc_msg_buffer);
        _rc_msg_buffer = nullptr;
    }
    delete[] _rx_buf;
}

} // namespace pyro