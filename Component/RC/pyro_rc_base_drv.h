#ifndef __PYRO_RC_BASE_DRV_H__
#define __PYRO_RC_BASE_DRV_H__

#include "pyro_rw_lock.h"
#include "pyro_uart_drv.h"
#include "message_buffer.h"
#include "pyro_virtual_rc.h"
#include "pyro_task.h"

namespace pyro
{

class rc_drv_t
{
  public:


    static void init_virtual_rc()
    {
        shared_v_rc.init_all();
    }

    static virtual_rc_t& read();

    // 暴露给 App 层的生命周期控制接口
    void start();
    void stop();
    void enable();
    void disable();

    [[nodiscard]] bool check_online() const;
    [[nodiscard]] static rw_lock &get_lock();

  protected:
    // 全局唯一虚拟控制器，作为所有驱动映射的终点
    inline static uint8_t sequence         = 0x80;
    inline static virtual_rc_t shared_v_rc = {};
    // 构造函数接收引用
    rc_drv_t(uart_drv_t &uart, const char *task_name, uint8_t priority_bit,
             uint16_t frame_len);
    virtual ~rc_drv_t();

    virtual bool check_packet(const uint8_t *buf) = 0;
    virtual void unpack(const uint8_t *buf)       = 0;

  private:
    // ---------------------------------------------------------
    // 【架构精髓】通过私有内部类实现组合，代理任务的运行逻辑
    // ---------------------------------------------------------
    class rc_task_t : public task_base_t
    {
      public:
        rc_task_t(rc_drv_t *parent, const char *name);

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        rc_drv_t *_parent;
    };

    // 供内部任务调用的实际执行函数
    status_t task_init();
    void task_run_loop();
    bool rc_callback(const uint8_t *buf, uint16_t len,
                     BaseType_t &xHigherPriorityTaskWoken);

    rc_task_t _task; // 包含一个任务实体 (Has-a)
    MessageBufferHandle_t _rc_msg_buffer{};
    uart_drv_t &_rc_uart;
    uint8_t _priority_bit{};
    uint16_t _frame_len{};
    uint8_t *_rx_buf{};
};

} // namespace pyro
#endif