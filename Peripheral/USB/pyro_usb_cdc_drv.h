#ifndef __PYRO_USB_CDC_DRV_H__
#define __PYRO_USB_CDC_DRV_H__

#include "pyro_core_def.h"
#include "pyro_task.h"
#include "pyro_mutex.h"
#include "usb_cdc_config.h"
#include <functional>
#include <vector>
#include <utility>

namespace pyro {

class usb_cdc_drv_t {
    /**
     * @brief RX event callback signature.
     * 接收事件回调函数签名。
     *
     * @param p Pointer to the received data buffer.
     * 接收数据缓冲区的指针。
     * @param size Size of the received data.
     * 接收数据的大小。
     * @return true if the data is consumed and buffer should switch.
     * 如果数据已被处理，则返回 true。
     */
    using rx_event_handler = std::function<bool(const uint8_t *p, uint16_t size)>;

  public:
    static usb_cdc_drv_t& instance();
    usb_cdc_drv_t(const usb_cdc_drv_t&) = delete;
    usb_cdc_drv_t& operator=(const usb_cdc_drv_t&) = delete;

    status_t init(const char *serial);
    status_t reset();

    status_t enable_rx();
    status_t disable_rx();
    status_t register_rx(uint8_t frame_id, const rx_event_handler &handler);
    status_t remove_rx(uint8_t frame_id);

    /**
     * @brief 无发送完成回调的发送函数
     * @param frame_id 发送帧ID
     * @param data 指向发送数据的指针
     * @param len 发送数据的长度
     */
    status_t send(uint8_t frame_id, const uint8_t *data, uint16_t len);

    [[nodiscard]] bool is_connected();

    void on_cdc_rx();
    void on_mount();
    void on_umount();

  private:
    /**
     * @brief 帧头结构体（5 字节）。
     *
     * 线上布局：
     * @code
     *   偏移    0      1          2        3        4
     *         +------+----------+--------+--------+------+
     *         | SOF  | FRAME_ID | LEN_LO | LEN_HI | CRC8 |
     *         +------+----------+--------+--------+------+
     * @endcode
     *
     * @par 端序约定（重要）
     * `data_len` 是 `uint16_t`，它在**线上**的字节序等于 MCU 在内存里的字节序，
     * 因为结构体是 `packed`，`memcpy` 到缓冲区就是原样拷贝内存。
     *
     * 本项目 MCU 为 STM32H723（ARM Cortex-M7）：
     *  - Cortex-M 系列**只支持小端**（`SCB->AIRCR.ENDIANNESS` 恒为 0）；
     *  - GCC 在 ARM 上遵循 AAPCS，`packed` 结构体无填充、字段按声明顺序紧密排列；
     */
    struct frame_header_t {
        uint8_t  sof;
        uint8_t  frame_id;
        uint16_t data_len;
        uint8_t  crc8;
    } __attribute__((packed));

    class usb_task_t final : public task_base_t {
      public:
        explicit usb_task_t(usb_cdc_drv_t *owner) :
            task_base_t("usb_task", 256, 512, priority_t::ABOVE_NORMAL), _owner(owner) {}

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        usb_cdc_drv_t *_owner;
    };

    usb_cdc_drv_t() = default;
    ~usb_cdc_drv_t();

    void drain_tx();
    void process_rx();
    void dispatch(uint8_t frame_id, const uint8_t *data, uint16_t len);

    uint8_t _itf{0};
    usb_task_t *_task;
    std::vector<std::pair<uint8_t, rx_event_handler>> _rx_handlers;
    uint8_t _rx_buf[usb_cdc::RX_BUF_CAPACITY];
    uint16_t _rx_tail{0};
    uint8_t _tx_buf[usb_cdc::TX_BUF_CAPACITY];
    uint16_t _tx_tail{0};
    mutex_t  _tx_mutex;

    bool _init{false};
    bool _rx_enable{false};
};

}

#endif