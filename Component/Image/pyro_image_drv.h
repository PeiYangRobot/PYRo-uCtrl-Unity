/**
 * @file pyro_image_drv.h
 * @brief RoboMaster 图传链路驱动 (轻量级订阅机制，零拷贝 DMA 发送)
 */

#ifndef __PYRO_IMAGE_DRV_H__
#define __PYRO_IMAGE_DRV_H__

#include "protocol.h"
#include "pyro_uart_drv.h"
#include "pyro_task.h"
#include "message_buffer.h"
#include <initializer_list>

namespace pyro
{

class image_drv_t
{
  public:
    static constexpr size_t MSG_BUF_SIZE        = 1024;
    static constexpr size_t MAX_FRAME_LEN       = 350;
    static constexpr uint8_t MAX_SUBSCRIBE_NUM  = 4; // 图传链路指令极少，4 个配额绰绰有余

#pragma pack(push, 1)
    /**
     * @brief 机器人 -> 自定义控制器 数据负载 (0x0309)
     */
    struct tx_controller_payload_t { uint8_t data[30]; };

    /**
     * @brief 机器人 -> 自定义客户端 数据负载 (0x0310)
     */
    struct tx_client_payload_t { uint8_t data[300]; };

    /**
     * @brief 零拷贝完整数据包定义：自定义控制器
     */
    struct tx_controller_packet_t
    {
        frame_header_t header;
        uint16_t cmd_id;
        tx_controller_payload_t payload;
        uint16_t crc16;
    };

    /**
     * @brief 零拷贝完整数据包定义：自定义客户端
     */
    struct tx_client_packet_t
    {
        frame_header_t header;
        uint16_t cmd_id;
        tx_client_payload_t payload;
        uint16_t crc16;
    };
#pragma pack(pop)

#ifdef IMAGE_UART
    static image_drv_t &get_instance();
#endif

    // 禁止拷贝
    image_drv_t(const image_drv_t &)            = delete;
    image_drv_t &operator=(const image_drv_t &) = delete;

    /* ================= Init & Config ================= */

    /**
     * @brief 初始化并指定订阅的命令 ID
     * @param listening_ids 订阅列表，例如 {cmd_id::CUSTOM_CONTROLLER, (cmd_id)0x0311}
     */
    void init(std::initializer_list<cmd_id> listening_ids);

    /**
     * @brief 默认初始化，自动订阅 0x0302 (自定义控制器) 和 0x0311 (自定义客户端指令)
     */
    void init();

    /**
     * @brief 启动图传接收和处理任务
     */
    void start() const;

    /* ================= Rx API ================= */

    [[nodiscard]] bool is_online() const { return _is_online; }

    /**
     * @brief 获取自定义控制器接收到的原始数据 (0x0302)
     */
    [[nodiscard]] const uint8_t* get_controller_rx_data() const { return _controller_rx_buf; }

    /**
     * @brief 获取自定义客户端接收到的指令数据 (0x0311)
     */
    [[nodiscard]] const uint8_t* get_client_rx_cmd_data() const { return _client_rx_buf; }

    /* ================= Tx API (零拷贝自瞄风格) ================= */

    /**
     * @brief 获取控制器发送数据的引用，可直接进行赋值操作 (内存位于 DMA 区)
     */
    tx_controller_payload_t &get_controller_tx_data()
    {
        return _tx_controller_pkt->payload;
    }

    /**
     * @brief 获取自定义客户端发送数据的引用，可直接进行赋值操作 (内存位于 DMA 区)
     */
    tx_client_payload_t &get_client_tx_data()
    {
        return _tx_client_pkt->payload;
    }

    /**
     * @brief 补全帧头与校验，并触发 DMA 发送当前的控制器数据 (0x0309)
     */
    status_t send_controller_data();

    /**
     * @brief 补全帧头与校验，并触发 DMA 发送当前的客户端数据 (0x0310)
     */
    status_t send_client_data();

  private:
    explicit image_drv_t(uart_drv_t *uart_handle);
    ~image_drv_t();

    /* Private Task */
    class image_task_t final : public task_base_t
    {
      public:
        explicit image_task_t(image_drv_t *owner)
            : task_base_t("image_task", 256, 512, priority_t::NORMAL), _owner(owner) {}
      protected:
        status_t init() override { return PYRO_OK; }
        void run_loop() override { if (_owner) _owner->task_loop(); }
      private:
        image_drv_t *_owner;
    };

    /* Logic Methods */
    bool rx_callback(const uint8_t *p, uint16_t size, BaseType_t &task_woken) const;
    void task_loop();
    void solve_frame(const uint8_t *frame, uint16_t full_len);

    /* Members */
    uart_drv_t *_uart;
    image_task_t *_task;
    MessageBufferHandle_t _rx_msg_buf;

    // Tx DMA 整包缓冲区指针
    tx_controller_packet_t *_tx_controller_pkt;
    tx_client_packet_t     *_tx_client_pkt;

    // 轻量级白名单列表
    uint16_t _subscribed_ids[MAX_SUBSCRIBE_NUM]{};
    uint8_t  _subscribed_count;

    bool _is_online;
    float _last_update_time;
    uint8_t _send_seq;

    // Rx 缓存
    uint8_t _controller_rx_buf[30]{};
    uint8_t _client_rx_buf[30]{};
};

} // namespace pyro

#endif // __PYRO_IMAGE_DRV_H__