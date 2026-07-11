/**
 * @file pyro_referee.h
 * @brief RoboMaster Referee System Driver (Modern C++ Style)
 */

#ifndef PYRO_REFEREE_H
#define PYRO_REFEREE_H

#include "protocol.h"
#include "pyro_uart_drv.h"
#include "pyro_task.h"
#include <bitset>
#include <initializer_list>
#include "pyro_core_config.h"
#include "fifo.h"
#include <cstdint>

#include "FreeRTOS.h"
#include "pyro_mutex.h"
#include "semphr.h"

namespace pyro
{

class referee_drv_t
{
  public:
    static constexpr uint16_t FIFO_BUF_LEN   = 1024;
    static constexpr size_t MAX_CMD_ID_COUNT = 1024;
    static constexpr size_t MAX_TX_FRAME_LEN = FRAME_MAX_SIZE;

    // 【新增】双缓冲（Ping-Pong）池数量配置
    static constexpr uint8_t TX_BUFFER_NUM   = 2;

#ifdef REFEREE_UART
    static referee_drv_t *get_instance();
#endif

    // 禁止拷贝
    referee_drv_t(const referee_drv_t &)            = delete;
    referee_drv_t &operator=(const referee_drv_t &) = delete;

    /* ================= Init & Config ================= */

    void init(std::initializer_list<cmd_id> listening_ids);
    void init();

    void set_robot_id(const uint16_t id)
    {
        _robot_id = id;
    }
    [[nodiscard]] uint16_t get_robot_id() const
    {
        return _robot_id;
    }
    [[nodiscard]] uint16_t get_client_id() const;

    /* ================= Rx API ================= */

    [[nodiscard]] const referee_data_t &get_data() const
    {
        return _data;
    }
    [[nodiscard]] bool is_online() const
    {
        return _is_online;
    }

    /* ================= Tx API ================= */

    bool send_packet(cmd_id cmd_id_val, const void *data, uint16_t len);

    bool send_robot_interaction(uint16_t receiver_id, uint16_t sub_cmd_id,
                                const void *data, uint16_t len);
    bool send_ui_interaction(uint16_t sub_cmd_id, const void *data);
    bool send_custom_info(const char *message);

  private:
    /* Private Inner Types */
    enum class unpack_step
    {
        HEADER_SOF = 0,
        LENGTH_LOW,
        LENGTH_HIGH,
        FRAME_SEQ,
        HEADER_CRC8,
        DATA_CRC16
    };

    struct unpack_context
    {
        unpack_step step  = unpack_step::HEADER_SOF;
        uint16_t index    = 0;
        uint16_t data_len = 0;
        uint8_t protocol_packet[FRAME_MAX_SIZE];
    };

    /* Private Task */
    class referee_task final : public task_base_t
    {
      public:
        explicit referee_task(referee_drv_t *parent)
            : task_base_t("referee_task", 512, 512, priority_t::NORMAL),
              _parent(parent)
        {
        }

        status_t init() override;
        void run_loop() override;

      private:
        referee_drv_t *_parent;
    };

    /* Private Methods */
    explicit referee_drv_t(uart_drv_t *uart_handle);
    bool rx_callback(uint8_t *p, uint16_t size,
                     BaseType_t task_woken);
    void unpack_fifo_data();

    template <typename T>
    static void safe_copy(T &target, const uint8_t *src,
                          const uint16_t packet_data_len)
    {
        // Safety copy to prevent buffer overflows
        const size_t copy_len =
            packet_data_len < sizeof(T) ? packet_data_len : sizeof(T);
        memcpy(&target, src, copy_len);
    }
    void solve_data(const uint8_t *frame);
    bool _send_interaction_packet_base(uint16_t sub_cmd_id,
                                       uint16_t receiver_id, const void *data,
                                       uint16_t len);

    /* Members */
    uart_drv_t *_uart;
    referee_task *_task;
    fifo_s_t _fifo{};
    uint8_t _fifo_buf[FIFO_BUF_LEN]{};

    referee_data_t _data;
    unpack_context _unpack_obj;

    uint8_t _send_seq;
    uint16_t _robot_id;

    // 【修改】双缓冲 DMA 发送队列
    uint8_t* _tx_buffers[TX_BUFFER_NUM]{};
    uint8_t  _tx_buffer_idx;

    // 【修改】互斥锁与二值信号量
    mutex_t _tx_mutex;
    SemaphoreHandle_t _tx_cplt_sem;

    std::bitset<MAX_CMD_ID_COUNT> _enabled_ids;
    bool _is_online;
    float _last_update_time;
};

} // namespace pyro

#endif // PYRO_REFEREE_H