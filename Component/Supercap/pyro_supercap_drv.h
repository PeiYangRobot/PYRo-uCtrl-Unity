/**
 * @file pyro_supercap_drv.h
 * @brief Supercapacitor Driver (Main Controller Side)
 *
 * Implements communication with the Supercap module using:
 * 1. Composition pattern (Driver HAS-A Task)
 * 2. FreeRTOS Message Buffer (ISR to Task data passing)
 * 3. DMA (Transmission)
 *
 * @author Lucky (AI Assistant)
 * @date 2026-02-03
 */

#ifndef __PYRO_SUPERCAP_DRV_H__
#define __PYRO_SUPERCAP_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_task.h"
#include "pyro_uart_drv.h"
#include "pyro_core_def.h"
#include "message_buffer.h"

namespace pyro
{

class supercap_drv_t
{
  public:
/* Public Nested Types (User Data Payload) -------------------------------*/
#pragma pack(push, 1)

    /**
     * @brief [Main -> Cap] Command Data (8 Bytes)
     */
    struct chassis_cmd_t
    {
        uint16_t power_referee;             // 底盘功率 (裁判系统)
        uint8_t power_limit_referee;        // 底盘功率上限
        uint8_t power_buffer_referee;       // 底盘缓冲功率
        uint8_t power_buffer_limit_referee; // 底盘缓冲功率上限
        uint8_t use_cap;                    // 1: 使用电容, 0: 不使用
        uint8_t kill_chassis_user;          // 1: 自杀/断电
        uint8_t speed_up_user_now;          // 保留/加速标志
    };

    /**
     * @brief [Cap -> Main] Feedback Data (9 Bytes)
     */
    struct cap_feedback_t
    {
        uint16_t cap_power_cap;     // 电容充放电功率
        uint16_t chassis_power_cap; // ADC测得底盘功率
        uint16_t vot_cap;           // 电容电压
        uint8_t error_flag;         // 错误标志
        uint8_t cap_low_flag;       // 电容没电标志
        uint8_t over_normal_c_l;    // 电流到达额定值
    };

#pragma pack(pop)
#ifdef SUPERCAP_UART
    /* Public Methods --------------------------------------------------------*/
    static supercap_drv_t * get_instance();
#endif
    /**
     * @brief Starts the internal task and enables communication.
     */
    void start_rx() const;

    /**
     * @brief Sends command data via DMA.
     */
    status_t send_cmd(const chassis_cmd_t &cmd) const; // NOLINT

    /**
     * @brief Gets the latest feedback.
     */
    [[nodiscard]] const cap_feedback_t &get_feedback() const;

    /**
     * @brief Checks connection status.
     */
    [[nodiscard]] bool check_online() const;

  private:
    /**
     * @brief Constructor.
     * @param uart_handle Pointer to the existing PYRO UART driver instance.
     */
    explicit supercap_drv_t(uart_drv_t *uart_handle);

    /**
     * @brief Destructor. Stops task and frees resources.
     */
    ~supercap_drv_t();
    /* Private Task Implementation (Composition) -----------------------------*/
    class supercap_task_t final : public task_base_t
    {
      public:
        // Task runs at BELOW_NORMAL priority as it handles non-critical
        // telemetry decoding
        explicit supercap_task_t(supercap_drv_t *owner_ptr)
            : task_base_t("supercap_task", 128, 128, priority_t::BELOW_NORMAL),
              _owner(owner_ptr)
        {
        }

      protected:
        // Delegate FreeRTOS hooks back to the owner driver
        status_t init() override;
        void run_loop() override;

      private:
        supercap_drv_t *_owner;
    };

/* Private Protocol Types ------------------------------------------------*/
#pragma pack(push, 1)

    struct frame_header_t
    {
        uint8_t sof; // 0x55
        uint8_t crc8;
    };

    struct frame_tailer_t
    {
        uint16_t crc16;
    };

    struct tx_packet_t
    {
        frame_header_t header;
        chassis_cmd_t data;
        frame_tailer_t tailer;
        uint8_t end_byte;
    };

    struct rx_packet_t
    {
        frame_header_t header;
        cap_feedback_t data;
        frame_tailer_t tailer;
    };

#pragma pack(pop)

    /* Private Members -------------------------------------------------------*/
    uart_drv_t *_uart_drv;
    supercap_task_t *_task;  // The internal task instance
    tx_packet_t *_tx_buffer; // DMA buffer
    MessageBufferHandle_t _rx_msg_buf;

    cap_feedback_t _latest_feedback{};
    bool _is_online;

    static constexpr uint8_t FRAME_SOF = 0x55;

    /* Private Methods (Logic) -----------------------------------------------*/
    // Actual implementation of initialization and loop logic
    void init_impl();
    void run_loop_impl();

    bool rx_callback(const uint8_t *p_data, uint16_t size,
                     BaseType_t& xHigherPriorityTaskWoken) const;

    static status_t error_check(const rx_packet_t *buf);
    void unpack(const rx_packet_t *buf);
};

} // namespace pyro

#endif // __PYRO_SUPERCAP_DRV_H__