/**
 * @version 1.1.0
 */

#ifndef CAN_DRV_H
#define CAN_DRV_H

#include "fdcan.h"
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include <array>
#include <cmsis_os.h>

#include "map.h"

namespace pyro
{
class can_msg_buffer_t
{
public:
    enum id_type_t
    {
        STANDARD_ID = 0,
        EXTENDED_ID = 1
    };

    explicit can_msg_buffer_t(uint32_t id, id_type_t type = STANDARD_ID);
    explicit can_msg_buffer_t(uint32_t id, uint32_t mask, id_type_t type = STANDARD_ID); // 带掩码的构造函数
    ~can_msg_buffer_t();

    [[nodiscard]] uint32_t get_id() const;
    [[nodiscard]] uint32_t get_mask() const;
    [[nodiscard]] bool matches(uint32_t received_id) const; // 检查ID是否匹配
    [[nodiscard]] id_type_t get_id_type() const;
    [[nodiscard]] bool is_fresh() const;
    void mark_read();
    void update_data(const uint8_t *data, uint32_t received_id);
    [[nodiscard]] bool get_data(std::array<uint8_t, 8> &data) const;
    [[nodiscard]] bool get_data(std::array<uint8_t, 8> &data, uint32_t &received_id) const;
    [[nodiscard]] TickType_t get_last_update_time() const;
    [[nodiscard]] uint32_t get_received_id() const;

private:
    uint32_t _id;
    uint32_t _mask;    // 掩码：1表示必须匹配，0表示忽略
    bool _use_mask;    // 是否使用掩码过滤
    id_type_t _id_type;
    std::array<uint8_t, 8> _buffer;
    volatile bool _is_fresh;
    volatile TickType_t _last_update_time;
    volatile uint32_t _received_id;
};

class can_drv_t
{
    const uint8_t MAX_ID_REGIST_NUM = 32;
    using can_id_regist_t           = uint16_t;
    using register_key_t = std::pair<uint32_t, can_msg_buffer_t::id_type_t>;

public:
    explicit can_drv_t(FDCAN_HandleTypeDef *hfdcan);
    ~can_drv_t();

    status_t init();
    status_t start() const;
    status_t send_msg(uint32_t id, const uint8_t *data,
                      can_msg_buffer_t::id_type_t type = can_msg_buffer_t::STANDARD_ID) const;
    status_t register_rx_msg(can_msg_buffer_t *msg_buffer);
    status_t handle_rx_msg(uint32_t id, const uint8_t *data,
                           can_msg_buffer_t::id_type_t type);

private:
    FDCAN_HandleTypeDef *_hfdcan;
    map_t<register_key_t, can_msg_buffer_t *> _registerlist;

    // 专门存储使用掩码的消息缓冲区
    can_msg_buffer_t *_mask_buffers[10];
    int _mask_buffer_count;
};

}

#endif