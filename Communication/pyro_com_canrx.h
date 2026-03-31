#ifndef __PYRO_COM_CANRX_H__
#define __PYRO_COM_CANRX_H__

#include "pyro_can_drv.h"
#include <vector>
#include <array>
#include <cstdint>

namespace pyro
{

/**
 * @brief 通用CAN接收驱动类 (Singleton)
 * 允许应用层订阅特定的 (CAN总线, CAN ID)，并轮询获取原始数据。
 */
class can_rx_drv_t
{
public:
    can_rx_drv_t(const can_rx_drv_t &) = delete;
    can_rx_drv_t &operator=(const can_rx_drv_t &) = delete;
    static can_rx_drv_t *instance();

    /**
     * @brief 订阅指定的 CAN ID
     * @param which 指定 CAN 通道 (can1, can2, can3...)
     * @param id 要监听的 CAN ID
     * @return status_t PYRO_OK 成功，PYRO_ERROR 已存在或驱动无效
     */
    static status_t subscribe(can_hub_t::which_can which, uint32_t id);

    /**
     * @brief 获取原始 CAN 数据
     * @param which 指定 CAN 通道
     * @param id 目标 CAN ID
     * @param data [out] 用于存储 8 字节原始数据的容器
     * @return true 获取到新数据
     * @return false 无新数据、未订阅或参数错误
     */
    static bool get_data(can_hub_t::which_can which, uint32_t id, std::array<uint8_t, 8>& data);

    /**
     * @brief 检查指定 CAN 通道上的 ID 是否有新数据
     */
    static bool is_fresh(can_hub_t::which_can which, uint32_t id);

private:
    can_rx_drv_t() = default;
    ~can_rx_drv_t();

    // 禁止拷贝


    struct rx_node_t
    {
        can_hub_t::which_can which; // 【新增】 区分总线通道
        uint32_t id;
        can_msg_buffer_t *msg_buffer;
    };

    /**
     * @brief 内部查找节点，需同时匹配 which 和 id
     */
    static rx_node_t* get_node(can_hub_t::which_can which, uint32_t id);

    std::vector<rx_node_t> _rx_nodes{};
};

}

#endif // __PYRO_COM_CANRX_H__