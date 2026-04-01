#include "pyro_com_canrx.h"

namespace pyro
{

can_rx_drv_t::~can_rx_drv_t()
{
    for (auto &node : _rx_nodes)
    {
        if (node.msg_buffer)
        {
            delete node.msg_buffer;
            node.msg_buffer = nullptr;
        }
    }
    delete this;
}

can_rx_drv_t *can_rx_drv_t::instance()
{
    static can_rx_drv_t instance;
    return &instance;
}

// 【修改】 查找逻辑增加 which 匹配
can_rx_drv_t::rx_node_t *
can_rx_drv_t::get_node(const can_hub_t::which_can which, const uint32_t id)
{
    for (auto &node : instance()->_rx_nodes)
    {
        if (node.id == id && node.which == which)
            return &node;
    }
    return nullptr;
}

status_t can_rx_drv_t::subscribe(const can_hub_t::which_can which,
                                 const uint32_t id)
{
    // 【修改】 检查重复时带上 which
    if (get_node(which, id) != nullptr)
    {
        return PYRO_ERROR; // 避免重复订阅
    }

    can_hub_t *can_hub = can_hub_t::get_instance();
    if (can_hub == nullptr)
        return PYRO_ERROR;

    can_drv_t *driver = can_hub->hub_get_can_obj(which);
    if (driver == nullptr)
        return PYRO_ERROR;

    // 申请 buffer
    auto *new_buffer = new can_msg_buffer_t(id);
    if (new_buffer == nullptr)
        return PYRO_NO_MEMORY;

    // 注册到底层驱动
    const status_t reg_status = driver->register_rx_msg(new_buffer);
    if (reg_status != PYRO_OK)
    {
        delete new_buffer;
        return reg_status;
    }

    // 保存节点信息
    rx_node_t new_node{};
    new_node.which      = which; // 【新增】 记录通道
    new_node.id         = id;
    new_node.msg_buffer = new_buffer;
    instance()->_rx_nodes.push_back(new_node);

    return PYRO_OK;
}

bool can_rx_drv_t::get_data(const can_hub_t::which_can which, const uint32_t id,
                            std::array<uint8_t, 8> &data)
{
    // 【修改】 使用 (which, id) 查找
    const rx_node_t *node = get_node(which, id);

    if (!node || !node->msg_buffer)
        return false;

    if (!node->msg_buffer->is_fresh())
        return false;

    if (node->msg_buffer->get_data(data))
    {
        node->msg_buffer->mark_read();
        return true;
    }

    return false;
}

bool can_rx_drv_t::is_fresh(const can_hub_t::which_can which, const uint32_t id)
{
    const rx_node_t *node = get_node(which, id);
    if (node && node->msg_buffer)
    {
        return node->msg_buffer->is_fresh();
    }
    return false;
}

} // namespace pyro