#ifndef __PYRO_COM_CANTX_H__
#define __PYRO_COM_CANTX_H__

#include "pyro_can_drv.h"
#include <vector>
#include <cstdint>

namespace pyro
{

class can_tx_drv_t
{
public:
    static can_tx_drv_t *instance();

    static void clear(uint32_t id);

    template <typename T>
    static void add_data(const uint32_t id, const uint8_t bit_len, T data)
    {
        add_data_raw(id, bit_len, &data);
    }

    // 【修改点1】 增加 driver 参数，不再依赖内部成员
    static void send(uint32_t id, can_drv_t *driver);

private:
    can_tx_drv_t();
    ~can_tx_drv_t();

    static void add_data_raw(uint32_t id, uint8_t bit_len, const void *data);

    struct frame_node_t
    {
        uint32_t id;
        uint8_t current_bit_usage;
        uint8_t buffer[8];
    };

    frame_node_t* get_frame(uint32_t id);

    std::vector<frame_node_t> _frames{};

    // 【修改点2】 删除 _can_driver
    // can_drv_t *_can_driver;
};

}

#endif