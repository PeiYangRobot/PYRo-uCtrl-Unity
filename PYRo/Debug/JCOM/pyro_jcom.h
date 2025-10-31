#ifndef __PYRO_JCOM_H__
#define __PYRO_JCOM_H__

#include "pyro_uart_drv.h"
#include "vector"

namespace pyro
{
class jcom_drv_t
{
public:
    explicit jcom_drv_t(uint8_t max_length, uart_drv_t *uart);
    ~jcom_drv_t();
    static jcom_drv_t &get_instance(uint8_t max_length);
    void thread();

private:
    void init();
    void add_data(float *data);
    void add_data(float *data, uint8_t len);
    void remove_data(const float *data);
    void update_data();
    void send();

    typedef struct
    {
        float *data;
        uint8_t size;
    } data_node_t;

    std::vector<data_node_t> _data_nodes;
    float *_data_pack;
    uint8_t _length;
    uart_drv_t *_jcom_uart;
};


}

#endif