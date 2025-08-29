#include "pyro_common_func.h"

namespace pyro
{

    
template <typename T>
uint8_t get_bit_pos_l(T data)
{
    uint8_t pos = 0;
    if(0 == (uint32_t)data)
    {
        return 255;
    }
    while ((data & 1) == 0) 
    {
        data >>= 1;
        pos++;
    }
    return pos;
}

template <typename T>
uint8_t get_bit_pos_h(T data)
{
    uint8_t pos = 0;
    uint8_t total_bits;

    total_bits = sizeof(T) * 8;
    pos = total_bits - 1;

    if(0 == (uint32_t)data)
    {
        return 255;
    }

    while ((data & (1 << (total_bits - 1)))== 0) {
        data <<= 1;
        pos--;
    }
    return pos;
}
}





