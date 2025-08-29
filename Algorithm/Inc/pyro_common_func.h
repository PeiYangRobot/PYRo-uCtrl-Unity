#ifndef __PYRO_COMMON_FUNC_H__
#define __PYRO_COMMON_FUNC_H__

#include <stdint.h>

namespace pyro 
{

template <typename T>
uint8_t get_bit_pos_l(T data);

template <typename T>
uint8_t get_bit_pos_h(T data);

}    
#endif
