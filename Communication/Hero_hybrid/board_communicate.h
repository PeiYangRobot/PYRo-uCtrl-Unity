#ifndef __PYRO_BOARD_COMMUNICATE_H__
#define __PYRO_BOARD_COMMUNICATE_H__

#include "cstdint"

namespace pyro
{

struct chassis2gimbal_data_t
{
    int16_t quaternion[4]; // 4个 int16_t 分别对应四元数
};

inline chassis2gimbal_data_t chassis2gimbal_data = {};

class chassis2gimbal_tx
{

};

}

#endif
