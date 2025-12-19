#include "pyro_dm_motor_drv.h"

namespace pyro
{
dm_motor_drv_t::dm_motor_drv_t(uint32_t can_id, uint32_t master_id,
                                     can_hub_t::which_can which)
    : motor_base_t(which)
{
    _master_id     = master_id;
    _can_id        = can_id;
    _feedback_msg = new can_msg_buffer_t(_master_id);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_feedback_msg);
    }
}

dm_motor_drv_t::~dm_motor_drv_t()
{
}

status_t pyro::dm_motor_drv_t::enable()
{
    std::array<uint8_t, 8> data;
    data.fill(0xFF);
    data[7] = 0xfc;
    _enable = true;
    if(PYRO_OK!=_can_drv->send_msg(_can_id, data.data()))
        return PYRO_ERROR;
    return PYRO_OK;
}

status_t dm_motor_drv_t::disable()
{
    std::array<uint8_t, 8> data;
    data.fill(0xFF);
    data[7] = 0xfc;
    _enable = false;
    if(PYRO_OK!=_can_drv->send_msg(_can_id, data.data()))
        return PYRO_ERROR;
    return PYRO_OK;
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

status_t pyro::dm_motor_drv_t::update_feedback()
{
    std::array<uint8_t, 8> data;
    _feedback_msg->get_data(data);
    _error_code = static_cast<error_code>(((data[0]>>4)&0x0f));
    uint16_t position = ((uint16_t)((data[1] << 8) | (data[2])));
    uint16_t rotate   = ((uint16_t)((data[3] << 4) | ((data[4] >> 4) & 0x0f)));
    uint16_t torque =
        ((uint16_t)(((data[4] << 8) & 0x0f00) | (data[5] & 0xff)));
    _current_position =uint_to_float(position, _min_position, _max_position, 16);
    _current_rotate = uint_to_float(rotate, _min_rotate, _max_rotate, 12);
    _current_torque = uint_to_float(torque, _min_torque, _max_torque, 12);
    return PYRO_OK;
}


status_t pyro::dm_motor_drv_t::send_torque(float torque)
{
    uint16_t torque_int, position_int, rotate_int, kp_int, kd_int;
    std::array<uint8_t, 8> data;
    data.fill(0);
    position_int = float_to_uint(0.0f, _min_position, _max_position, 16);
    rotate_int   = float_to_uint(0.0f, _min_rotate, _max_rotate, 12);
    torque_int   = float_to_uint(torque, _min_torque, _max_torque, 12);
    kp_int       = float_to_uint(_runtime_kp, _min_kp, _max_kp, 12);
    kd_int       = float_to_uint(_runtime_kd, _min_kd, _max_kd, 12);

    data[0]      = (position_int >> 8);
    data[1]      = position_int & 0xff;
    data[2]      = (rotate_int >> 4);
    data[3]      = ((rotate_int & 0x0f) << 4) | (kp_int >> 8);
    data[4]      = kp_int;
    data[5]      = kd_int >> 4;
    data[6]      = ((kd_int & 0x0f) << 4) | (torque_int >> 8);
    data[7]      = torque_int;

    if(PYRO_OK!=_can_drv->send_msg(_can_id, data.data()))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

void dm_motor_drv_t::set_position_range(float min, float max)
{
    _min_position = min;
    _max_position = max;
}

void dm_motor_drv_t::set_rotate_range(float min, float max)
{
    _min_rotate = min;
    _max_rotate = max;
}

void dm_motor_drv_t::set_torque_range(float min, float max)
{
    _min_torque = min;
    _max_torque = max;
}

void dm_motor_drv_t::set_runtime_kp(float kp)
{
    _runtime_kp = kp;
}

void dm_motor_drv_t::set_runtime_kd(float kd)
{
    _runtime_kd = kd;
}

};