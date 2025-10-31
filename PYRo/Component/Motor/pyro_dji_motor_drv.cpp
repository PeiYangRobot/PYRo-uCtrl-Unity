#include "pyro_dji_motor_drv.h"

#include <cmath>
#include <cstring>

namespace pyro
{
dji_motor_tx_frame_t::dji_motor_tx_frame_t(can_hub_t::which_can which,
                                                 uint32_t id)
    : _key(id, which)
{
    _can = can_hub_t::get_instance()->hub_get_can_obj(_key.second);
    _register_list.fill(0);
    _update_list.fill(0);
    _value_list.fill(0);
}

dji_motor_tx_frame_t::~dji_motor_tx_frame_t()
{
}

status_t dji_motor_tx_frame_t::register_id(dji_motor_tx_frame_t::register_id_t id)
{
    if (_register_list[id % 4] != 0)
        return PYRO_ERROR;
    _register_list[id % 4] = 1;
    return PYRO_OK;
}

const dji_motor_tx_frame_t::_frame_key_t &
dji_motor_tx_frame_t::get_key(void)
{
    return _key;
}

status_t dji_motor_tx_frame_t::update_value(uint8_t id,int16_t value)
{
    if (_register_list[id % 4] == 0)
        return PYRO_ERROR;
    _value_list[id % 4]  = value;
    _update_list[id % 4] = 1;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (_register_list[i])
        {
            if (!_update_list[i])
                return PYRO_ERROR;
        }
    }
    std::array<uint8_t, 8> data;
    data.fill(0);
    for (uint8_t i = 0; i < 4; i++)
    {
        _update_list[i] = 0 ;
        data[i * 2]     = (_value_list[i] & 0xff00) >> 8;
        data[i * 2 + 1] = _value_list[i] & 0xff;
    }
    _can->send_msg(_key.first, data.data());
    return PYRO_OK;
}

dji_motor_tx_frame_pool_t::dji_motor_tx_frame_pool_t(void)
{
    _frame_list.clear();
}
dji_motor_tx_frame_pool_t *dji_motor_tx_frame_pool_t::_instancePtr = nullptr;

dji_motor_tx_frame_pool_t * dji_motor_tx_frame_pool_t::get_instance(void)
{
    if (_instancePtr == nullptr) // Mutex
    {
        _instancePtr = new dji_motor_tx_frame_pool_t();
    }
    return _instancePtr;
}

dji_motor_tx_frame_t *
dji_motor_tx_frame_pool_t::get_frame(can_hub_t::which_can which, uint32_t id)
{
    dji_motor_tx_frame_t *frame = nullptr;
    dji_motor_tx_frame_t::_frame_key_t key(id, which);
    int length = _frame_list.size();
    for (uint8_t i = 0; i < length; i++)
    {
        if (_frame_list[i]->get_key() == key)
        {
            frame = _frame_list[i];
            break;
        }
    }
    if (frame == nullptr)
    {
        frame = new dji_motor_tx_frame_t(which, id);
        _frame_list.push_back(frame);
    }
    return frame;
}


dji_motor_drv_t::dji_motor_drv_t(
    dji_motor_tx_frame_t::register_id_t id,
    can_hub_t::which_can which)
    : motor_base_t(which), _register_id(id)
{
}

status_t pyro::dji_motor_drv_t::enable()
{
    _enable = true;
    return PYRO_OK;
}

status_t dji_motor_drv_t::disable()
{
    _enable = false;
    return PYRO_OK;
}

status_t dji_motor_drv_t::update_feedback()
{
    static std::array<uint8_t, 8> data;
    _feedback_msg->get_data(data);

    _current_position = ((float)((uint16_t)((data[0] << 8) | (data[1])))) /
                        8192.0f * 2 * PI;
    if(_current_position > PI)
    {
        _current_position -= 2 * PI;
    }
    _current_rotate =
        ((float)((int16_t)((data[2] << 8) | (data[3])))) * 2 * pyro::PI / 60;
    _current_torque = ((float)((int16_t)((data[4] << 8) | (data[5])))) /
                      _max_torque_i * _max_torque_f;
    _temperature = (int8_t)(data[6]);

    return PYRO_OK;
}

static float constraint(float value, float max)
    {
        if(value > max)
            return max;
        if(value < -max)
            return -max;
        return value;
    }

status_t dji_motor_drv_t::send_torque(float torque)
{
    static std::array<uint8_t, 8> data;
    data.fill(0);
    torque=constraint(torque,_max_torque_f);
    int16_t torque_i = (int16_t)(torque / _max_torque_f * _max_torque_i);
    _tx_frame->update_value(_register_id, torque_i);
    return PYRO_OK;
}

dji_m3508_motor_drv_t::dji_m3508_motor_drv_t(
    dji_motor_tx_frame_t::register_id_t id, can_hub_t::which_can which)
    : dji_motor_drv_t(id, which)
{
    switch (id)
    {
        case dji_motor_tx_frame_t::id_1:
        case dji_motor_tx_frame_t::id_2:
        case dji_motor_tx_frame_t::id_3:
        case dji_motor_tx_frame_t::id_4:
            _tx_id = 0x200;
            _rx_id = 0x200 + id + 1;
            break;
        case dji_motor_tx_frame_t::id_5:
        case dji_motor_tx_frame_t::id_6:
        case dji_motor_tx_frame_t::id_7:
        case dji_motor_tx_frame_t::id_8:
            _tx_id = 0x1FF;
            _rx_id = 0x200 + id + 1;
            break;
        default:
            _init_status = pyro::PYRO_ERROR;
            break;
    }
    _feedback_msg = new can_msg_buffer_t(_rx_id);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_feedback_msg);
    }

    _tx_frame =
        dji_motor_tx_frame_pool_t::get_instance()->get_frame(which, _tx_id);
    _tx_frame->register_id(_register_id);
    _max_torque_f = 20.0f;
    _max_torque_i = 16384;
}

dji_m2006_motor_drv_t::dji_m2006_motor_drv_t(
    dji_motor_tx_frame_t::register_id_t id, can_hub_t::which_can which)
    : dji_motor_drv_t(id, which)
{
    switch (id)
    {
        case dji_motor_tx_frame_t::id_1:
        case dji_motor_tx_frame_t::id_2:
        case dji_motor_tx_frame_t::id_3:
        case dji_motor_tx_frame_t::id_4:
            _tx_id = 0x200;
            _rx_id = 0x200 + id + 1;
            break;
        case dji_motor_tx_frame_t::id_5:
        case dji_motor_tx_frame_t::id_6:
        case dji_motor_tx_frame_t::id_7:
        case dji_motor_tx_frame_t::id_8:
            _tx_id = 0x1FF;
            _rx_id = 0x200 + id + 1;
            break;
        default:
            _init_status = PYRO_ERROR;
            break;
    }
    _feedback_msg = new can_msg_buffer_t(_rx_id);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_feedback_msg);
    }

    _tx_frame =
        dji_motor_tx_frame_pool_t::get_instance()->get_frame(which, _tx_id);
    _tx_frame->register_id(_register_id);
    _max_torque_f = 10.0f;
    _max_torque_i = 10000;
}

dji_gm_6020_motor_drv_t::dji_gm_6020_motor_drv_t(
    dji_motor_tx_frame_t::register_id_t id, can_hub_t::which_can which)
    : dji_motor_drv_t(id, which)
{
    switch (id)
    {
        case dji_motor_tx_frame_t::id_1:
        case dji_motor_tx_frame_t::id_2:
        case dji_motor_tx_frame_t::id_3:
        case dji_motor_tx_frame_t::id_4:
            _tx_id = 0x1ff;
            _rx_id = 0x204 + id + 1;
            break;
        case dji_motor_tx_frame_t::id_5:
        case dji_motor_tx_frame_t::id_6:
        case dji_motor_tx_frame_t::id_7:
            // case dji_motor_tx_frame_t::id_8: for gm6020 id8 is unavailable
            _tx_id = 0x2fe;
            _rx_id = 0x204 + id + 1;
            break;
        default:
            _init_status = pyro::PYRO_ERROR;
            break;
    }

    _feedback_msg = new can_msg_buffer_t(_rx_id);
    if (_can_drv)
    {
        _can_drv->register_rx_msg(_feedback_msg);
    }

    _tx_frame =
        dji_motor_tx_frame_pool_t::get_instance()->get_frame(which, _tx_id);
    _tx_frame->register_id(_register_id);
    _max_torque_f = 3.0f;
    _max_torque_i = 16384;
}

}