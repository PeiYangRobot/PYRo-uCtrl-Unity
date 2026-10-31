/**
 * @version 1.1.0
 */

#include "pyro_motor_base.h"

namespace pyro
{
motor_base_t::motor_base_t(bsp_can::which_can which)
    : _which_can(which), _enable(false), _temperature(0), _current_position(0),
      _current_rotate(0), _current_torque(0)
{
    _can_drv = bsp_can::get_can(which);
}

motor_base_t::~motor_base_t()
{
}

int8_t motor_base_t::get_temperature(void)
{
    return _temperature;
}

float motor_base_t::get_current_position(void)
{
    return _current_position;
}

float motor_base_t::get_current_rotate(void)
{
    return _current_rotate;
}

float motor_base_t::get_current_torque(void)
{
    return _current_torque;
}

bool motor_base_t::is_enable(void)
{
    return _enable;
}

bool motor_base_t::is_online(void)
{
    return _online;
}

// 默认实现：什么都不做
// 大疆电机等固定范围的电机可以使用默认实现
void motor_base_t::set_position_range(float min, float max)
{
    // 默认空实现
    (void)min;
    (void)max;
}

void motor_base_t::set_rotate_range(float min, float max)
{
    // 默认空实现
    (void)min;
    (void)max;
}

void motor_base_t::set_torque_range(float min, float max)
{
    // 默认空实现
    (void)min;
    (void)max;
}

} // namespace pyro
