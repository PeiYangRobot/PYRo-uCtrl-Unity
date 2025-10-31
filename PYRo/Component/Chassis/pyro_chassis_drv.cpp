#include "pyro_chassis_drv.h"
#include "pyro_rc_hub.h"

#include <cmath>

#define Ox 0.17332f
#define Oy 0.16238f
#define Sy 0.17135f
#define Sx 0.1675f


namespace pyro
{

chassis_drv_t::chassis_drv_t(steering_wheel_drv_t *steering_wheel_drv_1,
                             steering_wheel_drv_t *steering_wheel_drv_2,
                             wheel_drv_t *wheel_drv_1,
                             wheel_drv_t *wheel_drv_2)
    : _steering_wheel_drv_1(steering_wheel_drv_1),
      _steering_wheel_drv_2(steering_wheel_drv_2),
      _wheel_drv_1(wheel_drv_1),
      _wheel_drv_2(wheel_drv_2)
{
}

void chassis_drv_t::dr16_cmd()
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_p_ctrl());
    _vy = static_cast<float>(p_ctrl->rc.ch[3]) / 660.0f * 2.0f;
    _vx = static_cast<float>(p_ctrl->rc.ch[2]) / 660.0f * 2.0f;
    _wz = static_cast<float>(p_ctrl->rc.ch[0]) / 660.0f;
    _s_right = static_cast<uint8_t>(p_ctrl->rc.s[0]);
}

void chassis_drv_t::update_feedback()
{
    _steering_wheel_drv_1->update_feedback();
    _steering_wheel_drv_2->update_feedback();
    _wheel_drv_1->update_feedback();
    _wheel_drv_2->update_feedback();
}

void chassis_drv_t::zero_force()
{
    _wheel_drv_1->zero_force();
    _wheel_drv_2->zero_force();
    _steering_wheel_drv_1->zero_force();
    _steering_wheel_drv_2->zero_force();
}

void chassis_drv_t::chassis_control()
{
    if( _s_right == 1 )
    {
        zero_force();
    }
    else {
        // _steering_wheel_drv_1->set_radian(-atan2f(_vy - _wz * 17.0f, _vx + _wz * 17.0f));
        // _steering_wheel_drv_2->set_radian(-atan2f(_vy - _wz * 17.0f, _vx - _wz * 17.0f));

        // float steering_wheel_1_speed = hypot(_vx - _wz * 0.17f, _vy + _wz * 0.17f);
        // float steering_wheel_2_speed = hypot(_vx - _wz * 0.17f, _vy - _wz * 0.17f);

        // _wheel_drv_1->set_speed( (_vx +_vy) * sqrtf(2.0f) / 2.0f + _wz );
        // _wheel_drv_2->set_speed( (-_vx +_vy) * sqrtf(2.0f) / 2.0f + _wz );
        // _steering_wheel_drv_1->wheel_drv->set_speed(-steering_wheel_1_speed);
        // _steering_wheel_drv_2->wheel_drv->set_speed(-steering_wheel_2_speed);

        _steering_wheel_drv_1->set_radian(-atan2f(_vx - _wz * Sx, _vy + _wz * Sy));
        _steering_wheel_drv_2->set_radian(-atan2f(_vx - _wz * Sx, _vy - _wz * Sy));

        float wheel1_speed = ( _vy + _wz * Oy) * cosf( PI/4) + ( _vx + _wz * Ox) * sinf( PI/4);
        float wheel2_speed = (-_vy + _wz * Oy) * cosf(-PI/4) + (-_vx - _wz * Ox) * sinf(-PI/4);

        float steering_wheel_1_speed = hypotf(_vy + _wz * Oy, _vx - _wz * Ox);
        float steering_wheel_2_speed = hypotf(_vy - _wz * Oy, _vx - _wz * Ox);

        _wheel_drv_1->set_speed(wheel1_speed);
        _wheel_drv_2->set_speed(wheel2_speed);

        _steering_wheel_drv_1->wheel_drv->set_speed(-steering_wheel_1_speed);
        _steering_wheel_drv_2->wheel_drv->set_speed(-steering_wheel_2_speed);
    }
}


}
