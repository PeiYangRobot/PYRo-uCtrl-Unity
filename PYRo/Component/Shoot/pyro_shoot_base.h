#ifndef __PYRO_SHOOT_BASE_H__
#define __PYRO_SHOOT_BASE_H__ 

#include "pyro_trigger_drv.h"
#include "pyro_fric_drv.h"
#include "pyro_rc_hub.h"

namespace pyro
{ 
class shoot_base_t
{
public:
    enum total_mode_t
    {
        ZERO_FORCE         = 0x00,
        RC_CONTROL         = 0x01,
        AUTO_AIM_CONTROL   = 0x02
    };
    enum local_mode_t
    {
        SHOOT_STOP         = 0x00,
        SHOOT_SETUP        = 0x01,
        SHOOT_READY        = 0x02,
        SHOOT_START        = 0x03,
        SHOOT_WAIT         = 0x04,
        SHOOT_CONTINUOUS   = 0x05,
        SHOOT_CALIBRATION   = 0x06
    };
    enum ready_mode_t
    {
        SHOOT_READY_STOP         = 0x00,
        SHOOT_READY_SETUP        = 0x01,
        SHOOT_READY_READY        = 0x02,
        SHOOT_READY_START        = 0x03,
        SHOOT_READY_WAIT         = 0x04,
        SHOOT_READY_CONTINUOUS   = 0x05
    };
    shoot_base_t();
    ~shoot_base_t()
    {
    }
    
    void set_continuous_mode_delay(uint16_t delay);
    void dr16_cmd(void const *rc_ctrl);
    void vt03_cmd();
    virtual void set_fric_speed(float target_speed);
    virtual void update_feedback();
    virtual void zero_force();
    virtual void control();

protected:
    bool _trigger_lock = false;
    total_mode_t _total_mode;
    local_mode_t _local_mode;
    local_mode_t _last_local_mode;
    ready_mode_t _ready_mode;


private:
    uint64_t _continuous_mode_delay = HAL_MAX_DELAY;
    uint16_t _continuous_delay = 0;
};

}

#endif

