#ifndef DM_MOTOR_DRV_H
#define DM_MOTOR_DRV_H

#include "pyro_motor_base.h"

namespace pyro
{
class dm_motor_drv_t : public motor_base_t // MIT only
{
  public:
    enum error_code
    {
        ok                    = 0x00,
        over_votlage          = 0x08,
        under_voltage         = 0x09,
        over_temperature      = 0x0a,
        mos_over_temperature  = 0x0b,
        coil_over_temperature = 0x0c,
        communication_lost    = 0x0d,
        over_load             = 0x0e,
    };
    dm_motor_drv_t(uint32_t tx_id, uint32_t rx_id, can_hub_t::which_can which);
    ~dm_motor_drv_t();

    status_t enable() override;
    status_t disable() override;

    status_t update_feedback() override;
    status_t send_torque(float torque) override;

    void set_position_range(float min, float max);
    void set_rotate_range(float min, float max);
    void set_torque_range(float min, float max);

    void set_runtime_kp(float kp);
    void set_runtime_kd(float kd);

  private:
    uint32_t _can_id;
    uint32_t _master_id;

    error_code _error_code;

    float _mos_temperature;
    float _coil_temperature;

    float _min_position;
    float _max_position;
    float _min_rotate;
    float _max_rotate;
    static constexpr float _min_kp = 0.0f;
    static constexpr float _max_kp = 500.0f;
    static constexpr float _min_kd = 0.0f;
    static constexpr float _max_kd = 5.0f;
    float _min_torque;
    float _max_torque;

    float _runtime_kp;
    float _runtime_kd;
};
}; // namespace pyro

#endif