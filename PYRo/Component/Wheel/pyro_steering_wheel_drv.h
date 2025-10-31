// #ifndef __PYRO_STEERING_WHEEL_DRV_H__
// #define __PYRO_STEERING_WHEEL_DRV_H__

// #include "pyro_wheel_drv.h"

// namespace pyro
// {
// class pyro_steering_wheel_drv_t : public pyro_wheel_drv_t
// {
//   public:
//     pyro_steering_wheel_drv_t(pyro::dji_motor_tx_frame_t::register_id_t id,
//                      can_hub_t::which_can which,
//                      pyro::dji_motor_tx_frame_t::register_id_t id,
//                      can_hub_t::which_can which,
//                      float radius,
//                      const pid_ctrl_t &speed_pid,
//                      rc_drv_t *rc_drv)
//     : pyro_wheel_drv_t(id, which, radius, speed_pid, rc_drv);
//     ~pyro_steering_wheel_drv_t()
//     {
//     }

//   private:
//     pyro::dji_motor_tx_frame_t::register_id_t _rudder_id;
//     can_hub_t::which_can _rudder_which;
//     pid_ctrl_t _position_pid;
// };

// }

// #endif

