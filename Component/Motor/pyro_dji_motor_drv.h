#ifndef DJI_M_MOTOR_DRV_H
#define DJI_M_MOTOR_DRV_H

#include "pyro_motor_base.h"

namespace pyro
{
class dji_motor_tx_frame_t
{
  public:
    enum register_id_t
    {
        id_1 = 0,
        id_2,
        id_3,
        id_4,
        id_5,
        id_6,
        id_7,
        id_8
    };
    using _frame_key_t = std::pair<uint32_t, bsp_can::which_can>;

    dji_motor_tx_frame_t(bsp_can::which_can which, uint32_t id);
    ~dji_motor_tx_frame_t();

    const _frame_key_t &get_key(void);
    status_t register_id(register_id_t id);

    status_t update_value(uint8_t id, int16_t value);

  private:
    _frame_key_t _key;
    can_drv_t *_can;
    std::array<uint8_t, 8> _data;

    std::array<uint8_t, 4> _register_list;
    std::array<uint8_t, 4> _update_list;
    std::array<int16_t, 4> _value_list;
};

class dji_motor_tx_frame_pool_t
{
  public:
    static dji_motor_tx_frame_pool_t *get_instance(void);
    dji_motor_tx_frame_t *get_frame(bsp_can::which_can which,
                                    uint32_t id);

  private:
    dji_motor_tx_frame_pool_t(void);
    dji_motor_tx_frame_pool_t(const dji_motor_tx_frame_pool_t &) = delete;
    dji_motor_tx_frame_pool_t &
    operator=(const dji_motor_tx_frame_pool_t &) = delete;
    static dji_motor_tx_frame_pool_t *_instancePtr;
    std::vector<dji_motor_tx_frame_t *> _frame_list;
};

class dji_motor_drv_t : public motor_base_t
{
  public:
    dji_motor_drv_t(dji_motor_tx_frame_t::register_id_t id,
                    bsp_can::which_can which);
    ~dji_motor_drv_t() = default;

    status_t enable() override;
    status_t disable() override;

    status_t update_feedback() override;
    status_t send_torque(float torque) override;

  protected:
    dji_motor_tx_frame_t::register_id_t _register_id;
    uint32_t _tx_id;
    uint32_t _rx_id;
    float _max_torque_f;
    int16_t _max_torque_i;
    status_t _init_status = status_t::PYRO_OK;
    dji_motor_tx_frame_t *_tx_frame;
};

class dji_m3508_motor_drv_t : public dji_motor_drv_t
{
  public:
    static constexpr float reduction_ratio = 19.20320855614973f;
    static constexpr float reciprocal_reduction_ratio =  0.0520746310219994f;
    static constexpr float reduction_ratio_14 = 13.77f;
    static constexpr float reciprocal_reduction_ratio_14 =  1 / reduction_ratio_14;
    dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::register_id_t id,
                          bsp_can::which_can which);
    ~dji_m3508_motor_drv_t()
    {
    }
};

class dji_m2006_motor_drv_t : public dji_motor_drv_t
{
  public:
    dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::register_id_t id,
                          bsp_can::which_can which);
    ~dji_m2006_motor_drv_t()
    {
    }
};
class dji_gm_6020_motor_drv_t : public dji_motor_drv_t
{
  public:
    dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::register_id_t id,
                            bsp_can::which_can which);
    ~dji_gm_6020_motor_drv_t()
    {
    }
};
}; // namespace pyro


#endif