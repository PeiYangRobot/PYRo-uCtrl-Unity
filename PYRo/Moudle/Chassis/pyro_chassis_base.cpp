#include "pyro_chassis_base.h"

extern "C" void chassis_task(void *argument);
extern "C" void chassis_init(void *argument);

namespace pyro
{

chassis_base_t::chassis_base_t() : chassis_base_t(type_t::UNKNOWN)
{
}
chassis_base_t::chassis_base_t(const type_t type)
{
    _type = type;
    xTaskCreate(chassis_init, "chassis_init", 512, this, tskIDLE_PRIORITY + 1,
                &_chassis_init_handle);
}

void chassis_base_t::thread()
{
    scoped_mutex_t lock(_mutex);
    update_feedback();
    kinematics_solve();
    chassis_control();
    power_control();
    send_motor_command();
}

} // namespace pyro

extern "C" void chassis_init(void *argument)
{
    auto *chassis = static_cast<pyro::chassis_base_t *>(argument);
    if (chassis)
    {
        chassis->init();
        xTaskCreate(chassis_task, "chassis_thread", 256, chassis,
                    tskIDLE_PRIORITY + 2, &chassis->_chassis_task_handle);
        vTaskDelete(nullptr);
    }
    while (true)
    {
    }
}

extern "C" void chassis_task(void *argument)
{
    auto *chassis = static_cast<pyro::chassis_base_t *>(argument);
    if (chassis)
    {
        while (true)
        {
            chassis->thread();
            vTaskDelay(1);
        }
    }
    vTaskDelete(nullptr);
}