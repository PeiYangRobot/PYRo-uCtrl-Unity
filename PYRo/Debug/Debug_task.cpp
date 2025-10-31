#include "cmsis_os.h"
#include "pyro_core_config.h"
#include "task.h"

extern "C"
{
    extern void pyro_vofa_task(void *arg);
    extern void pyro_jcom_task(void *arg);
    void start_debug_task(void *arg)
    {
#if VOFA_DEBUG_EN
        xTaskCreate(pyro_vofa_task, "pyro_vofa_demo", 128, nullptr,
                    tskIDLE_PRIORITY + 1, nullptr);
#endif

#if JCOM_DEBUG_EN
        xTaskCreate(pyro_jcom_task, "pyro_jcom_task", 128, nullptr,
                    tskIDLE_PRIORITY + 1, nullptr);
#endif
        vTaskDelete(nullptr);
    }
}
