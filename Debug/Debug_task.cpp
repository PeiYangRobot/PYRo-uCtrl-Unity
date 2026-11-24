#include "cmsis_os.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C"
{
    extern void pyro_vofa_task(void *arg);
    extern void pyro_jcom_task(void *arg);
    extern void pyro_edt_task(void *arg);
    void start_debug_task(void *arg)
    {
#if DEBUG_MODE
#ifdef VOFA_DEBUG_PORT
        xTaskCreate(pyro_vofa_task, "pyro_vofa_demo", 128, nullptr,
                    tskIDLE_PRIORITY + 1, nullptr);
#endif

#ifdef JCOM_DEBUG_PORT
        xTaskCreate(pyro_jcom_task, "pyro_jcom_task", 128, nullptr,
                    tskIDLE_PRIORITY + 1, nullptr);
#endif
#if EDT_DEBUG_EN
        xTaskCreate(pyro_edt_task, "pyro_edt_task", 384, nullptr,
                    tskIDLE_PRIORITY + 1, nullptr);
#endif
#endif
        vTaskDelete(nullptr);
    }
}
