#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#include "edt_rtt.h"
#include "edt_scope.h"

extern "C" void pyro_edt_task(void *arg)
{
    (void)arg;
    edt_rtt_init();
    edt_scope_init(1000, 0);

    TickType_t last_wake = xTaskGetTickCount();
    for (;;)
    {
        edt_scope_task(HAL_GetTick());
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}
