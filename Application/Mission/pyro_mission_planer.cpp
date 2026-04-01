#include "cmsis_os.h"
#include "pyro_core_config.h"
extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);

#if (ROBOT_ID == HERO_ID)
#if BOARD_ID == GIMBAL_ID
    extern void hero_gimbal_init(void *argument);
    extern void hero_booster_init(void *argument);
    extern void hero_pc_com_init(void *argument);
#elif BOARD_ID == CHASSIS_ID
    extern void hero_chassis_init(void *argument);
    extern void hero_ui_init(void *argument);
#endif
#endif

#if (ROBOT_ID == SUB_HERO_ID)
#if BOARD_ID == GIMBAL_ID
    extern void hero_gimbal_init(void *argument);
    extern void hero_booster_init(void *argument);
#elif BOARD_ID == CHASSIS_ID
    extern void hero_chassis_init(void *argument);
#endif
#endif

    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

#if (ROBOT_ID == HERO_ID)
#if BOARD_ID == GIMBAL_ID
        xTaskCreate(hero_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(hero_booster_init, "pyro_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(hero_pc_com_init, "pyro_pc_com_init", 512, nullptr,
                    configMAX_PRIORITIES - 3, nullptr);
#elif BOARD_ID == CHASSIS_ID
        xTaskCreate(hero_chassis_init, "pyro_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(hero_ui_init, "pyro_ui_init", 512, nullptr,
                    configMAX_PRIORITIES - 3, nullptr);
#endif
#endif

#if (ROBOT_ID == SUB_HERO_ID)
#if BOARD_ID == GIMBAL_ID
        xTaskCreate(hero_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(hero_booster_init, "pyro_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
#elif BOARD_ID == CHASSIS_ID
        xTaskCreate(hero_chassis_init, "pyro_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
#endif
#endif

#if DEBUG_MODE
        xTaskCreate(start_debug_task, "start_debug_task", 128, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif


        vTaskDelete(nullptr);
    }
}