/*
 * @Author: lucky zhou_20006ee@163.com
 * @Date: 2025-10-15 14:38:32
 * @LastEditors: lucky zhou_20006ee@163.com
 * @LastEditTime: 2025-10-15 15:03:04
 * @FilePath: \PYRo-uCtrl-Unity\PYRo\Application\Demo\demo_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "cmsis_os.h"
#include "pyro_core_config.h"
#include "task.h"


extern "C"
{
    extern void pyro_rc_demo(void *arg);
    extern void pyro_motor_demo(void *arg);
    extern void pyro_wheel_demo(void *arg);
    extern void pyro_controller_demo(void *arg);
    extern void pyro_vofa_demo(void *arg);
    extern void IMU_task(void *argument);
    extern void referee_task(void *arg);
    void start_demo_task(void const *argument)
    {
        // 变成IMU_TASK 之后自己写一个demo
#if DEMO_MODE

#if RC_DEMO_EN
        xTaskCreate(pyro_rc_demo, "pyro_rc_demo", 128, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif
#if MOTOR_DEMO_EN
        xTaskCreate(pyro_motor_demo, "pyro_motor_demo", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif
#if CONTROL_DEMO_EN
        xTaskCreate(pyro_control_demo, "pyro_control_demo", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif


#if CONTROLLER_DEMO_EN
        xTaskCreate(pyro_controller_demo, "pyro_controller_demo", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif

#if SHOOT_DEMO_EN
        xTaskCreate(pyro_shoot_demo, "pyro_shoot_demo", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif
#if IMU_DEMO_EN
        xTaskCreate(IMU_task, "IMU_task", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif

#if REFEREE_DEMO_EN
        xTaskCreate(referee_task, "referee_task", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif

#endif
        vTaskDelete(nullptr);
    }
}