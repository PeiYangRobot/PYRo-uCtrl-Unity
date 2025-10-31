/*
 * @Author: lucky zhou_20006ee@163.com
 * @Date: 2025-10-15 14:38:32
 * @LastEditors: lucky zhou_20006ee@163.com
 * @LastEditTime: 2025-10-15 15:03:04
 * @FilePath: \PYRo-uCtrl-Unity\PYRo\Application\Demo\demo_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "cmsis_os.h"
#include "pyro_core_config.h"
#include "task.h"


extern "C"
{
void pyro_rc_demo(void *arg);
void pyro_motor_demo(void *arg);
void pyro_wheel_demo(void *arg);
void pyro_controller_demo(void *arg);
void pyro_vofa_demo(void *arg);
void pyro_control_demo(void *arg);

void start_demo_task(void const *argument)
{
#if DEMO_MODE

#if RC_DEMO_EN
     xTaskCreate(pyro_rc_demo, "pyro_rc_demo", 128, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif
#if MOTOR_DEMO_EN
     xTaskCreate(pyro_motor_demo, "pyro_motor_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif
#if WHEEL_DEMO_EN
     xTaskCreate(pyro_wheel_demo, "pyro_wheel_demo", 512, nullptr,
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


#endif
    vTaskDelete(nullptr);
}

}