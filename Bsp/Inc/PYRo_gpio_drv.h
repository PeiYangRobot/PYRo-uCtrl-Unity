#ifndef __PYRO_GPIO_DRV_H__
#define __PYRO_GPIO_DRV_H__

#include "PYRo_core_def.h"

#ifdef PYRO_USE_H7
#include "stm32h7xx_hal.h"      // IWYU pragma: keep
#include "stm32h7xx_hal_gpio.h" // IWYU pragma: keep
#endif

#include <functional>
namespace pyro 
{
class gpio_drv_t
{
public:
    enum state_t
    {
        RESET = 0,
        SET   = 1,
    };
    gpio_drv_t(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    state_t read(void);
    void write(state_t);
    void toogle(void);
    status_t register_callback(std::function<void(void)>callback);

private:    
    enum{MAX_PIN_NUM = 16};
    GPIO_TypeDef *_port;
    uint16_t      _pin;
    uint8_t       _pin_num;
    state_t       _state;
    friend void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
    static std::function<void(void)> callback_map[16];
    static void port_callback(uint16_t pin);  
     
};

}

#endif
