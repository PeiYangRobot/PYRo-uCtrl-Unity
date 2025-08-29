#include "PYRo_gpio_drv.h"


#include "pyro_common_func.h"

namespace pyro
{

gpio_drv_t::gpio_drv_t(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    _port = GPIOx;
    _pin  = GPIO_Pin;

    _pin_num = get_bit_pos_l(_pin); 
}

gpio_drv_t::state_t gpio_drv_t::read(void)
{
    _state = (state_t)HAL_GPIO_ReadPin(_port, _pin);
    return _state;
}

void gpio_drv_t::write(gpio_drv_t::state_t state) 
{
    HAL_GPIO_WritePin(_port, 
                   _pin,
                  (GPIO_PinState)state);
}

void gpio_drv_t::toogle(void)
{
    HAL_GPIO_TogglePin(_port, _pin);
}

status_t gpio_drv_t::register_callback(std::function<void(void)>callback)
{
    if(NULL == callback)
    {
        return PYRO_PARAM_ERROR;
    }
    else if(_pin_num >= MAX_PIN_NUM)
    {
        return PYRO_PARAM_ERROR;
    }
    else if(NULL != callback_map[_pin_num])
    {
        return PYRO_PARAM_ERROR;
    }
    if(!(SYSCFG->EXTICR[_pin_num >> 2U] & 
        (GPIO_GET_INDEX(_port) << (4U * (_pin_num & 0x03U)))))
    {
        return PYRO_PARAM_ERROR;
    }       
    else 
    {
        callback_map[_pin_num] = callback;
    }
}
void gpio_drv_t::port_callback(uint16_t pin)
{
    if(NULL != callback_map[pin])
    {
        callback_map[pin]();
    }
    else 
    {
        return;
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    gpio_drv_t::port_callback(GPIO_Pin);
}

    
}

