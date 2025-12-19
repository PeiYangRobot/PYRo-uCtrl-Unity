#include "cmsis_os.h"
#include "pyro_uart_drv.h"

#include <cstdio>
#include <cstring>

extern "C" void referee_usart_task(void *argument);
extern "C" void referee_rx_handler(uint8_t *buf, uint16_t Size);

bool referee_uart_callback(uint8_t *data, uint16_t size,
                           BaseType_t xHigherPriorityTaskWoken)
{

    referee_rx_handler(data, size);
    return true;
}

extern "C" void referee_init()
{
    pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart1)
        ->add_rx_event_callback(referee_uart_callback, 0x20);
}

extern "C" void referee_task(void *arg)
{
    referee_usart_task(nullptr);
}
