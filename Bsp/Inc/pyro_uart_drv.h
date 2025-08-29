#ifndef __PYRO_UART_DRV_H__
#define __PYRO_UART_DRV_H__

#include "pyro_core_def.h"

namespace pyro
{

class uart_drv_t
{

public:
    uart_drv_t();

    
    get_notify_queue(QueueHandle_t* queue);

    write(uint8_t* p, uint16_t size, uint32_t waittime);
    write(uint8_t* p, uint16_t size);

    get_status()
    reset()

private:
    error_callback();
    event_callback();

}    


}

#endif
