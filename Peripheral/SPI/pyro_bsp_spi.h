#ifndef PYRO_BSP_SPI_H
#define PYRO_BSP_SPI_H

#include "pyro_spi_drv.h"

namespace pyro
{
class bsp_spi
{
  public:
    // 在 MX_SPI2_Init 和调度器启动后由任务调用。
    static spi_drv_t &get_spi2();
};
} // namespace pyro

#endif
