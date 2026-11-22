#include "pyro_bsp_spi.h"
#include "spi.h"
#include "task.h"

namespace pyro
{
spi_drv_t &bsp_spi::get_spi2()
{
    // 本工程关闭了 C++ 的局部静态初始化锁。
    static spi_drv_t bus(&hspi2);
    return bus;
}
} // namespace pyro
