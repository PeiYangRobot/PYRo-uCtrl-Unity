#include <cstdlib>
#include "FreeRTOS.h"

void *operator new(const std::size_t size)
{
    void *ptr = pvPortMalloc(size);
    return ptr;
}

void *operator new[](const std::size_t size)
{
    void *ptr = pvPortMalloc(size);
    return ptr;
}

void operator delete(void *ptr) noexcept
{
    vPortFree(ptr);
}

void operator delete[](void *ptr) noexcept
{
    vPortFree(ptr);
}