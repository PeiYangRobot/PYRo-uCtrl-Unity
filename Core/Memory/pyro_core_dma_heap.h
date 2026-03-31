#ifndef __PYRO_CORE_DMA_HEAP_H__
#define __PYRO_CORE_DMA_HEAP_H__

#include "FreeRTOS.h"   /* for HeapStats_t, config macros */
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

    void *pvPortDmaMalloc( size_t xWantedSize );
    void vPortDmaFree( void *pv );
    void vPortGetDmaHeapStats( HeapStats_t *pxHeapStats );

#ifdef __cplusplus
}
#endif

#endif /* DMA_HEAP_H */
