# Core Memory

This directory is responsible for memory management. It includes `new` operator overloading, DMA-capable memory allocation, and other memory-related utility functions.

该目录负责内存管理。它包括new重载、支持 DMA 的内存分配以及其他内存相关工具函数的实现。

---
**Change Log**

* V1.0, 2025-10-15, By Lucky: created
    * 实现了new重载和dma内存分配
    * warning:需要为.dma_heap编写ld文件