/**
 * @file pyro_spi_drv.h
 * @brief Header file for the PYRO SPI driver.
 * PYRo SPI 驱动头文件。
 *
 * 封装一路 SPI 的收发，覆盖两种模式（仿 pyro_uart_drv 的阻塞/DMA 双 API）：
 *   - 阻塞：tx()/txrx()，HAL 轮询，任务忙等至传输完成；
 *   - DMA ：tx_dma()/txrx_dma()，fire-and-forget——发起后立即返回，
 *     完成经 wait_dma() 查询（或完成钩子在 ISR 中即时通知）。
 * 异步用法：start 之后、wait_dma 之前的窗口内任务可执行其他工作，
 * 传输在后台由 DMA 完成，无需占用 CPU。
 *
 * 同一个 HAL 句柄由 BSP 创建一份总线对象。设备层取得总线互斥量，
 * 管理独立片选，并维护 DMA 缓冲的生命周期与 H7 D-cache 一致性。
 * HAL 默认回调用弱符号覆盖。
 */

#ifndef __PYRO_SPI_DRV_H__
#define __PYRO_SPI_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "pyro_core_def.h"
#include "map.h"

#include "FreeRTOS.h"
#include "semphr.h" /* SemaphoreHandle_t */

namespace pyro
{

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief SPI driver class（单句柄一份实例，任务上下文使用）。
 * SPI 驱动类。
 */
class spi_drv_t
{
    /* 实例化权收归 BSP：一个 HAL 外设句柄 = 一条总线 = 一个驱动实例。
     * 外部统一通过 bsp_spi::get_spiX() 取得引用。 */
    friend class bsp_spi;

  public:
    /* Public Types ---------------------------------------------------------*/
    /**
     * @brief Internal state flags for tracking driver status.
     * 用于跟踪驱动状态的内部标志位。
     */
    typedef struct state_t
    {
        volatile uint8_t busy    : 1; /* DMA 事务在途（ISR 完成时清零） */
        volatile uint8_t timeout : 1; /* 等待 DMA 完成超时 */
        volatile uint8_t error   : 1; /* 传输错误（详见 last_error()） */
    } state_t;

    /* Constructor and Destructor -------------------------------------------*/
    /**
     * @brief Destructor.
     * 析构函数。
     */
    ~spi_drv_t();

    /* 禁用拷贝：持有信号量句柄 + 注册进全局句柄表 */
    spi_drv_t(const spi_drv_t &)            = delete;
    spi_drv_t &operator=(const spi_drv_t &) = delete;

    /* Public Methods - Blocking Transmission -------------------------------*/
    /**
     * @brief Blocking transmit using HAL polling.
     * 使用 HAL 轮询方式的阻塞发送。
     *
     * @param tx       Send buffer.
     *                发送缓冲。
     * @param size     Byte count.
     *                字节数。
     * @param waittime HAL timeout in ms.
     *                HAL 超时时间（ms）。
     */
    status_t tx(const uint8_t *tx, uint16_t size, uint32_t waittime);

    /**
     * @brief Blocking full-duplex transmit/receive using HAL polling.
     * 使用 HAL 轮询方式的阻塞全双工收发。
     *
     * @param tx       Send buffer.
     *                发送缓冲。
     * @param rx       Receive buffer, valid on return.
     *                接收缓冲，返回时数据有效。
     * @param size     Byte count.
     *                字节数。
     * @param waittime HAL timeout in ms.
     *                HAL 超时时间（ms）。
     */
    status_t txrx(const uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t waittime);

    /* Public Methods - DMA Transmission (fire-and-forget) ------------------*/
    /**
     * @brief Transmit using HAL DMA; returns immediately after issuing.
     * 使用 HAL DMA 发送，发起后立即返回（fire-and-forget）。
     *
     * @param tx   Send buffer, must not be modified until the transaction
     *             completes (wait_dma() returns / hook fires).
     *             发送缓冲，事务完成前不得改写（DMA 在后台读）。
     * @param size Byte count.
     *             字节数。
     * @retval PYRO_OK     已发起（在途）
     * @retval PYRO_BUSY   上一笔事务仍在途
     * @retval PYRO_ERROR  参数错误 / 未配 TX DMA 通道 / 启动失败
     * @note  完成用 wait_dma() 查询；完成点会调用 set_cplt_hook() 注册的
     *        钩子（ISR 上下文），随后才给完成信号量。
     */
    status_t tx_dma(const uint8_t *tx, uint16_t size);

    /**
     * @brief Full-duplex transmit/receive using HAL DMA; returns immediately.
     * 使用 HAL DMA 全双工收发，发起后立即返回（fire-and-forget）。
     *
     * @param tx   Send buffer, must not be modified until completion.
     *             发送缓冲，完成前不得改写。
     * @param rx   Receive buffer, valid after wait_dma() returns PYRO_OK.
     *             接收缓冲，wait_dma() 返回 PYRO_OK 后有效。
     * @param size Byte count.
     *             字节数。
     * @retval PYRO_OK     已发起（在途）
     * @retval PYRO_BUSY   上一笔事务仍在途
     * @retval PYRO_ERROR  参数错误 / 未配 DMA 通道 / 启动失败
     */
    status_t txrx_dma(const uint8_t *tx, uint8_t *rx, uint16_t size);

    /**
     * @brief Waits for the in-flight DMA transaction to complete.
     * 等待在途 DMA 事务完成。
     *
     * @param waittime Timeout in ms; on expiry the DMA is aborted.
     *                超时（ms）；超时则中止 DMA。
     * @retval PYRO_OK     完成且无错误（数据已完整，HAL 已保证 BSY=0）；
     *                     无在途事务时也返回 OK（绝不干等）
     * @retval PYRO_TIMEOUT 等待超时（已中止 DMA）
     * @retval PYRO_ERROR   传输错误（错误码见 last_error()）
     * @note  发起与等待分离是异步的关键：start 之后到本调用之前，
     *        任务可执行任意其他工作。
     */
    status_t wait_dma(uint32_t waittime);

    /**
     * @brief Queries whether a DMA transaction is in flight.
     * 查询是否有 DMA 事务在途。
     */
    [[nodiscard]] bool dma_busy() const;

    /* Public Methods - Recovery --------------------------------------------*/
    /**
     * @brief Aborts the ongoing DMA transaction.
     * 中止正在进行的 DMA 事务（超时/错误恢复用）。
     */
    status_t abort();

    /* Public Methods - ISR Entry -------------------------------------------*/
    /**
     * @brief DMA complete/error callback entry (ISR context).
     * DMA 完成/错误回调入口（ISR 上下文，由 HAL 弱回调覆盖函数调用）。
     *
     * @note  Only for ISR; task context uses the return values of
     *        tx_dma()/txrx_dma().
     *        仅限中断；任务上下文请用 tx_dma()/txrx_dma() 的返回值。
     */
    void notify_from_isr();

    /* Public Methods - Bus Ownership ---------------------------------------*/
    /**
     * @brief Acquires exclusive ownership of the bus.
     * 取得总线独占权（多从机时串行化各设备的访问）。
     *
     * 【为什么需要】
     * 一条 SPI 总线上可挂多颗从机，同一时刻只能有一笔事务在途。
     * 原先各设备靠 `dma_busy()` 做「非阻塞检查 + 失败返回」，这既是
     * TOCTOU（检查与占用之间可被抢占），又把「排队」误报成「总线故障」。
     * 改为阻塞在互斥量上等待：**忙就挂起，不发起请求**。
     *
     * 由任务在拉低 CS 前取得总线锁，完成传输或中止后归还锁。
     * DMA 完成钩子在锁内绑定，释放总线前解绑。
     *
     * @param waittime Timeout in ms; portMAX_DELAY waits forever.
     *                 超时（ms）；portMAX_DELAY 表示永久等待。
     * @retval PYRO_OK    已取得
     * @retval PYRO_BUSY  超时未取得（调用方应放弃本次事务，绝不继续拉 CS）
     * @retval PYRO_ERROR 互斥量不可用（构造时分配失败）
     */
    status_t acquire_bus(uint32_t waittime);

    /**
     * @brief Releases bus ownership.
     * 释放总线独占权。
     *
     * @retval PYRO_OK 已释放
     * @note 仅任务上下文可调用。ISR 中只做 cs_high() 与唤醒，
     *       不在此释放（mutex 有优先级继承，必须用任务态 API）。
     * @note 由取得总线锁的任务配对调用一次。
     */
    status_t release_bus();

    /* Public Methods - Completion Hook -------------------------------------*/
    /**
     * @brief Registers a transaction-complete hook (ISR context).
     * 注册事务完成钩子（中断上下文）。
     *
     * 在 DMA 完成/错误时、给完成信号量之前调用，用于在数据搬运的
     * 完成点立即释放 CS。这不只是"省掉唤醒延迟"的性能优化——
     * **CS 上升沿是器件唯一的原子事务边界**：器件在此沿提交写入、
     * 复位相位序列器、释放 MISO。若 CS 迟迟不拉高，下一笔事务的
     * cs_low() 对已是低电平的引脚是 no-op，器件会把两笔独立事务
     * 合并成一笔（地址字节被当数据、FIFO 指针少推进一次）。
     *
     * @param hook Callback function; pass nullptr to unregister.
     *             回调函数指针；传 nullptr 注销。
     * @param ctx  Context pointer passed to the hook (usually the caller's this).
     *             回调上下文（通常为调用方 this）。
     *
     * @note 【一主多从 + 绑定规则】
     *       钩子是单槽的，本实例被同总线的多个从机共享。绑定的生命周期
     *       必须与总线所有权严格对齐——**绑定在 acquire_bus 之后，
     *       解绑在 release_bus 之前**，两者近乎同步：
     *
     *         acquire_bus()              ← 取得总线
     *           set_cplt_hook(self)      ← 绑定（只可能被自己看到）
     *           ...发起、等待、收取...
     *           set_cplt_hook(nullptr)   ← 解绑（业务已完成）
     *         release_bus()              ← 归还总线
     *
     *       两层保证：
     *         ① 绑定发生在取锁之后 → 别的设备此刻无论如何都改不到它；
     *         ② 解绑发生在放锁之前 → 总线交出去时钩子已清空，
     *            即使后续出现迟到的完成中断（超时/abort 后），
     *            ISR 也不会误调上一个设备的 CS 释放函数。
     *       发起失败路径必须显式解绑，否则钩子会残留到下一次事务，
     *       届时若发生迟到中断就会释放错误的 CS。
     *
     * @note 为什么用裸函数指针而非 std::function：
     *       本回调在 ISR 中执行，且每笔事务前都要重设一次。实测改为
     *       std::function 会增加 1312 B Flash，且每拍两次事务的赋值
     *       使采集平均耗时从 157 µs 升至 173 µs（+10%）。
     *       裸指针的调用是一次已知地址的跳转，赋值是两条字写入，
     *       对 1 ms 采集周期最友好。
     */
    void set_cplt_hook(void (*hook)(void *), void *ctx);

    /* Public Methods - Static Access ---------------------------------------*/
    /**
     * @brief Provides access to the static map for ISR routing.
     * 提供对静态映射表的访问，用于中断路由。
     */
    static map_t<SPI_HandleTypeDef *, spi_drv_t *> &spi_map();

    /**
     * @brief Looks up the driver instance by its hardware handle.
     * 按硬件句柄查找驱动实例。
     *
     * @retval Instance pointer; nullptr if not registered.
     *         实例指针；未注册返回 nullptr。
     */
    static spi_drv_t *get(SPI_HandleTypeDef *hspi);

    /* Public Members - State/Data ------------------------------------------*/
    /**
     * @brief Internal state tracking.
     * 内部状态跟踪。
     */
    state_t state{};

    /**
     * @brief Last HAL error code (HAL_SPI_ERROR_*), snapshot at completion.
     * 最近一次 HAL 错误码（HAL_SPI_ERROR_*），完成时快照。
     */
    [[nodiscard]] uint32_t last_error() const;

  private:
    /* Private Methods ------------------------------------------------------*/
    /**
     * @brief Constructor（私有，仅友元 bsp_spi 可调用）。
     * 构造函数（私有，由 BSP 创建以保证「一总线一实例」）。
     *
     * @param hspi Pointer to the HAL SPI handle.
     * HAL SPI 句柄指针。
     */
    explicit spi_drv_t(SPI_HandleTypeDef *hspi);

    /* Private Members ------------------------------------------------------*/
    SPI_HandleTypeDef *_hspi;      /* HAL handle for the peripheral. 外设 HAL 句柄 */
    status_t stop_dma();

    bool _needs_recovery = false;
    SemaphoreHandle_t  _done;      /* Binary semaphore, given by ISR on completion. 完成信号量 */
    SemaphoreHandle_t  _bus_mutex; /* Bus mutex, task context only. 总线互斥量 */
    volatile uint32_t  _err;       /* HAL error code snapshot. HAL 错误码快照 */

    /* Transaction-complete hook (ISR context, invoked before giving _done).
     * 事务完成钩子（ISR 上下文，给完成信号量之前调用）。
     * 单槽：一条总线同一时刻只有一笔事务，故只需保存一个订阅者。
     * 裸函数指针 + 上下文指针，赋值是两条字写入，ISR 中是一次已知地址跳转。 */
    void (*_cplt_hook)(void *){nullptr};
    void *_cplt_ctx{nullptr};
};

} // namespace pyro

#endif // __PYRO_SPI_DRV_H__
