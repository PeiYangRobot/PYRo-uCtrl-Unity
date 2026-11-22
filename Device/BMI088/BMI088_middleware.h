#ifndef BMI088_MIDDLEWARE_H
#define BMI088_MIDDLEWARE_H

#include <cstdint>

#include "pyro_spi_drv.h"

// BMI088 两个子器件的寄存器接口。读出数据不含总线的地址和 dummy 字节。
class bmi088_t
{
  public:
    enum class sensor_t : uint8_t { accel, gyro };

    virtual ~bmi088_t() = default;

    virtual bool read_regs(sensor_t sensor, uint8_t reg, uint8_t *data, uint16_t size) = 0;
    virtual bool write_reg(sensor_t sensor, uint8_t reg, uint8_t value) = 0;

    enum class request_result_t : uint8_t { started, unsupported, failed };

    // 同一任务顺序执行三段；成功发起后必须等待，成功等待后必须收取。
    // 只有 unsupported 允许改走同步读取。failed 可能已开始总线传输。
    virtual request_result_t read_regs_request(sensor_t, uint8_t, uint16_t)
    {
        return request_result_t::unsupported;
    }
    virtual bool wait_dma(uint32_t) { return false; }
    virtual bool read_regs_collect(uint8_t *, uint16_t) { return false; }
};

// BMI088 的 SPI 适配。每个实例从 DMA 堆分配自己的收发缓冲。
class bmi088_spi_t final : public bmi088_t
{
  public:
    struct chip_select_t
    {
        GPIO_TypeDef *port;
        uint16_t pin;
    };

    bmi088_spi_t(pyro::spi_drv_t &bus, chip_select_t accel_cs, chip_select_t gyro_cs);
    // 销毁实例前须结束在途的 DMA 事务。
    ~bmi088_spi_t() override;

    bmi088_spi_t(const bmi088_spi_t &) = delete;
    bmi088_spi_t &operator=(const bmi088_spi_t &) = delete;

    bool read_regs(sensor_t sensor, uint8_t reg, uint8_t *data, uint16_t size) override;
    bool write_reg(sensor_t sensor, uint8_t reg, uint8_t value) override;
    request_result_t read_regs_request(sensor_t sensor, uint8_t reg, uint16_t size) override;
    bool wait_dma(uint32_t timeout_ms) override;
    bool read_regs_collect(uint8_t *data, uint16_t size) override;

  private:
    static constexpr uint16_t register_transfer_limit = 32U;
    // 可容纳 63 个加速度帧及 SPI 地址和 dummy 字节。
    static constexpr uint16_t dma_buffer_size = 448U;

    // 两颗子器件共用一条外部总线，所有权由总线锁管理。
    pyro::spi_drv_t &_bus;
    chip_select_t _accel_cs;
    chip_select_t _gyro_cs;
    uint8_t *const _tx;
    uint8_t *const _rx;

    enum class dma_state_t : uint8_t { idle, in_flight, ready };
    struct
    {
        dma_state_t state = dma_state_t::idle;
        chip_select_t cs{};
        uint16_t size = 0U;
        uint16_t prefix = 0U;
    } _dma;

    chip_select_t chip_select(sensor_t sensor) const;
    static void deselect(void *context);
    // 完成或失败后清理钩子和片选，再归还总线。
    void finish_dma();
};

#endif
