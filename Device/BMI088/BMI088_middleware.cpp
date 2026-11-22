#include "BMI088_middleware.h"
#include "BMI088_reg.h"
#include "pyro_core_dma_heap.h"

#include <cstring>

namespace
{
constexpr uint8_t spi_read_bit = 0x80U;
constexpr uint8_t spi_write_mask = 0x7FU;
constexpr uint8_t spi_accel_dummy_bytes = 1U;

constexpr uint32_t bus_timeout_ms = 2U;
constexpr uint32_t register_timeout_ms = 5U;

uint16_t read_prefix(bmi088_t::sensor_t sensor)
{
    // 地址阶段占一个字节；加速度计在其后还返回一个 dummy 字节。
    return 1U + (sensor == bmi088_t::sensor_t::accel ? spi_accel_dummy_bytes : 0U);
}

void prepare_read(uint8_t *tx, uint8_t reg, uint16_t length)
{
    tx[0] = reg | spi_read_bit;
    std::memset(tx + 1, 0, length - 1U);
}
} // namespace

bmi088_spi_t::bmi088_spi_t(pyro::spi_drv_t &bus, chip_select_t accel_cs, chip_select_t gyro_cs)
    : _bus(bus), _accel_cs(accel_cs), _gyro_cs(gyro_cs),
      _tx(static_cast<uint8_t *>(pvPortDmaMalloc(2U * dma_buffer_size))),
      _rx(_tx != nullptr ? _tx + dma_buffer_size : nullptr)
{
}

bmi088_spi_t::~bmi088_spi_t()
{
    configASSERT(_dma.state == dma_state_t::idle);
    vPortDmaFree(_tx);
}

bmi088_spi_t::chip_select_t bmi088_spi_t::chip_select(sensor_t sensor) const
{
    return sensor == sensor_t::accel ? _accel_cs : _gyro_cs;
}

void bmi088_spi_t::deselect(void *context)
{
    const auto &cs = *static_cast<const chip_select_t *>(context);
    HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_SET);
}

bool bmi088_spi_t::read_regs(sensor_t sensor, uint8_t reg, uint8_t *data, uint16_t size)
{
    const uint16_t prefix = read_prefix(sensor);
    if (_tx == nullptr || _dma.state != dma_state_t::idle || data == nullptr || size == 0U ||
        size > register_transfer_limit - prefix || _bus.acquire_bus(bus_timeout_ms) != pyro::PYRO_OK)
    {
        return false;
    }

    auto cs = chip_select(sensor);
    const uint16_t length = prefix + size;
    prepare_read(_tx, reg, length);
    HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
    const bool ok = _bus.txrx(_tx, _rx, length, register_timeout_ms) == pyro::PYRO_OK;
    deselect(&cs);
    if (ok)
        std::memcpy(data, _rx + prefix, size);
    (void)_bus.release_bus();
    return ok;
}

bmi088_t::request_result_t bmi088_spi_t::read_regs_request(sensor_t sensor, uint8_t reg, uint16_t size)
{
    const uint16_t prefix = read_prefix(sensor);
    const bool fifo = reg == (sensor == sensor_t::accel ? BMI088::ACC_FIFO_DATA : BMI088::GYRO_FIFO_DATA);
    const uint16_t limit = fifo ? dma_buffer_size : register_transfer_limit;
    if (_tx == nullptr || _dma.state != dma_state_t::idle || size == 0U || size > limit - prefix ||
        _bus.acquire_bus(bus_timeout_ms) != pyro::PYRO_OK)
    {
        return request_result_t::failed;
    }

    _dma.cs = chip_select(sensor);
    _dma.prefix = prefix;
    _dma.size = size;
    const uint16_t length = prefix + size;
    prepare_read(_tx, reg, length);

    _dma.state = dma_state_t::in_flight;
    _bus.set_cplt_hook(deselect, &_dma.cs);
    HAL_GPIO_WritePin(_dma.cs.port, _dma.cs.pin, GPIO_PIN_RESET);
    if (_bus.txrx_dma(_tx, _rx, length) != pyro::PYRO_OK)
    {
        finish_dma();
        return request_result_t::failed;
    }
    return request_result_t::started;
}

bool bmi088_spi_t::wait_dma(uint32_t timeout_ms)
{
    if (_dma.state != dma_state_t::in_flight)
        return false;

    if (_bus.wait_dma(timeout_ms) != pyro::PYRO_OK)
    {
        finish_dma();
        return false;
    }

    _dma.state = dma_state_t::ready;
    return true;
}

bool bmi088_spi_t::read_regs_collect(uint8_t *data, uint16_t size)
{
    if (_dma.state != dma_state_t::ready)
        return false;

    const bool ok = data != nullptr && size == _dma.size;
    if (ok)
        std::memcpy(data, _rx + _dma.prefix, size);
    finish_dma();
    return ok;
}

void bmi088_spi_t::finish_dma()
{
    _bus.set_cplt_hook(nullptr, nullptr);
    deselect(&_dma.cs);
    _dma.state = dma_state_t::idle;
    (void)_bus.release_bus();
}

bool bmi088_spi_t::write_reg(sensor_t sensor, uint8_t reg, uint8_t value)
{
    if (_tx == nullptr || _dma.state != dma_state_t::idle ||
        _bus.acquire_bus(bus_timeout_ms) != pyro::PYRO_OK)
    {
        return false;
    }
    chip_select_t cs = chip_select(sensor);
    const uint8_t tx[] = {static_cast<uint8_t>(reg & spi_write_mask), value};
    HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
    const auto result = _bus.tx(tx, sizeof(tx), register_timeout_ms);
    deselect(&cs);
    (void)_bus.release_bus();
    return result == pyro::PYRO_OK;
}
