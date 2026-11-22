#include "BMI088_driver.h"
#include "BMI088_middleware.h"
#include "BMI088_reg.h"
#include "stm32h7xx_hal.h"
#include "pyro_dwt_drv.h"
#include <cstddef>
#include <cstring>

namespace
{
using sensor_t = bmi088_t::sensor_t;

using xyz_t = int16_t[3];
constexpr uint16_t accel_fifo_frame_size = 1U + sizeof(xyz_t);

#if defined(__BYTE_ORDER__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "BMI088 burst reads require a little-endian target");
#elif !defined(_WIN32)
#error "Cannot verify byte order for BMI088 burst reads"
#endif

static_assert(sizeof(xyz_t) == BMI088::ACCEL_ZOUT_M - BMI088::ACCEL_XOUT_L + 1U);
static_assert(sizeof(xyz_t) == BMI088::GYRO_Z_H - BMI088::GYRO_X_L + 1U);
static_assert(BMI088::TEMP_L == BMI088::TEMP_M + 1U);

template <typename T>
uint8_t *register_bytes(T &value)
{
    return reinterpret_cast<uint8_t *>(&value);
}

constexpr imu_cfg_t default_config()
{
    imu_cfg_t cfg{};
    cfg.acc_range = 6U;
    cfg.acc_freq = 800U;
    cfg.gyro_range = 2000U;
    cfg.gyro_freq = 2000U;

    cfg.fifo_en = true;
    return cfg;
}

struct register_setting_t
{
    uint8_t address;
    uint8_t value;
};

bool accel_odr_code(uint16_t hz, uint8_t &code);
bool gyro_odr_code(uint16_t hz, uint8_t &code);
bool accel_range_code(uint16_t g, uint8_t &code, float &mps2_per_lsb);
bool gyro_range_code(uint16_t dps, uint8_t &code, float &rps_per_lsb);

template <std::size_t N>
bool apply_settings(bmi088_t &device, sensor_t sensor, const register_setting_t (&settings)[N]);

template <std::size_t N>
bool init_sensor(bmi088_t &device, sensor_t sensor, uint8_t chip_id_reg, uint8_t chip_id,
                    uint8_t reset_reg, uint8_t reset_value, const register_setting_t (&settings)[N]);

bool scan_accel_fifo(const uint8_t *raw, uint16_t bytes, uint16_t *payload_offsets, uint16_t &frames);
float temperature_celsius(const uint8_t (&raw)[2]);
bool read_sample(bmi088_t &device, sensor_t sensor, uint8_t reg, uint8_t *data, uint16_t size);

} // namespace

bmi088_drv_t::bmi088_drv_t(bmi088_t &device)
    : imu_base_drv(default_config()), _device(device)
{
    _data.acc.sample = _acc;
    _data.acc.capacity = stream_capacity;
    _data.gyro.sample = _gyro;
    _data.gyro.capacity = stream_capacity;
}

bool bmi088_drv_t::init()
{
    _ready = false;
    _fifo_recovery_pending = false;
    assume_acc_point(_data.acc.available());
    assume_gyro_point(_data.gyro.available());

    // 把板级物理量配置映射到寄存器编码和样本换算系数。
    const auto &cfg = _data.cfg;
    uint8_t acc_odr = 0U, gyro_odr = 0U, acc_range = 0U, gyro_range = 0U;
    float acc_scale = 0.0f, gyro_scale = 0.0f;
    if (SystemCoreClock == 0U ||
        !accel_odr_code(cfg.acc_freq, acc_odr) || !gyro_odr_code(cfg.gyro_freq, gyro_odr) ||
        !accel_range_code(cfg.acc_range, acc_range, acc_scale) ||
        !gyro_range_code(cfg.gyro_range, gyro_range, gyro_scale))
    {
        return false;
    }

    // 各子器件复位后，先设置满量程，再设置 ODR。
    const register_setting_t accel_settings[] = {
        {BMI088::ACC_PWR_CTRL, BMI088::ACC_ENABLE_ACC_ON},
        {BMI088::ACC_PWR_CONF, BMI088::ACC_PWR_ACTIVE_MODE},
        {BMI088::ACC_RANGE, acc_range},
        {BMI088::ACC_CONF,
         static_cast<uint8_t>(BMI088::ACC_NORMAL | BMI088::ACC_CONF_MUST_SET | acc_odr)},
        {BMI088::INT1_IO_CTRL, BMI088::ACC_INT1_IO_ENABLE | BMI088::ACC_INT1_GPIO_PP | BMI088::ACC_INT1_GPIO_LOW},
        {BMI088::INT_MAP_DATA, BMI088::ACC_INT1_DRDY_INTERRUPT},
    };
    const register_setting_t gyro_settings[] = {
        {BMI088::GYRO_RANGE, gyro_range},
        {BMI088::GYRO_BANDWIDTH, static_cast<uint8_t>(BMI088::GYRO_BANDWIDTH_MUST_SET | gyro_odr)},
        {BMI088::GYRO_LPM1, BMI088::GYRO_NORMAL_MODE},
        {BMI088::GYRO_CTRL, BMI088::DRDY_ON},
        {BMI088::GYRO_INT3_INT4_IO_CONF, BMI088::GYRO_INT3_GPIO_PP | BMI088::GYRO_INT3_GPIO_LOW},
        {BMI088::GYRO_INT3_INT4_IO_MAP, BMI088::GYRO_DRDY_IO_INT3},
    };

    if (!init_sensor(_device, sensor_t::accel, BMI088::ACC_CHIP_ID, BMI088::ACC_CHIP_ID_VALUE,
                     BMI088::ACC_SOFTRESET, BMI088::ACC_SOFTRESET_VALUE, accel_settings) ||
        !init_sensor(_device, sensor_t::gyro, BMI088::GYRO_CHIP_ID, BMI088::GYRO_CHIP_ID_VALUE,
                     BMI088::GYRO_SOFTRESET, BMI088::GYRO_SOFTRESET_VALUE, gyro_settings))
    {
        return false;
    }

    _fifo_enabled = cfg.fifo_en;
    if (_fifo_enabled && !configure_fifo())
        return false;

    // 两路使用同一个 MCU DWT 时钟。FIFO 内样本按各自的标称 ODR 回推。
    _data.timestamp_tick_s = 1.0f / static_cast<float>(SystemCoreClock);
    _acc_clock = {SystemCoreClock / cfg.acc_freq};
    _gyro_clock = {SystemCoreClock / cfg.gyro_freq};
    _acc_mps2_per_lsb = acc_scale;
    _gyro_rps_per_lsb = gyro_scale;
    _ready = true;

    if (cfg.calibrate_flag)
        _ready = calibrate();
    return _ready;
}

bool bmi088_drv_t::read()
{
    if (!_ready)
        return false;
    return _fifo_enabled ? read_fifo() : read_direct();
}

bool bmi088_drv_t::read_direct()
{
    // 陀螺 DRDY 只有短暂的有效窗口。直读模式按配置 ODR 限制陀螺轮询间隔。
    const uint32_t now = pyro::dwt_drv_t::get_current_ticks();
    if (_gyro_clock.valid && now - _gyro_clock.last_timestamp < _gyro_clock.period_ticks)
        return false;

    auto *acc_sample = _data.acc.reserve();
    auto *gyro_sample = _data.gyro.reserve();
    if (acc_sample == nullptr || gyro_sample == nullptr)
        return false;

    uint8_t status = 0U;
    if (!read_sample(_device, sensor_t::accel, BMI088::ACC_STATUS, &status, 1U) ||
        (status & BMI088::ACCEL_DRDY) == 0U)
    {
        return false;
    }

    xyz_t accel;
    int16_t gyro[4];
    uint8_t temp[2];
    // 首个字含 CHIP_ID 和保留字节，后三个字是 XYZ。
    static_assert(BMI088::GYRO_X_L == BMI088::GYRO_CHIP_ID + sizeof(gyro[0]));
    static_assert(sizeof(gyro) == BMI088::GYRO_Z_H - BMI088::GYRO_CHIP_ID + 1U);
    const uint32_t acc_timestamp = pyro::dwt_drv_t::get_current_ticks();
    if (!read_sample(_device, sensor_t::accel, BMI088::ACCEL_XOUT_L, register_bytes(accel), sizeof(accel)))
        return false;

    const uint32_t gyro_timestamp = pyro::dwt_drv_t::get_current_ticks();
    if (!read_sample(_device, sensor_t::gyro, BMI088::GYRO_CHIP_ID, register_bytes(gyro), sizeof(gyro)) ||
        static_cast<uint8_t>(gyro[0]) != BMI088::GYRO_CHIP_ID_VALUE ||
        !read_sample(_device, sensor_t::accel, BMI088::TEMP_M, temp, sizeof(temp)))
    {
        return false;
    }

    decode_accel(*acc_sample, accel, acc_timestamp);
    decode_gyro(*gyro_sample, gyro + 1U, gyro_timestamp);
    _data.temperature = temperature_celsius(temp);
    _data.acc.publish();
    _data.gyro.publish();
    _gyro_clock.last_timestamp = gyro_timestamp;
    _gyro_clock.valid = true;
    return true;
}

uint32_t bmi088_drv_t::sample_clock_t::first_timestamp(uint32_t snapshot, uint16_t frames) const
{
    const uint32_t first = snapshot - (frames - 1U) * period_ticks;
    // 跨批估计可能重叠。维持每路时间戳递增，同时允许 DWT 计数器自然回卷。
    const uint32_t gap = first - last_timestamp;
    return valid && (gap == 0U || gap >= 0x80000000U) ? last_timestamp + 1U : first;
}

bool bmi088_drv_t::configure_fifo()
{
    const register_setting_t accel_settings[] = {
        {BMI088::ACC_FIFO_CONFIG_0, BMI088::ACC_FIFO_STREAM},
        {BMI088::ACC_FIFO_CONFIG_1, BMI088::ACC_FIFO_ACCEL_ON},
    };
    const register_setting_t gyro_settings[] = {
        {BMI088::GYRO_FIFO_CONFIG_1, BMI088::GYRO_FIFO_STREAM_XYZ},
    };

    return apply_settings(_device, sensor_t::accel, accel_settings) &&
           apply_settings(_device, sensor_t::gyro, gyro_settings) &&
           reset_fifo();
}

bool bmi088_drv_t::reset_fifo()
{
    _acc_clock.valid = false;
    _gyro_clock.valid = false;
    const bool acc_ok = _device.write_reg(sensor_t::accel, BMI088::ACC_SOFTRESET, BMI088::ACC_FIFO_RESET);
    // 重写模式会清除陀螺 FIFO 及其溢出标志。
    const bool gyro_ok = _device.write_reg(sensor_t::gyro, BMI088::GYRO_FIFO_CONFIG_1, BMI088::GYRO_FIFO_STREAM_XYZ);
    _fifo_recovery_pending = !acc_ok || !gyro_ok;
    return !_fifo_recovery_pending;
}

bool bmi088_drv_t::read_fifo()
{
    if (_fifo_recovery_pending)
    {
        (void)reset_fifo();
        return false;
    }

    uint16_t acc_status;
    uint8_t gyro_status = 0U;
    const uint32_t acc_snapshot = pyro::dwt_drv_t::get_current_ticks();
    if (!read_sample(_device, sensor_t::accel, BMI088::ACC_FIFO_LENGTH, register_bytes(acc_status), sizeof(acc_status)))
        return false;
    const uint32_t gyro_snapshot = pyro::dwt_drv_t::get_current_ticks();
    if (!read_sample(_device, sensor_t::gyro, BMI088::GYRO_FIFO_STATUS, &gyro_status, 1U))
        return false;

    const uint16_t acc_bytes = acc_status & BMI088::ACC_FIFO_LENGTH_MASK;
    const uint16_t acc_max_frames = acc_bytes / accel_fifo_frame_size;
    const uint16_t gyro_frames = gyro_status & BMI088::GYRO_FIFO_COUNT_MASK;
    if (acc_bytes == 0U && gyro_frames == 0U && (gyro_status & BMI088::GYRO_FIFO_OVERRUN) == 0U)
        return false;

    uint8_t acc_raw[(stream_capacity - 1U) * accel_fifo_frame_size];
    uint16_t acc_payload_offsets[stream_capacity - 1U];
    xyz_t gyro_raw[stream_capacity - 1U];
    const uint16_t cap = _data.cfg.fifo_max_words_per_read;
    // 沿用基类的背压语义。超限时保留软件队列，丢弃器件中积压的一批。
    if ((gyro_status & BMI088::GYRO_FIFO_OVERRUN) != 0U ||
        acc_bytes > sizeof(acc_raw) ||
        acc_max_frames >= _data.acc.capacity - _data.acc.available() ||
        gyro_frames >= _data.gyro.capacity - _data.gyro.available() ||
        (cap != 0U && acc_max_frames + gyro_frames > cap))
    {
        (void)reset_fifo();
        return false;
    }

    uint8_t temp[2];
    if (!read_sample(_device, sensor_t::accel, BMI088::TEMP_M, temp, sizeof(temp)))
        return false;

    // 任一路读中断都可能已弹出部分帧。恢复前不发布半批样本。
    _fifo_recovery_pending = true;
    uint16_t acc_frames = 0U;
    if (acc_bytes != 0U &&
        (!read_sample(_device, sensor_t::accel, BMI088::ACC_FIFO_DATA, acc_raw, acc_bytes) ||
         !scan_accel_fifo(acc_raw, acc_bytes, acc_payload_offsets, acc_frames)))
    {
        return false;
    }
    if (gyro_frames != 0U &&
        !read_sample(_device, sensor_t::gyro, BMI088::GYRO_FIFO_DATA, register_bytes(gyro_raw),
                                  gyro_frames * sizeof(gyro_raw[0])))
    {
        return false;
    }
    _fifo_recovery_pending = false;

    if (acc_frames != 0U)
    {
        uint32_t timestamp = _acc_clock.first_timestamp(acc_snapshot, acc_frames);
        for (uint16_t i = 0U; i < acc_frames; ++i)
        {
            xyz_t xyz;
            std::memcpy(xyz, acc_raw + acc_payload_offsets[i], sizeof(xyz));
            decode_accel(*_data.acc.reserve(), xyz, timestamp);
            _data.acc.publish();
            timestamp += _acc_clock.period_ticks;
        }
        _acc_clock.last_timestamp = timestamp - _acc_clock.period_ticks;
        _acc_clock.valid = true;
    }
    if (gyro_frames != 0U)
    {
        uint32_t timestamp = _gyro_clock.first_timestamp(gyro_snapshot, gyro_frames);
        for (uint16_t i = 0U; i < gyro_frames; ++i)
        {
            decode_gyro(*_data.gyro.reserve(), gyro_raw[i], timestamp);
            _data.gyro.publish();
            timestamp += _gyro_clock.period_ticks;
        }
        _gyro_clock.last_timestamp = timestamp - _gyro_clock.period_ticks;
        _gyro_clock.valid = true;
    }

    if (acc_frames == 0U && gyro_frames == 0U)
        return false;
    _data.temperature = temperature_celsius(temp);
    return true;
}

void bmi088_drv_t::decode_accel(acc_sample_t &sample, const int16_t (&axes)[3], uint32_t timestamp) const
{
    for (uint8_t axis = 0U; axis < 3U; ++axis)
        sample.Accel[axis] = axes[axis] * _acc_mps2_per_lsb;
    sample.timestamp = timestamp;
}

void bmi088_drv_t::decode_gyro(gyro_sample_t &sample, const int16_t *axes, uint32_t timestamp) const
{
    const float bias[] = {_data.cali.gx_offset, _data.cali.gy_offset, _data.cali.gz_offset};
    for (uint8_t axis = 0U; axis < 3U; ++axis)
        sample.Gyro[axis] = axes[axis] * _gyro_rps_per_lsb - bias[axis];
    sample.timestamp = timestamp;
}

namespace
{

float temperature_celsius(const uint8_t (&raw)[2])
{
    int16_t value = static_cast<int16_t>((raw[0] << 3U) | (raw[1] >> 5U));
    if (value > 1023)
        value -= 2048;
    return value * 0.125f + 23.0f; // 0.125 °C/LSB，原码 0 对应 23 °C。
}

bool read_sample(bmi088_t &device, sensor_t sensor, uint8_t reg, uint8_t *data, uint16_t size)
{
    constexpr uint32_t dma_timeout_ms = 2U;
    switch (device.read_regs_request(sensor, reg, size))
    {
        case bmi088_t::request_result_t::started:
            return device.wait_dma(dma_timeout_ms) && device.read_regs_collect(data, size);
        case bmi088_t::request_result_t::unsupported:
            return device.read_regs(sensor, reg, data, size);
        default:
            return false;
    }
}

// 复位、写入和解包辅助函数。
template <std::size_t N>
bool apply_settings(bmi088_t &device, sensor_t sensor, const register_setting_t (&settings)[N])
{
    for (const auto &setting : settings)
    {
        uint8_t value = 0U;
        if (!device.write_reg(sensor, setting.address, setting.value))
            return false;
        HAL_Delay(1U);
        if (!device.read_regs(sensor, setting.address, &value, 1U) || value != setting.value)
            return false;
    }
    return true;
}

template <std::size_t N>
bool init_sensor(bmi088_t &device, sensor_t sensor, uint8_t chip_id_reg, uint8_t chip_id,
                    uint8_t reset_reg, uint8_t reset_value, const register_setting_t (&settings)[N])
{
    uint8_t value = 0U;
    // 加速度计用第一次读事务的 CS 上升沿选择 SPI。软复位后重复该时序。
    if (!device.read_regs(sensor, chip_id_reg, &value, 1U))
        return false;
    HAL_Delay(1U);
    if (!device.read_regs(sensor, chip_id_reg, &value, 1U) || value != chip_id ||
        !device.write_reg(sensor, reset_reg, reset_value))
    {
        return false;
    }
    HAL_Delay(80U); // 两路软复位的统一等待时间。
    if (!device.read_regs(sensor, chip_id_reg, &value, 1U))
        return false;
    HAL_Delay(1U);

    return device.read_regs(sensor, chip_id_reg, &value, 1U) && value == chip_id &&
           apply_settings(device, sensor, settings);
}

// 扫描时记录 XYZ 偏移，整批通过后再复制到对齐的寄存器镜像。
// 调用者已限制字节数，payload_offsets 能容纳所有完整的 XYZ 帧。
bool scan_accel_fifo(const uint8_t *raw, uint16_t bytes, uint16_t *payload_offsets, uint16_t &frames)
{
    constexpr uint8_t header_mask = 0xFCU;
    constexpr uint8_t xyz_header = 0x84U;
    constexpr uint8_t config_header = 0x48U;

    frames = 0U;
    for (uint16_t offset = 0U; offset < bytes;)
    {
        const uint8_t header = raw[offset] & header_mask;
        const uint16_t remaining = bytes - offset;
        if (header == config_header && remaining >= 2U)
        {
            offset += 2U;
            continue;
        }
        if (header != xyz_header || remaining < accel_fifo_frame_size)
            return false;

        payload_offsets[frames++] = offset + 1U;
        offset += accel_fifo_frame_size;
    }
    return true;
}

// 物理量到寄存器值的映射。陀螺仪寄存器同时决定 ODR 和低通带宽。
bool accel_odr_code(uint16_t hz, uint8_t &code)
{
    switch (hz)
    {
        case 25U:   code = BMI088::ACC_25_HZ;   return true;
        case 50U:   code = BMI088::ACC_50_HZ;   return true;
        case 100U:  code = BMI088::ACC_100_HZ;  return true;
        case 200U:  code = BMI088::ACC_200_HZ;  return true;
        case 400U:  code = BMI088::ACC_400_HZ;  return true;
        case 800U:  code = BMI088::ACC_800_HZ;  return true;
        case 1600U: code = BMI088::ACC_1600_HZ; return true;
        default: return false;
    }
}

bool gyro_odr_code(uint16_t hz, uint8_t &code)
{
    switch (hz)
    {
        case 100U:  code = BMI088::GYRO_100_12_HZ;    return true;
        case 200U:  code = BMI088::GYRO_200_23_HZ;    return true;
        case 400U:  code = BMI088::GYRO_400_47_HZ;    return true;
        case 1000U: code = BMI088::GYRO_1000_116_HZ;  return true;
        case 2000U: code = BMI088::GYRO_2000_532_HZ;  return true;
        default: return false;
    }
}

bool accel_range_code(uint16_t g, uint8_t &code, float &mps2_per_lsb)
{
    switch (g)
    {
        case 3U:  code = BMI088::ACC_RANGE_3G;  break;
        case 6U:  code = BMI088::ACC_RANGE_6G;  break;
        case 12U: code = BMI088::ACC_RANGE_12G; break;
        case 24U: code = BMI088::ACC_RANGE_24G; break;
        default: return false;
    }

    // 保留 ±3 g 的单独系数，其余档位按 ±6 g 的系数缩放。
    constexpr float scale_6g = 0.00179443359375f;
    mps2_per_lsb = g == 3U ? 0.0008974358974f : scale_6g * (g / 6.0f);
    return true;
}

bool gyro_range_code(uint16_t dps, uint8_t &code, float &rps_per_lsb)
{
    switch (dps)
    {
        case 125U:  code = BMI088::GYRO_125;  break;
        case 250U:  code = BMI088::GYRO_250;  break;
        case 500U:  code = BMI088::GYRO_500;  break;
        case 1000U: code = BMI088::GYRO_1000; break;
        case 2000U: code = BMI088::GYRO_2000; break;
        default: return false;
    }

    constexpr float scale_2000dps = 0.0010652644360316953f;
    rps_per_lsb = scale_2000dps * (dps / 2000.0f);
    return true;
}

} // namespace
