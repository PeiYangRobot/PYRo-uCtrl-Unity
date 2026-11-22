#ifndef BMI088_REG_H
#define BMI088_REG_H

#include <cstdint>

namespace BMI088
{

// 加速度计寄存器与字段。
constexpr uint8_t ACC_CHIP_ID = 0x00;
constexpr uint8_t ACC_CHIP_ID_VALUE = 0x1E;

constexpr uint8_t ACC_ERR_REG = 0x02;
constexpr uint8_t ACCEL_CONFIG_ERROR_POS = 0x2;
constexpr uint8_t ACCEL_CONFIG_ERROR = (1 << ACCEL_CONFIG_ERROR_POS);
constexpr uint8_t FATAL_ERROR_POS = 0x0;
constexpr uint8_t FATAL_ERROR = (1U << FATAL_ERROR_POS);

constexpr uint8_t ACC_STATUS = 0x03;
constexpr uint8_t ACCEL_DRDY_POS = 0x7;
constexpr uint8_t ACCEL_DRDY = (1 << ACCEL_DRDY_POS);

constexpr uint8_t ACCEL_XOUT_L = 0x12;
constexpr uint8_t ACCEL_XOUT_M = 0x13;
constexpr uint8_t ACCEL_YOUT_L = 0x14;
constexpr uint8_t ACCEL_YOUT_M = 0x15;
constexpr uint8_t ACCEL_ZOUT_L = 0x16;
constexpr uint8_t ACCEL_ZOUT_M = 0x17;

constexpr uint8_t SENSORTIME_DATA_L = 0x18;
constexpr uint8_t SENSORTIME_DATA_M = 0x19;
constexpr uint8_t SENSORTIME_DATA_H = 0x1A;

constexpr uint8_t ACC_INT_STAT_1 = 0x1D;
constexpr uint8_t ACCEL_DRDY_INTERRUPT_POS = 0x7;
constexpr uint8_t ACCEL_DRDY_INTERRUPT = (1 << ACCEL_DRDY_INTERRUPT_POS);

constexpr uint8_t TEMP_M = 0x22;

constexpr uint8_t TEMP_L = 0x23;

constexpr uint8_t ACC_FIFO_LENGTH = 0x24U;
constexpr uint16_t ACC_FIFO_LENGTH_MASK = 0x3FFFU;
constexpr uint8_t ACC_FIFO_DATA = 0x26U;
constexpr uint8_t ACC_FIFO_CONFIG_0 = 0x48U;
constexpr uint8_t ACC_FIFO_STREAM = 0x02U;
constexpr uint8_t ACC_FIFO_CONFIG_1 = 0x49U;
constexpr uint8_t ACC_FIFO_ACCEL_ON = 0x50U;
constexpr uint8_t ACC_FIFO_RESET = 0xB0U;
constexpr uint8_t ACC_CONF = 0x40;
constexpr uint8_t ACC_CONF_MUST_SET = 0x80;
constexpr uint8_t ACC_BWP_POS = 0x4;
constexpr uint8_t ACC_OSR4 = (0x0 << ACC_BWP_POS);
constexpr uint8_t ACC_OSR2 = (0x1 << ACC_BWP_POS);
constexpr uint8_t ACC_NORMAL = (0x2 << ACC_BWP_POS);

constexpr uint8_t ACC_ODR_POS = 0x0;
constexpr uint8_t ACC_12_5_HZ = (0x5 << ACC_ODR_POS);
constexpr uint8_t ACC_25_HZ = (0x6 << ACC_ODR_POS);
constexpr uint8_t ACC_50_HZ = (0x7 << ACC_ODR_POS);
constexpr uint8_t ACC_100_HZ = (0x8 << ACC_ODR_POS);
constexpr uint8_t ACC_200_HZ = (0x9 << ACC_ODR_POS);
constexpr uint8_t ACC_400_HZ = (0xA << ACC_ODR_POS);
constexpr uint8_t ACC_800_HZ = (0xB << ACC_ODR_POS);
constexpr uint8_t ACC_1600_HZ = (0xC << ACC_ODR_POS);

constexpr uint8_t ACC_RANGE = 0x41;

constexpr uint8_t ACC_RANGE_POS = 0x0;
constexpr uint8_t ACC_RANGE_3G = (0x0 << ACC_RANGE_POS);
constexpr uint8_t ACC_RANGE_6G = (0x1 << ACC_RANGE_POS);
constexpr uint8_t ACC_RANGE_12G = (0x2 << ACC_RANGE_POS);
constexpr uint8_t ACC_RANGE_24G = (0x3 << ACC_RANGE_POS);

constexpr uint8_t INT1_IO_CTRL = 0x53;
constexpr uint8_t ACC_INT1_IO_ENABLE_POS = 0x3;
constexpr uint8_t ACC_INT1_IO_ENABLE = (0x1 << ACC_INT1_IO_ENABLE_POS);
constexpr uint8_t ACC_INT1_GPIO_MODE_POS = 0x2;
constexpr uint8_t ACC_INT1_GPIO_PP = (0x0 << ACC_INT1_GPIO_MODE_POS);
constexpr uint8_t ACC_INT1_GPIO_OD = (0x1 << ACC_INT1_GPIO_MODE_POS);
constexpr uint8_t ACC_INT1_GPIO_LVL_POS = 0x1;
constexpr uint8_t ACC_INT1_GPIO_LOW = (0x0 << ACC_INT1_GPIO_LVL_POS);
constexpr uint8_t ACC_INT1_GPIO_HIGH = (0x1 << ACC_INT1_GPIO_LVL_POS);

constexpr uint8_t INT2_IO_CTRL = 0x54;
constexpr uint8_t ACC_INT2_IO_ENABLE_POS = 0x3;
constexpr uint8_t ACC_INT2_IO_ENABLE = (0x1 << ACC_INT2_IO_ENABLE_POS);
constexpr uint8_t ACC_INT2_GPIO_MODE_POS = 0x2;
constexpr uint8_t ACC_INT2_GPIO_PP = (0x0 << ACC_INT2_GPIO_MODE_POS);
constexpr uint8_t ACC_INT2_GPIO_OD = (0x1 << ACC_INT2_GPIO_MODE_POS);
constexpr uint8_t ACC_INT2_GPIO_LVL_POS = 0x1;
constexpr uint8_t ACC_INT2_GPIO_LOW = (0x0 << ACC_INT2_GPIO_LVL_POS);
constexpr uint8_t ACC_INT2_GPIO_HIGH = (0x1 << ACC_INT2_GPIO_LVL_POS);

constexpr uint8_t INT_MAP_DATA = 0x58;
constexpr uint8_t ACC_INT2_DRDY_INTERRUPT_POS = 0x6;
constexpr uint8_t ACC_INT2_DRDY_INTERRUPT = (0x1 << ACC_INT2_DRDY_INTERRUPT_POS);
constexpr uint8_t ACC_INT1_DRDY_INTERRUPT_POS = 0x2;
constexpr uint8_t ACC_INT1_DRDY_INTERRUPT = (0x1 << ACC_INT1_DRDY_INTERRUPT_POS);

constexpr uint8_t ACC_SELF_TEST = 0x6D;
constexpr uint8_t ACC_SELF_TEST_OFF = 0x00;
constexpr uint8_t ACC_SELF_TEST_POSITIVE_SIGNAL = 0x0D;
constexpr uint8_t ACC_SELF_TEST_NEGATIVE_SIGNAL = 0x09;

constexpr uint8_t ACC_PWR_CONF = 0x7C;
constexpr uint8_t ACC_PWR_SUSPEND_MODE = 0x03;
constexpr uint8_t ACC_PWR_ACTIVE_MODE = 0x00;

constexpr uint8_t ACC_PWR_CTRL = 0x7D;
constexpr uint8_t ACC_ENABLE_ACC_OFF = 0x00;
constexpr uint8_t ACC_ENABLE_ACC_ON = 0x04;

constexpr uint8_t ACC_SOFTRESET = 0x7E;
constexpr uint8_t ACC_SOFTRESET_VALUE = 0xB6;

// 陀螺仪寄存器与字段。
constexpr uint8_t GYRO_CHIP_ID = 0x00;
constexpr uint8_t GYRO_CHIP_ID_VALUE = 0x0F;

constexpr uint8_t GYRO_X_L = 0x02;
constexpr uint8_t GYRO_X_H = 0x03;
constexpr uint8_t GYRO_Y_L = 0x04;
constexpr uint8_t GYRO_Y_H = 0x05;
constexpr uint8_t GYRO_Z_L = 0x06;
constexpr uint8_t GYRO_Z_H = 0x07;

constexpr uint8_t GYRO_INT_STAT_1 = 0x0A;
constexpr uint8_t GYRO_DYDR_POS = 0x7;
constexpr uint8_t GYRO_DYDR = (0x1 << GYRO_DYDR_POS);

constexpr uint8_t GYRO_FIFO_STATUS = 0x0EU;
constexpr uint8_t GYRO_FIFO_OVERRUN = 0x80U;
constexpr uint8_t GYRO_FIFO_COUNT_MASK = 0x7FU;
constexpr uint8_t GYRO_RANGE = 0x0F;
constexpr uint8_t GYRO_RANGE_POS = 0x0;
constexpr uint8_t GYRO_2000 = (0x0 << GYRO_RANGE_POS);
constexpr uint8_t GYRO_1000 = (0x1 << GYRO_RANGE_POS);
constexpr uint8_t GYRO_500 = (0x2 << GYRO_RANGE_POS);
constexpr uint8_t GYRO_250 = (0x3 << GYRO_RANGE_POS);
constexpr uint8_t GYRO_125 = (0x4 << GYRO_RANGE_POS);

constexpr uint8_t GYRO_BANDWIDTH = 0x10;
constexpr uint8_t GYRO_BANDWIDTH_MUST_SET = 0x80;
constexpr uint8_t GYRO_2000_532_HZ = 0x00;
constexpr uint8_t GYRO_2000_230_HZ = 0x01;
constexpr uint8_t GYRO_1000_116_HZ = 0x02;
constexpr uint8_t GYRO_400_47_HZ = 0x03;
constexpr uint8_t GYRO_200_23_HZ = 0x04;
constexpr uint8_t GYRO_100_12_HZ = 0x05;
constexpr uint8_t GYRO_200_64_HZ = 0x06;
constexpr uint8_t GYRO_100_32_HZ = 0x07;

constexpr uint8_t GYRO_LPM1 = 0x11;
constexpr uint8_t GYRO_NORMAL_MODE = 0x00;
constexpr uint8_t GYRO_SUSPEND_MODE = 0x80;
constexpr uint8_t GYRO_DEEP_SUSPEND_MODE = 0x20;

constexpr uint8_t GYRO_SOFTRESET = 0x14;
constexpr uint8_t GYRO_SOFTRESET_VALUE = 0xB6;

constexpr uint8_t GYRO_CTRL = 0x15;
constexpr uint8_t DRDY_OFF = 0x00;
constexpr uint8_t DRDY_ON = 0x80;

constexpr uint8_t GYRO_INT3_INT4_IO_CONF = 0x16;
constexpr uint8_t GYRO_INT4_GPIO_MODE_POS = 0x3;
constexpr uint8_t GYRO_INT4_GPIO_PP = (0x0 << GYRO_INT4_GPIO_MODE_POS);
constexpr uint8_t GYRO_INT4_GPIO_OD = (0x1 << GYRO_INT4_GPIO_MODE_POS);
constexpr uint8_t GYRO_INT4_GPIO_LVL_POS = 0x2;
constexpr uint8_t GYRO_INT4_GPIO_LOW = (0x0 << GYRO_INT4_GPIO_LVL_POS);
constexpr uint8_t GYRO_INT4_GPIO_HIGH = (0x1 << GYRO_INT4_GPIO_LVL_POS);
constexpr uint8_t GYRO_INT3_GPIO_MODE_POS = 0x1;
constexpr uint8_t GYRO_INT3_GPIO_PP = (0x0 << GYRO_INT3_GPIO_MODE_POS);
constexpr uint8_t GYRO_INT3_GPIO_OD = (0x1 << GYRO_INT3_GPIO_MODE_POS);
constexpr uint8_t GYRO_INT3_GPIO_LVL_POS = 0x0;
constexpr uint8_t GYRO_INT3_GPIO_LOW = (0x0 << GYRO_INT3_GPIO_LVL_POS);
constexpr uint8_t GYRO_INT3_GPIO_HIGH = (0x1 << GYRO_INT3_GPIO_LVL_POS);

constexpr uint8_t GYRO_INT3_INT4_IO_MAP = 0x18;

constexpr uint8_t GYRO_DRDY_IO_OFF = 0x00;
constexpr uint8_t GYRO_DRDY_IO_INT3 = 0x01;
constexpr uint8_t GYRO_DRDY_IO_INT4 = 0x80;
constexpr uint8_t GYRO_DRDY_IO_BOTH = (GYRO_DRDY_IO_INT3 | GYRO_DRDY_IO_INT4);

constexpr uint8_t GYRO_FIFO_CONFIG_1 = 0x3EU;
constexpr uint8_t GYRO_FIFO_STREAM_XYZ = 0x88U;
constexpr uint8_t GYRO_FIFO_DATA = 0x3FU;

constexpr uint8_t GYRO_SELF_TEST = 0x3C;
constexpr uint8_t GYRO_RATE_OK_POS = 0x4;
constexpr uint8_t GYRO_RATE_OK = (0x1 << GYRO_RATE_OK_POS);
constexpr uint8_t GYRO_BIST_FAIL_POS = 0x2;
constexpr uint8_t GYRO_BIST_FAIL = (0x1 << GYRO_BIST_FAIL_POS);
constexpr uint8_t GYRO_BIST_RDY_POS = 0x1;
constexpr uint8_t GYRO_BIST_RDY = (0x1 << GYRO_BIST_RDY_POS);
constexpr uint8_t GYRO_TRIG_BIST_POS = 0x0;
constexpr uint8_t GYRO_TRIG_BIST = (0x1 << GYRO_TRIG_BIST_POS);

} // namespace BMI088

#endif
